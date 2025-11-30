/*
 * storage.cpp
 *
 *  Created on: Jul 30, 2025
 *      Author: naoki
 */

#include <string.h>
#include <inttypes.h>

#include <algorithm>

#include "analog3/stm32impl.h"
#include "analog3/storage.h"

const uint16_t kMetadataOffset = 0x7f8;  // the last row of a page
const uint16_t kNullSeqNumber = 0xffff;
const uint16_t kLastSeqNumber = 0xfffe;

inline uint64_t FlashRead64(uint32_t address) {
  return *(__IO uint64_t*) (address);
}

inline uint32_t FlashRead32(uint32_t address) {
  return *(__IO uint32_t*) (address);
}

inline uint16_t FlashRead16(uint32_t address) {
  return *(__IO uint16_t*) (address);
}

inline uint8_t FlashRead8(uint32_t address) {
  return *(__IO uint8_t*) (address);
}

inline void UnlockFlash() {
  HAL_StatusTypeDef status;
  if ((status = HAL_FLASH_Unlock()) != HAL_OK) {
    HandleError("Failed to unlock flash; status=%d", status);
  }
}

inline void LockFlash() {
  HAL_StatusTypeDef status;
  if ((status = HAL_FLASH_Lock()) != HAL_OK) {
    HandleError("Failed to lock flash; status=%d", status);
  }
}

union Buffer {
  uint64_t u64;
  uint8_t u8v[8];
};

static struct Storage {
  uint32_t page_address;
  uint32_t page;
  uint16_t page_sequence_number;
} storage;

static inline uint64_t GetRow(uint16_t address) {
  return FlashRead64(storage.page_address + address % 8);
}

static bool AreRowsClean(uint16_t address, size_t size) {
  uint16_t row_address = address / 8 * 8;
  do {
    if (FlashRead64(storage.page_address + row_address) != 0xffffffffffffffff) {
      return false;
    }
    row_address += 8;
  } while (row_address < address + size);
  return true;
}

template<typename T>
static uint64_t MergeValue(uint16_t address, T data, uint64_t original_row) {
  size_t size = sizeof(T);
  uint32_t offset = address % 8;
  // assert that the data should fit within the row
  if (offset + size > 8) {
    HandleError("Data out of bounds; offset=%lx, size=%zu", offset, size);
  }
  T mask = ~0;
  uint64_t overlay = static_cast<uint64_t>(mask) << (offset * 8);
  uint64_t new_row = original_row & ~overlay;
  overlay = static_cast<uint64_t>(data) << (offset * 8);
  new_row |= overlay;
  return new_row;
}

template<typename T>
static void FlashWriteInt(uint16_t address, T data, uint64_t original_row) {
  uint64_t value = MergeValue(address, data, original_row);
  uint32_t row_address = storage.page_address + address / 8 * 8;
  UnlockFlash();
  HAL_StatusTypeDef status;
  if ((status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, row_address, value)) != HAL_OK) {
    HAL_FLASH_Lock();
    HandleError("Failed to program flash; status=%d, address=%lx, value=%" PRIx64, status,
                row_address, value);
  }
  LockFlash();
}

static uint32_t SwitchPages() {
  // switch pages
  uint32_t next_page;
  uint32_t next_page_address;
  if (storage.page == STORAGE_PAGE_0) {
    next_page = STORAGE_PAGE_1;
    next_page_address = STORAGE_PAGE_1_ADDR;
  } else {
    next_page = STORAGE_PAGE_0;
    next_page_address = STORAGE_PAGE_0_ADDR;
  }
  uint32_t prev_page_address = storage.page_address;
  uint16_t next_seq =
      storage.page_sequence_number == kLastSeqNumber ? 0 : storage.page_sequence_number + 1;
  storage.page = next_page;
  storage.page_address = next_page_address;

  // clear the next page
  UnlockFlash();
  FLASH_EraseInitTypeDef erase_init = { .TypeErase = FLASH_TYPEERASE_PAGES, .Page = next_page,
      .NbPages = 1, };
  uint32_t page_error;
  HAL_StatusTypeDef status;
  if ((status = HAL_FLASHEx_Erase(&erase_init, &page_error)) != HAL_OK) {
    HAL_FLASH_Lock();
    HandleError("Failed to erase flash page; status=%d, error=%lu", status, page_error);
  }
  LockFlash();

  // initialize the next page
  FlashWriteInt(kMetadataOffset, next_seq, 0xffffffffffffffff);

  return prev_page_address;
}

template<typename T>
static void PingPongWriteInt(uint16_t address, T data, uint64_t original_row) {
  // switch pages
  uint32_t prev_page_address = SwitchPages();

  // copy rows
  uint16_t target_row = address / 8 * 8;
  UnlockFlash();
  HAL_StatusTypeDef status;
  for (uint16_t row = 0; row < FLASH_PAGE_SIZE - 8; row += 8) {
    uint64_t value = FlashRead64(prev_page_address + row);
    if (row == target_row) {
      value = MergeValue(address, data, value);
    }
    if (value != 0xffffffffffffffff
        && (status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, storage.page_address + row,
                                       value)) != HAL_OK) {
    HAL_FLASH_Lock();
    HandleError("Failed to copy a row; status=%d, prev_page_addr=%lx, next_page_addr=%lx, row=%u",
                  status, prev_page_address, storage.page_address, row);
    }
  }
  LockFlash();
}

static bool IsPage0(uint16_t seq0, uint16_t seq1) {
  if (seq1 == kNullSeqNumber) {
    return true;
  }
  if (seq1 == kLastSeqNumber && seq0 == 0) {
    return true;
  }
  if (seq0 == kLastSeqNumber && seq1 == 0) {
    return false;
  }
  return seq0 > seq1;
}

template<typename T>
inline static void SaveInt(uint16_t address, T data) {
  auto row = GetRow(address);
  if (row == 0xffffffffffffffff) {
    FlashWriteInt(address, data, row);
  } else {
    PingPongWriteInt(address, data, row);
  }
}

class StoreStream {
 private:
  union Buffer buffer_;
  Storage *storage_;
  uint32_t src_page_address_;

  uint16_t row_;
  uint16_t offset_;

 public:
  StoreStream(uint16_t address, Storage *storage, uint32_t src_page_address = 0xffffffff)
      :
      storage_ { storage },
      src_page_address_(src_page_address) {
    offset_ = address % 8;
    row_ = address - offset_;
    if (src_page_address_ != 0xffffffff) {
      buffer_.u64 = FlashRead64(src_page_address_ + row_);
    } else {
      buffer_.u64 = 0xffffffffffffffff;
    }
  }

  uint16_t GetRow() const {
    return row_;
  }

  void PutData(const void *data, size_t size) {
    size_t remaining = size;
    while (remaining > 0) {
      size_t bytes_to_write = std::min(static_cast<size_t>(8 - offset_), size);
      memcpy(buffer_.u8v + offset_, data, bytes_to_write);
      remaining -= bytes_to_write;
      offset_ += bytes_to_write;
      CheckOffset();
    }
  }

  /**
   * Flush remaining data into the storage.
   */
  void Flush() {
    if (offset_ > 0) {
      HAL_StatusTypeDef status;
      if ((status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, storage_->page_address + row_,
                                      buffer_.u64)) != HAL_OK) {
        HandleError("Failed to flush data; status=%d", status);
      }
    }
  }

 private:
  void CheckOffset() {
    if (offset_ == 8) {
      HAL_StatusTypeDef status;
      if ((status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, storage_->page_address + row_,
                                      buffer_.u64)) != HAL_OK) {
        HandleError("Failed to flush data; status=%d", status);
      }
      row_ += 8;
      offset_ = 0;
      if (src_page_address_ != 0xffffffff) {
        buffer_.u64 = FlashRead64(src_page_address_ + row_);
      } else {
        buffer_.u64 = 0xffffffffffffffff;
      }
    }
  }
};

// API functions /////////////////////////////////////////////////////////////

void InitializeStorage() {
  uint16_t seq0 = FlashRead16(STORAGE_PAGE_0_ADDR + kMetadataOffset);
  uint16_t seq1 = FlashRead16(STORAGE_PAGE_1_ADDR + kMetadataOffset);
  if (IsPage0(seq0, seq1)) {
    storage.page_address = STORAGE_PAGE_0_ADDR;
    storage.page = STORAGE_PAGE_0;
    if (seq0 == kNullSeqNumber) {
      storage.page_sequence_number = 0;
      FlashWriteInt(kMetadataOffset, storage.page_sequence_number, ~0);
    } else {
      storage.page_sequence_number = seq0;
    }
  } else {
    storage.page_address = STORAGE_PAGE_1_ADDR;
    storage.page = STORAGE_PAGE_1;
    storage.page_sequence_number = seq1;
  }
}

void Save64(uint16_t address, uint64_t data) {
  SaveInt(address, data);
}

uint64_t Load64(uint16_t address) {
  return FlashRead64(storage.page_address + address);
}

void Save32(uint16_t address, uint32_t data) {
  SaveInt(address, data);
}

uint32_t Load32(uint16_t address) {
  return FlashRead32(storage.page_address + address);
}

void Save16(uint16_t address, uint16_t data) {
  SaveInt(address, data);
}

uint16_t Load16(uint16_t address) {
  return FlashRead16(storage.page_address + address);
}

void Save8(uint16_t address, uint8_t data) {
  SaveInt(address, data);
}

uint8_t Load8(uint16_t address) {
  return FlashRead8(storage.page_address + address);
}

void SaveString(uint16_t address, const char *data, size_t max_length) {
  uint8_t length = static_cast<uint8_t>(std::min(strlen(data), max_length - 1));
  UnlockFlash();
  HAL_StatusTypeDef status;
  if (AreRowsClean(address, static_cast<size_t>(length + 1))) {
    StoreStream ostream { address, &storage };
    ostream.PutData(&length, sizeof(length));
    ostream.PutData(data, length);
    ostream.Flush();
  } else {
    uint32_t prev_page_address = SwitchPages();
    StoreStream pipe_stream { address, &storage, prev_page_address };
    for (uint16_t row = 0; row < FLASH_PAGE_SIZE - 8; row += 8) {
      if (row == pipe_stream.GetRow()) {
        pipe_stream.PutData(&length, sizeof(length));
        pipe_stream.PutData(data, length);
        pipe_stream.Flush();
        row = pipe_stream.GetRow();
      } else {
        uint64_t value = FlashRead64(prev_page_address + row);
        if (value != 0xffffffffffffffff
            && (status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, storage.page_address + row,
                                           value)) != HAL_OK) {
        HAL_FLASH_Lock();
        HandleError("Failed to store string; status=%d, src_addr=%lx, dst_addr=%lx, row=%u",
                      status, prev_page_address, storage.page_address, row);
        }
      }
    }
  }
  LockFlash();
}

void LoadString(uint16_t address, char *data, size_t max_length) {
  uint8_t length = FlashRead8(storage.page_address + address);
  if (length == 0xff) {
    // the string is not stored
    data[0] = 0;
    return;
  }
  ++address;
  if (length > max_length - 1) {
    length = max_length - 1;
  }
  size_t pos = 0;
  union Buffer buffer;
  size_t offset = address % 8;
  address -= offset;
  while (pos < length) {
    buffer.u64 = FlashRead64(storage.page_address + address);
    auto copy_len = std::min(8 - offset, length - pos);
    memcpy(data + pos, buffer.u8v + offset, copy_len);
    pos += copy_len;
    address += 8;
    offset = 0;
  }
  data[pos] = 0;
}
