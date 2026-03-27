use defmt::{debug, trace};
use embassy_executor::Spawner;
use embassy_stm32::flash::{Error, FLASH_SIZE, Flash, WRITE_SIZE};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, channel::Channel, signal::Signal};
use heapless::{String, Vec};

use crate::analog3::{
    definitions::{A3_MAX_PROP_DATA_SIZE, MAX_PROP_VECTOR_LENGTH},
    {Value, ValueType},
};

pub const PAGE_SIZE: usize = 0x800;
const NUM_ROWS: usize = PAGE_SIZE / WRITE_SIZE - 1;
// FLASH_SIZE is determined by the Embassy HAL so that the pages are guaranteed to locate
// at the tail of available flash memory. But we still need to avoid carefully having
// the pages conflict with the program code.
const PAGE_0: u32 = (FLASH_SIZE - 2 * PAGE_SIZE) as u32;
const PAGE_1: u32 = (FLASH_SIZE - PAGE_SIZE) as u32;
const METADATA_OFFSET: u32 = (PAGE_SIZE - WRITE_SIZE) as u32; // the last row of the page
const NULL_SEQ_NUMBER: u16 = u16::MAX;
const LAST_SEQ_NUMBER: u16 = u16::MAX - 1;

static CHANNEL_STORAGE: Channel<ThreadModeRawMutex, StorageRequest, 2> = Channel::new();

pub async fn start(spawner: Spawner, flash: Flash<'static>) {
    spawner.spawn(run_storage(flash).unwrap());
}

pub async fn load(
    address: u16,
    value_type: ValueType,
    reply: &'static Signal<ThreadModeRawMutex, Result<Value, Error>>,
) -> Result<Value, Error> {
    let request_sender = CHANNEL_STORAGE.sender();
    request_sender
        .send(StorageRequest::Load {
            address,
            value_type,
            reply,
        })
        .await;
    reply.wait().await
}

pub async fn save(
    address: u16,
    value: Value,
    reply: &'static Signal<ThreadModeRawMutex, Result<Value, Error>>,
) -> Result<(), Error> {
    let request_sender = CHANNEL_STORAGE.sender();
    request_sender
        .send(StorageRequest::Save {
            address,
            value,
            reply,
        })
        .await;
    reply.wait().await.map(|_| {})
}

/// Loads a U8 value from the flash memory.
pub async fn load_u8(
    address: u16,
    reply: &'static Signal<ThreadModeRawMutex, Result<Value, Error>>,
) -> u8 {
    let Value::U8(value) = load(address, ValueType::U8, reply).await.unwrap() else {
        panic!("wrong type returned"); // it's a bug if this happens.
    };
    value
}

/// Loads a U16 value from the flash memory.
pub async fn load_u16(
    address: u16,
    reply: &'static Signal<ThreadModeRawMutex, Result<Value, Error>>,
) -> u16 {
    let Value::U16(value) = load(address, ValueType::U16, reply).await.unwrap() else {
        panic!("wrong type returned"); // it's a bug if this happens.
    };
    value
}

/// Loads a U16 value from the flash memory. If the read value is 0xffff, the method
/// considers it "no value" and returns the default value.
pub async fn load_u16_or_default(
    address: u16,
    reply: &'static Signal<ThreadModeRawMutex, Result<Value, Error>>,
    default: u16,
) -> u16 {
    let Value::U16(value) = load(address, ValueType::U16, reply).await.unwrap() else {
        panic!("wrong type returned"); // it's a bug if this happens.
    };
    if value == u16::MAX { default } else { value }
}

/// Loads a U32 value from the flash memory.
pub async fn load_u32(
    address: u16,
    reply: &'static Signal<ThreadModeRawMutex, Result<Value, Error>>,
) -> u32 {
    let Value::U32(value) = load(address, ValueType::U32, reply).await.unwrap() else {
        panic!("wrong type returned"); // it's a bug if this happens.
    };
    value
}

/// Loads a string from the flash memory.
pub async fn load_string(
    address: u16,
    reply: &'static Signal<ThreadModeRawMutex, Result<Value, Error>>,
) -> String<A3_MAX_PROP_DATA_SIZE> {
    let Value::Text(name) = load(address, ValueType::Text, &reply).await.unwrap() else {
        panic!("wrong type returned"); // it's a bug if this happens
    };
    name
}

enum StorageRequest {
    Load {
        address: u16,
        value_type: ValueType,
        reply: &'static Signal<ThreadModeRawMutex, Result<Value, Error>>,
    },
    Save {
        address: u16,
        value: Value,
        reply: &'static Signal<ThreadModeRawMutex, Result<Value, Error>>,
    },
}

#[embassy_executor::task]
async fn run_storage(flash: Flash<'static>) {
    let mut storage = Storage::init(flash).await.unwrap();
    storage.run().await;
}

#[non_exhaustive]
pub struct Storage {
    flash: Flash<'static>,

    // page specifiers
    page_offset: u32,
    sequence_number: u16,
}

impl Storage {
    pub async fn init(mut flash: Flash<'static>) -> Result<Self, Error> {
        let (page_offset, sequence_number) = {
            let offset0 = PAGE_0 + METADATA_OFFSET;
            let offset1 = PAGE_1 + METADATA_OFFSET;
            let seq0 = Self::read16(&mut flash, offset0)?;
            let seq1 = Self::read16(&mut flash, offset1)?;
            if Self::is_page_0(seq0, seq1) {
                let seq = {
                    if seq0 == NULL_SEQ_NUMBER {
                        // assuming this is a fresh module, writing the first sequence number
                        let mut row = [u8::MAX; 8];
                        row[..2].copy_from_slice(&0u16.to_le_bytes());
                        let offset =
                            (PAGE_0 + METADATA_OFFSET) / WRITE_SIZE as u32 * WRITE_SIZE as u32;
                        flash.write(offset, &row).await.unwrap();
                        // verify
                        let seq0_written = Self::read16(&mut flash, offset0)?;
                        if seq0_written != 0 {
                            panic!("failed to write the initial sequence number");
                        }
                        0
                    } else {
                        seq0
                    }
                };
                debug!("last page identified, page 0, seq={}", seq0);
                (PAGE_0, seq)
            } else {
                debug!("last page identified, page 1, seq={}", seq0);
                (PAGE_1, seq1)
            }
        };
        Ok(Self {
            flash,
            page_offset,
            sequence_number,
        })
    }

    pub async fn run(&mut self) {
        let request_receiver = CHANNEL_STORAGE.receiver();
        loop {
            let request = request_receiver.receive().await;
            match request {
                StorageRequest::Load {
                    address,
                    value_type,
                    reply,
                } => {
                    let result = self.load(address, value_type);
                    reply.signal(result);
                }
                StorageRequest::Save {
                    address,
                    value,
                    reply,
                } => {
                    self.save(address, value).await.unwrap();
                    reply.signal(Ok(Value::U8(0)));
                }
            };
        }
    }

    fn load(&mut self, address: u16, value_type: ValueType) -> Result<Value, Error> {
        match value_type {
            ValueType::U8 => self.load_u8(address).map(|v| Value::U8(v)),
            ValueType::U16 => self.load_u16(address).map(|v| Value::U16(v)),
            ValueType::U32 => self.load_u32(address).map(|v| Value::U32(v)),
            ValueType::Text => self.load_text(address).map(|v| Value::Text(v)),
            ValueType::Boolean => self.load_u8(address).map(|v| Value::Boolean(v != 0)),
            ValueType::VectorU8 => self.load_vector_u8(address).map(|v| Value::VectorU8(v)),
            ValueType::VectorU16 => self.load_vector_u16(address).map(|v| Value::VectorU16(v)),
        }
    }

    fn load_u8(&mut self, address: u16) -> Result<u8, Error> {
        Self::read8(&mut self.flash, self.page_offset + address as u32)
    }

    pub fn load_u16(&mut self, address: u16) -> Result<u16, Error> {
        Self::read16(&mut self.flash, self.page_offset + address as u32)
    }

    pub fn load_u32(&mut self, address: u16) -> Result<u32, Error> {
        Self::read32(&mut self.flash, self.page_offset + address as u32)
    }

    pub fn load_text(&mut self, address: u16) -> Result<String<A3_MAX_PROP_DATA_SIZE>, Error> {
        let len = self.load_u8(address)?;
        if len == u8::MAX {
            return Ok(String::new());
        }
        let mut data = [0u8; A3_MAX_PROP_DATA_SIZE + 1];
        self.flash.blocking_read(
            self.page_offset + address as u32 + 1,
            &mut data[..len as usize],
        )?;
        let src_vec = Vec::from_slice(&data[..len as usize]).unwrap();
        Ok(String::from_utf8(src_vec).unwrap())
    }

    pub fn load_vector_u8(
        &mut self,
        address: u16,
    ) -> Result<Vec<u8, MAX_PROP_VECTOR_LENGTH>, Error> {
        let mut vec = Vec::new();
        let len = self.load_u8(address)?;
        if len == u8::MAX {
            return Ok(vec);
        }
        let mut data = [0u8; MAX_PROP_VECTOR_LENGTH];
        self.flash.blocking_read(
            self.page_offset + address as u32 + 1,
            &mut data[..len as usize],
        )?;
        vec.extend_from_slice(&data[..len as usize]).unwrap();

        Ok(vec)
    }

    pub fn load_vector_u16(
        &mut self,
        address: u16,
    ) -> Result<Vec<u16, MAX_PROP_VECTOR_LENGTH>, Error> {
        let mut vec = Vec::new();
        let len = self.load_u8(address)?;
        if len == u8::MAX {
            return Ok(vec);
        }
        let mut data = [0u8; MAX_PROP_VECTOR_LENGTH];
        self.flash.blocking_read(
            self.page_offset + address as u32 + 1,
            &mut data[..(len * 2) as usize],
        )?;
        for i in 0..len as usize {
            let offset = i * 2;
            let element: [u8; 2] = [data[offset], data[offset + 1]];
            vec.push(u16::from_be_bytes(element)).unwrap();
        }

        Ok(vec)
    }

    fn read8(flash: &mut Flash<'static>, offset: u32) -> Result<u8, Error> {
        let mut data = [0u8; 1];
        flash.blocking_read(offset, &mut data)?;
        Ok(u8::from_le_bytes(data))
    }

    fn read16(flash: &mut Flash<'static>, offset: u32) -> Result<u16, Error> {
        let mut data = [0u8; 2];
        flash.blocking_read(offset, &mut data)?;
        Ok(u16::from_le_bytes(data))
    }

    fn read32(flash: &mut Flash<'static>, offset: u32) -> Result<u32, Error> {
        let mut data = [0u8; 4];
        flash.blocking_read(offset, &mut data)?;
        Ok(u32::from_le_bytes(data))
    }

    fn read64(flash: &mut Flash<'static>, offset: u32) -> Result<u64, Error> {
        let mut data = [0u8; 8];
        flash.blocking_read(offset, &mut data)?;
        Ok(u64::from_le_bytes(data))
    }

    async fn save(&mut self, address: u16, value: Value) -> Result<(), Error> {
        match value {
            Value::U8(data) => self.save_u8(address, data).await,
            Value::U16(data) => self.save_u16(address, data).await,
            Value::U32(data) => self.save_u32(address, data).await,
            Value::Text(data) => self.save_text(address, &data).await,
            Value::Boolean(data) => self.save_u8(address, if data { 1 } else { 0 }).await,
            Value::VectorU8(data) => self.save_vector_u8(address, &data).await,
            Value::VectorU16(data) => self.save_vector_u16(address, &data).await,
        }
    }

    async fn save_u8(&mut self, address: u16, data: u8) -> Result<(), Error> {
        let data_array: [u8; 1] = [data];
        self.write(address, &data_array).await
    }

    async fn save_u16(&mut self, address: u16, data: u16) -> Result<(), Error> {
        self.write(address, &data.to_le_bytes()).await
    }

    async fn save_u32(&mut self, address: u16, data: u32) -> Result<(), Error> {
        self.write(address, &data.to_le_bytes()).await
    }

    async fn save_text(
        &mut self,
        address: u16,
        text: &String<A3_MAX_PROP_DATA_SIZE>,
    ) -> Result<(), Error> {
        let mut data = [0xffu8; A3_MAX_PROP_DATA_SIZE + 1];
        data[0] = text.len() as u8;
        data[1..1 + text.len()].copy_from_slice(text.as_bytes());
        self.write(address, &data).await
    }

    async fn save_vector_u8(
        &mut self,
        address: u16,
        vec: &Vec<u8, MAX_PROP_VECTOR_LENGTH>,
    ) -> Result<(), Error> {
        let mut data = [0xffu8; MAX_PROP_VECTOR_LENGTH + 1];
        data[0] = vec.len() as u8;
        data[1..1 + vec.len()].copy_from_slice(vec.as_slice());
        self.write(address, &data).await
    }

    async fn save_vector_u16(
        &mut self,
        address: u16,
        vec: &Vec<u16, MAX_PROP_VECTOR_LENGTH>,
    ) -> Result<(), Error> {
        let mut data = [0xffu8; MAX_PROP_VECTOR_LENGTH * 2 + 1];
        data[0] = (vec.len() * 2) as u8;
        for i in 0..vec.len() {
            let offset = i * 2 + 1;
            data[offset..offset + 1].copy_from_slice(&vec[i].to_be_bytes());
        }
        self.write(address, &data).await
    }

    /// Checks if the rows to write are dirty
    fn is_dirty(&mut self, start_page_offset: u32, end_page_offset: u32) -> Result<bool, Error> {
        let mut offset = start_page_offset;
        while offset < end_page_offset {
            let row = Self::read64(&mut self.flash, offset)?;
            if row != u64::MAX {
                return Ok(true);
            }
            offset += WRITE_SIZE as u32;
        }
        Ok(false)
    }

    async fn write(&mut self, address: u16, data: &[u8]) -> Result<(), Error> {
        debug!(
            "write addr={=u16:#x} page={=u32:#x}",
            address, self.page_offset
        );
        let start_page_offset = self.get_row_offset(address);
        let end_page_offset = self.get_row_offset(address + (data.len() + WRITE_SIZE - 1) as u16);

        if self.is_dirty(start_page_offset, end_page_offset)? {
            trace!("the rows are dirty, starting ping-pong write");
            self.ping_pong_write(address, data).await?;
        } else {
            trace!("the rows are clean, starting straight write");
            let mut position = 0usize;
            let start_column = address as usize % WRITE_SIZE;
            let mut page_offset = start_page_offset;
            if start_column > 0 {
                let mut buf = [0xffu8; WRITE_SIZE];
                let data_width = data.len().min(WRITE_SIZE - start_column);
                buf[start_column..start_column + data_width].copy_from_slice(&data[..data_width]);
                self.flash.write(page_offset, &buf).await?;
                page_offset += WRITE_SIZE as u32;
                position += data_width;
            }
            while position < data.len() {
                let mut buf = [0xffu8; WRITE_SIZE];
                let data_width = (data.len() - position).min(WRITE_SIZE);
                buf[..data_width].copy_from_slice(&data[position..position + data_width]);
                self.flash.write(page_offset, &buf).await?;
                page_offset += WRITE_SIZE as u32;
            }
        }
        Ok(())
    }

    /// Executes ping-pong write.
    /// The method copies the entire page to the other side, also merges the data into
    /// the new page.
    ///
    /// Arguments:
    /// * `address`: The address for the data to be merged into.
    /// * `data`: The data to be merged.
    async fn ping_pong_write(&mut self, address: u16, data: &[u8]) -> Result<(), Error> {
        // defensive code
        if data.len() == 0 {
            return Ok(());
        }

        let prev_page_offset = self.switch_pages().await?;

        // indexes for rows to be merged
        let start_row_index = address as usize / WRITE_SIZE;
        let end_row_index = (address as usize + data.len() - 1) / WRITE_SIZE;
        trace!(
            "row indexes; start={:#x}, end={:#x}",
            start_row_index, end_row_index
        );

        let start_column = address as usize % WRITE_SIZE;
        let end_column = (address as usize + data.len()) % WRITE_SIZE;
        trace!("start column={}, end column={}", start_column, end_column);

        // let target_row_offset = self.get_row_offset(address);
        let mut row_data = [0xffu8; WRITE_SIZE];
        let mut position = 0;
        for irow in 0..NUM_ROWS {
            let src_row_offset = prev_page_offset + (irow * WRITE_SIZE) as u32;
            let dst_row_offset = self.page_offset + (irow * WRITE_SIZE) as u32;

            // Read the data of the previous page from flash
            self.flash.blocking_read(src_row_offset, &mut row_data)?;

            // Merge data if the target rows overlap
            if start_row_index <= irow && irow <= end_row_index {
                if irow == start_row_index && start_column > 0 {
                    let data_width = data.len().min(WRITE_SIZE - start_column);
                    row_data[start_column..start_column + data_width]
                        .copy_from_slice(&data[..data_width]);
                    position += data_width;
                } else if irow == end_row_index && end_column > 0 {
                    let data_width = end_column.min(data.len() - position);
                    row_data[..data_width].copy_from_slice(&data[data.len() - data_width..]);
                } else {
                    trace!("irow={}, index={}", irow, position);
                    row_data.copy_from_slice(&data[position..position + WRITE_SIZE]);
                    position += WRITE_SIZE;
                }
            }

            if u64::from_le_bytes(row_data) != u64::MAX {
                self.flash.write(dst_row_offset, &row_data).await?;
            }
        }
        Ok(())
    }

    async fn simple_write(
        &mut self,
        address: u16,
        data: &[u8],
        original_row: u64,
    ) -> Result<(), Error> {
        let row_offset: u32 = self.get_row_offset(address);
        let buf = Self::merge_into_int(data, original_row, address);
        self.flash.write(row_offset, &buf).await
    }

    async fn switch_pages(&mut self) -> Result<u32, Error> {
        let prev_page_offset = self.page_offset;

        self.page_offset = if self.page_offset == PAGE_0 {
            PAGE_1
        } else {
            PAGE_0
        };

        self.sequence_number = if self.sequence_number == LAST_SEQ_NUMBER {
            0
        } else {
            self.sequence_number + 1
        };

        self.erase_current_page().await?;

        self.simple_write(
            METADATA_OFFSET as u16,
            &self.sequence_number.to_le_bytes(),
            u64::MAX,
        )
        .await?;

        Ok(prev_page_offset)
    }

    async fn erase_current_page(&mut self) -> Result<(), Error> {
        self.flash
            .erase(self.page_offset, self.page_offset + PAGE_SIZE as u32)
            .await
    }

    fn get_row_offset(&self, address: u16) -> u32 {
        (self.page_offset + address as u32) / WRITE_SIZE as u32 * WRITE_SIZE as u32
    }

    fn merge_into_int(src: &[u8], original_row: u64, address: u16) -> [u8; WRITE_SIZE] {
        let mut dst = original_row.to_le_bytes();
        let position = address as usize % WRITE_SIZE;
        dst[position..position + src.len()].copy_from_slice(src);
        dst
    }

    fn is_page_0(seq0: u16, seq1: u16) -> bool {
        if seq1 == NULL_SEQ_NUMBER {
            return true;
        }
        if seq1 == LAST_SEQ_NUMBER && seq0 == 0 {
            return true;
        }
        if seq0 == LAST_SEQ_NUMBER && seq1 == 0 {
            return false;
        }
        return seq0 > seq1;
    }
}
