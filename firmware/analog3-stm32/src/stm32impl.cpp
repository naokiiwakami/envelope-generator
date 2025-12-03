/*
 * analog3_internal.cpp
 *
 *  Created on: Jul 26, 2025
 *      Author: naoki
 */

#include <stdlib.h>
#include <string.h>
#ifdef DEBUG
#include <stdio.h>
#include <stdarg.h>
#endif

#include <algorithm>
#include <string>

#include "main.h"

#include <analog3/addresses-common.h>
#include "analog3/analog3.h"
#include "analog3/config-common.h"
#include "analog3/property.h"
#include "analog3/stm32impl.h"
#include "analog3/storage.h"
#include "analog3/tasks.h"

#ifdef DEBUG
char error_message[256];
#endif

stm32_can_message_t message_queue[MESSAGE_QUEUE_SIZE];
uint32_t q_head = 0;
uint32_t q_tail = 0;
uint8_t q_full = 0;

analog3::Analog3 *a3 = nullptr;

static void IncorporateString(analog3::Property *prop, const void *data, uint8_t len) {
  size_t data_len = std::min(static_cast<size_t>(len),
                             static_cast<size_t>(A3_MAX_CONFIG_DATA_LENGTH) - 1);
  auto dest = static_cast<std::string*>(prop->data);
  dest->assign(static_cast<const char*>(data), data_len);
  if (prop->save_addr == ADDR_UNSET) {
    return;
  }
  SaveString(prop->save_addr, dest->c_str(), A3_MAX_CONFIG_DATA_LENGTH);
}

static void IncorporateU8(analog3::Property *prop, const void *data, uint8_t len) {
  memcpy(prop->data, data, len);
  if (prop->save_addr == ADDR_UNSET) {
    return;
  }
  Save8(prop->save_addr, *static_cast<const uint8_t*>(data));
}

static void IncorporateU16(analog3::Property *prop, const void *data, uint8_t len) {
  if (len < sizeof(uint16_t)) {
    // The source is short of bytes, ignore the request silently
    return;
  }
  auto src = static_cast<const uint16_t*>(data);
  auto dest = static_cast<uint16_t*>(prop->data);
  *dest = __builtin_bswap16(*src);
  if (prop->save_addr == ADDR_UNSET) {
    return;
  }
  Save16(prop->save_addr, *dest);
}

static void IncorporateU32(analog3::Property *prop, const void *data, uint8_t len) {
  if (len < sizeof(uint32_t)) {
    // The source is short of bytes, ignore the request silently
    return;
  }
  auto src = static_cast<const uint32_t*>(data);
  auto dest = static_cast<uint32_t*>(prop->data);
  *dest = __builtin_bswap32(*src);
  if (prop->save_addr == ADDR_UNSET) {
    return;
  }
  Save32(prop->save_addr, *dest);
}

static void IncorporateVectorU8P(analog3::Property *prop, const void *data, uint8_t len) {
  auto src = static_cast<const uint8_t*>(data);
  auto dest = static_cast<analog3::A3VectorP<uint8_t>*>(prop->data);
  auto copy_length = std::min(dest->size, len);
  for (uint8_t i = 0; i < copy_length; ++i) {
    *dest->data[i] = src[i];
  }
  if (prop->save_addr == ADDR_UNSET) {
    return;
  }
  // Save32(prop->save_addr, *static_cast<const uint32_t*>(data));
}

static void IncorporateVectorU16P(analog3::Property *prop, const void *data, uint8_t len) {
  auto src = static_cast<const uint16_t*>(data);
  auto dest = static_cast<analog3::A3VectorP<uint16_t>*>(prop->data);
  auto copy_length = std::min(dest->size, (uint8_t)(len / 2));
  for (uint8_t i = 0; i < copy_length; ++i) {
    *dest->data[i] = __builtin_bswap16(src[i]);
  }
  if (prop->save_addr == ADDR_UNSET) {
    return;
  }
  // Save32(prop->save_addr, *static_cast<const uint32_t*>(data));
}

class Stm32CanTxMessage : public analog3::CanTxMessage {
 private:
  FDCAN_TxHeaderTypeDef tx_header_;
  uint8_t tx_data_[CAN_STD_DATA_LENGTH];

 public:
  Stm32CanTxMessage()
      :
      tx_data_ { 0 } {
    tx_header_.Identifier = 0x0;
    tx_header_.IdType = FDCAN_STANDARD_ID;
    tx_header_.TxFrameType = FDCAN_DATA_FRAME;
    tx_header_.DataLength = FDCAN_DLC_BYTES_0;
    tx_header_.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header_.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header_.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header_.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header_.MessageMarker = 0;
  }

  ~Stm32CanTxMessage() = default;

  void SetExtended() {
    tx_header_.IdType = FDCAN_EXTENDED_ID;
  }

  void SetStandard() {
    tx_header_.IdType = FDCAN_STANDARD_ID;
  }

  void SetId(uint32_t id) {
    tx_header_.Identifier = id;
  }

  void SetDlc(uint8_t dlc) {
    tx_header_.DataLength = (uint32_t) dlc;
  }

  uint8_t GetDlc() const {
    return (uint8_t) tx_header_.DataLength;
  }

  void SetRemote(bool is_remote) {
    tx_header_.TxFrameType = is_remote ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
  }

  bool IsRemote() const {
    return tx_header_.TxFrameType == FDCAN_REMOTE_FRAME;
  }

  uint8_t* GetDataMut() {
    return tx_data_;
  }

  void Transfer() {
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header_, tx_data_) != HAL_OK) {
      Error_Handler();
    }
  }
};

class Stm32CanRxMessage : public analog3::CanRxMessage {
 private:
  // borrowed from message queue item
  const FDCAN_RxHeaderTypeDef& rx_header_;
  const uint8_t *rx_data_;

 public:
  Stm32CanRxMessage(const stm32_can_message_t& message)
      :
      rx_header_(message.header),
      rx_data_(message.data) {
  }

  ~Stm32CanRxMessage() = default;

  bool IsExtended() const {
    return rx_header_.IdType == FDCAN_EXTENDED_ID;
  }

  bool IsStandard() const {
    return rx_header_.IdType == FDCAN_STANDARD_ID;
  }

  uint32_t GetId() const {
    return rx_header_.Identifier;
  }

  uint8_t GetDlc() const {
    return (uint8_t) rx_header_.DataLength;
  }

  bool IsRemote() const {
    return rx_header_.RxFrameType == FDCAN_REMOTE_FRAME;
  }

  const uint8_t* GetData() const {
    return rx_data_;
  }
};

struct BlinkState {
  GPIO_TypeDef *target_port;
  uint16_t target_pin;
  uint32_t remaining;
  uint32_t next_tick;
  uint32_t interval;

  BlinkState(GPIO_TypeDef *target_port, uint16_t target_pin, uint32_t times, uint32_t interval)
      :
      target_port(target_port),
      target_pin(target_pin),
      remaining(times * 2),
      interval(interval) {
    next_tick = HAL_GetTick();
  }
};

class Stm32Controller : public analog3::HwController {
 public:
  ~Stm32Controller() = default;

  analog3::CanTxMessage* CreateTxMessage() {
    return new Stm32CanTxMessage();
  }

  void DeleteTxMessage(analog3::CanTxMessage *message) {
    delete message;
  }

  void SetRedIndicator() const {
    HAL_GPIO_WritePin(A3_IND_RED_GPIO_Port, A3_IND_RED_Pin, GPIO_PIN_SET);
  }

  void ResetRedIndicator() const {
    HAL_GPIO_WritePin(A3_IND_RED_GPIO_Port, A3_IND_RED_Pin, GPIO_PIN_RESET);
  }

  void BlinkRedIndicator(uint32_t times, uint32_t interval) const {
    auto state = new BlinkState(A3_IND_RED_GPIO_Port, A3_IND_RED_Pin, times, interval);
    if (SubmitTask(BlinkIndicator, state) != 0) {
      delete state;
    }
  }

  void SetBlueIndicator() const {
    HAL_GPIO_WritePin(A3_IND_BLUE_GPIO_Port, A3_IND_BLUE_Pin, GPIO_PIN_SET);
  }

  void ResetBlueIndicator() const {
    HAL_GPIO_WritePin(A3_IND_BLUE_GPIO_Port, A3_IND_BLUE_Pin, GPIO_PIN_RESET);
  }

  void BlinkBlueIndicator(uint32_t times, uint32_t interval) const {
    auto state = new BlinkState(A3_IND_BLUE_GPIO_Port, A3_IND_BLUE_Pin, times, interval);
    if (SubmitTask(BlinkIndicator, state) != 0) {
      delete state;
    }
  }

#ifdef DEBUG
  void DebugPrint(const char *fmt, ...) const {
    va_list args;
    va_start(args, fmt);
    char buf[128];
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    put_string(buf);
  }
#endif

 private:
  static void BlinkIndicator(void *arg) {
    auto state = static_cast<BlinkState*>(arg);
    uint32_t now = HAL_GetTick();
    if (now >= state->next_tick) {
      HAL_GPIO_TogglePin(state->target_port, state->target_pin);
      if (--state->remaining == 0) {
        delete state;
        return;
      }
      state->next_tick = now + state->interval;
    }
    SubmitTask(BlinkIndicator, state);
  }

  static void ReleaseBlinkState(void *arg) {
    delete static_cast<BlinkState*>(arg);
  }
};

/**
 * Read stored UID or create one.
 */
uint32_t SetUpModuleUid() {
  uint32_t uid = Load32(ADDR_MODULE_UID);
  DEBUG_PRINT("stored UID: %08lx", uid);
  if (uid == 0xffffffff) {
    HAL_Delay(10);
    adc_change_channel(ADC_CHANNEL_TEMPSENSOR);
    uint32_t temperature = adc_run();
    adc_change_channel(ADC_CHANNEL_0);
    srandom(temperature);
    uid = random() & 0x1fffffff;
    Save32(ADDR_MODULE_UID, uid);
  }
  return uid;
}

void InitializeAnalog3() {
  uint32_t uid = SetUpModuleUid();
  char module_name[A3_MAX_CONFIG_DATA_LENGTH];
  LoadString(ADDR_MODULE_NAME, module_name, sizeof(module_name));
  if (module_name[0] == 0) {
    strcpy(module_name, GetDefaultModuleName());
    SaveString(ADDR_MODULE_NAME, module_name, A3_MAX_CONFIG_DATA_LENGTH);
  }
  uint16_t module_type = GetModuleType();
  a3 = new analog3::Analog3(uid, module_type, module_name, new Stm32Controller());
  auto properties = a3->GetPropertiesMut();

  // set up common properties
  properties->push_back( { A3_PROP_MODULE_UID, A3_TYPE_MODULE_UID, &a3->GetUid() });

  properties->push_back( { A3_PROP_MODULE_TYPE, A3_TYPE_MODULE_TYPE, &a3->GetModuleType() });

  properties->push_back( { A3_PROP_MODULE_NAME, A3_TYPE_MODULE_NAME, a3->GetNameMut(),
      IncorporateString, ADDR_MODULE_NAME });

  // set up module specific properties
  SetModuleSpecificProperties(properties);
}

void AddReadOnlyProperty(std::vector<analog3::Property> *properties, uint8_t id, uint8_t value_type,
                         const void *data) {
  properties->push_back( { id, value_type, data });
}

void AddProperty(std::vector<analog3::Property> *properties, uint8_t id, uint8_t value_type,
                 void *data, uint16_t save_addr) {
  void (*incorporate)(analog3::Property*, const void*, uint8_t);
  switch (value_type) {
  case A3_U8:
    incorporate = IncorporateU8;
    break;
  case A3_U16:
    incorporate = IncorporateU16;
    break;
  case A3_U32:
    incorporate = IncorporateU32;
    break;
  case A3_STRING:
    incorporate = IncorporateString;
    break;
  case A3_VECTOR_U8P:
    incorporate = IncorporateVectorU8P;
    break;
  case A3_VECTOR_U16P:
    incorporate = IncorporateVectorU16P;
    break;
  default:
    incorporate = nullptr;
  }
  properties->push_back( { id, value_type, data, incorporate, save_addr });
}

void SignIn() {
  a3->GetHwController().SetRedIndicator();
  a3->SignIn();
}

void HandleCanRxMessage(const stm32_can_message_t *message) {
  Stm32CanRxMessage new_message{*message};
  a3->HandleRxMessage(new_message);
}
