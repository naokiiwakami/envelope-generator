/*
 * analog3.cpp
 *
 *  Created on: Jul 26, 2025
 *      Author: naoki
 */

#include <algorithm>

#include "analog3/analog3.h"

namespace analog3 {

#define NOWHERE 0xffffffff

Analog3::Analog3(uint32_t uid, uint16_t module_type, const char *name, HwController *can_handler)
    :
    uid_(uid),
    id_(A3_ID_UNASSIGNED),
    module_type_(module_type),
    name_ { name },
    properties_ { },
    hw_controller_(can_handler),
    stream_ { },
    message_handler_(nullptr) {
  if (name_.size() > A3_MAX_CONFIG_DATA_LENGTH) {
    name_.resize(A3_MAX_CONFIG_DATA_LENGTH);
  }
}

Analog3::~Analog3() {
  delete hw_controller_;
}

void Analog3::Transfer(CanTxMessage *message) {
  message->Transfer();
  hw_controller_->DeleteTxMessage(message);
}

void Analog3::SignIn() {
  auto message = hw_controller_->CreateTxMessage();
  message->SetId(uid_);
  message->SetExtended();
  message->GetDataMut()[0] = A3_ADMIN_SIGN_IN;
  message->SetDlc(1);
  Transfer(message);
}

void Analog3::NotifyId() {
  auto message = hw_controller_->CreateTxMessage();
  message->SetId(uid_);
  message->SetExtended();
  auto data = message->GetDataMut();
  data[0] = A3_ADMIN_NOTIFY_ID;
  data[1] = id_ - A3_ID_IM_BASE;
  message->SetDlc(2);
  Transfer(message);
}

void Analog3::HandleRxMessage(const CanRxMessage& message) {
  if (message_handler_ != nullptr) {
    message_handler_->Handle(message);
  }
  if (message.IsExtended()) {
    // TODO: Handle an extension frame
    return;
  }
  uint32_t remote_id = message.GetId();
  if (remote_id == stream_.GetWireAddress()) {
    ReadDataFrame(message);
  }
  if (remote_id == A3_ID_MISSION_CONTROL) {
    HandleMissionControlMessage(message);
  }
}

void Analog3::ReadDataFrame(const CanRxMessage& message) {
  auto *data = message.GetData();
  auto length = message.GetDlc();
  if (stream_.ImportDataFrame(data, length)) {
    // done
    for (auto raw_prop : stream_.GetRawProperties()) {
      for (auto prop : properties_) {
        if (prop.id == raw_prop.type) {
          prop.Incorporate(raw_prop.value.data(), raw_prop.length);
        }
      }
    }
    stream_.ClearRawProperties();
  } else {
    // continue
    auto message = hw_controller_->CreateTxMessage();
    message->SetId(stream_.GetWireAddress());
    message->SetRemote(true);
    message->SetDlc(0);
    Transfer(message);
  }
}

void Analog3::HandleMissionControlMessage(const CanRxMessage& message) {
  auto *data = message.GetData();
  uint8_t opcode = data[0];

  switch (opcode) {
    case A3_MC_SIGN_IN:
      if (id_ == A3_ID_UNASSIGNED) {
        SignIn();
      } else {
        NotifyId();
      }
      break;
    case A3_MC_ASSIGN_MODULE_ID: {
      uint32_t target_module = data[1] << 24 | data[2] << 16 | data[3] << 8 | data[4];
      if (target_module == uid_) {
        id_ = data[5] + A3_ID_IM_BASE;

        auto message = hw_controller_->CreateTxMessage();
        message->SetId(id_);
        message->GetDataMut()[0] = A3_IM_ID_ASSIGN_ACK;
        message->SetDlc(1);
        Transfer(message);

        hw_controller_->SetBlueIndicator();
        hw_controller_->ResetRedIndicator();
      }
      break;
    }
    default: {
      uint16_t target_id = data[1] + A3_ID_IM_BASE;
      if (target_id == id_) {
        ProcessMissionControlCommand(opcode, data, message.GetDlc());
      }
    }
  }
}

void Analog3::ProcessMissionControlCommand(uint8_t opcode, const uint8_t *data, uint8_t dlc) {
  switch (opcode) {
    case A3_MC_PING: {
      auto message = hw_controller_->CreateTxMessage();
      message->SetId(id_);
      message->GetDataMut()[0] = A3_IM_PING_REPLY;
      message->SetDlc(1);
      Transfer(message);
      if (dlc >= 3 && data[2]) {
        hw_controller_->BlinkBlueIndicator(3, 70);
      }
      break;
    }
    case A3_MC_REQUEST_NAME:
      HandleRequestName(data, dlc);
      break;
    case A3_MC_CONTINUE_NAME:
      HandleContinueName(data, dlc);
      break;
    case A3_MC_REQUEST_CONFIG:
      HandleRequestConfig(data, dlc);
      break;
    case A3_MC_CONTINUE_CONFIG:
      HandleContinueConfig(data, dlc);
      break;
    case A3_MC_MODIFY_CONFIG:
      HandleModifyConfig(data, dlc);
      break;
  }
}

void Analog3::HandleRequestName(const uint8_t *data, uint8_t dlc) {
  uint32_t wire_addr = data[2] + A3_ID_ADMIN_WIRES_BASE;
  stream_.InitiateAdminWrites(wire_addr, A3_PROP_MODULE_NAME, 1);
  auto message = hw_controller_->CreateTxMessage();
  message->SetId(wire_addr);

  int32_t payload_size = stream_.FillPropertyData(properties_, message->GetDataMut(), 0);
  message->SetDlc(payload_size);
  Transfer(message);
}

void Analog3::HandleContinueName(const uint8_t *data, uint8_t dlc) {
  uint32_t wire_addr = stream_.GetWireAddress();
  if (wire_addr == A3_ID_INVALID) {
    // no active stream, ignore.
    return;
  }
  auto message = hw_controller_->CreateTxMessage();
  message->SetId(wire_addr);
  int32_t payload_size = stream_.FillPropertyData(properties_, message->GetDataMut(), 0);
  message->SetDlc(payload_size);
  Transfer(message);
}

void Analog3::HandleRequestConfig(const uint8_t *data, uint8_t dlc) {
  uint32_t wire_addr = data[2] + A3_ID_ADMIN_WIRES_BASE;
  stream_.InitiateAdminWrites(wire_addr, 0, properties_.size());
  auto message = hw_controller_->CreateTxMessage();
  message->SetId(wire_addr);
  int32_t payload_index = 0;
  message->GetDataMut()[payload_index++] = static_cast<uint8_t>(properties_.size() & 0xff);
  while (payload_index < CAN_STD_DATA_LENGTH && !stream_.IsDone()) {
    payload_index = stream_.FillPropertyData(properties_, message->GetDataMut(), payload_index);
  }
  message->SetDlc(payload_index);
  Transfer(message);
}

void Analog3::HandleContinueConfig(const uint8_t *data, uint8_t dlc) {
  uint32_t wire_addr = stream_.GetWireAddress();
  if (wire_addr == A3_ID_INVALID) {
    // no active stream, ignore.
    return;
  }
  auto message = hw_controller_->CreateTxMessage();
  message->SetId(wire_addr);
  int payload_index = 0;
  while (payload_index < CAN_STD_DATA_LENGTH && !stream_.IsDone()) {
    payload_index = stream_.FillPropertyData(properties_, message->GetDataMut(), payload_index);
  }
  message->SetDlc(payload_index);
  Transfer(message);
}

void Analog3::HandleModifyConfig(const uint8_t *data, uint8_t dlc) {
  uint32_t wire_addr = data[2] + A3_ID_ADMIN_WIRES_BASE;
  stream_.InitiateAdminReads(wire_addr);
  auto message = hw_controller_->CreateTxMessage();
  message->SetId(wire_addr);
  message->SetRemote(true);
  message->SetDlc(0);
  Transfer(message);
}

}  // namespace analog3
