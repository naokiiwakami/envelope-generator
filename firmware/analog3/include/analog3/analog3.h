/*
 * analog3.h
 *
 *  Created on: Jul 26, 2025
 *      Author: naoki
 */

#ifndef INCLUDE_ANALOG3_ANALOG3_H_
#define INCLUDE_ANALOG3_ANALOG3_H_

#include <stdint.h>

#include <string>
#include <vector>

#include "analog3/definitions.h"
#include "analog3/property.h"
#include "analog3/stream.h"

namespace analog3 {

/**
 * Abstract class to handle CAN TX message.
 */
class CanTxMessage {
 public:
  virtual ~CanTxMessage() = default;

  virtual void SetExtended() = 0;
  virtual void SetStandard() = 0;
  virtual void SetId(uint32_t id) = 0;
  virtual void SetDlc(uint8_t dlc) = 0;
  virtual uint8_t GetDlc() const = 0;
  virtual uint8_t* GetDataMut() = 0;
  virtual void Transfer() = 0;
};

/**
 * Abstract class to handle CAN RX message.
 */
class CanRxMessage {
 public:
  virtual ~CanRxMessage() = default;

  virtual bool IsExtended() const = 0;
  virtual bool IsStandard() const = 0;
  virtual uint32_t GetId() const = 0;
  virtual uint8_t GetDlc() const = 0;
  virtual const uint8_t* GetData() const = 0;
};

/**
 * Abstract class to create a CAN message.
 */
class HwController {
 public:
  virtual ~HwController() = default;

  virtual CanTxMessage* CreateTxMessage() = 0;
  virtual void DeleteTxMessage(CanTxMessage *message) = 0;
  virtual void SetRedIndicator() const = 0;
  virtual void ResetRedIndicator() const = 0;
  virtual void BlinkRedIndicator(uint32_t times, uint32_t interval) const = 0;
  virtual void SetBlueIndicator() const = 0;
  virtual void ResetBlueIndicator() const = 0;
  virtual void BlinkBlueIndicator(uint32_t times, uint32_t interval) const = 0;

#ifdef DEBUG
  virtual void DebugPrint(const char *fmt, ...) const = 0;
#else
  inline void DebugPrint(const char *fmt, ...) {
    // do nothing hoping optimizer removes this method call
  }
#endif
};

/**
 * Analog3 module.
 */
class Analog3 {
 private:
  uint32_t uid_;
  uint16_t id_;
  uint16_t module_type_;
  std::string name_;

  std::vector<Property> properties_;

  HwController *hw_controller_;

  Stream stream_;

 public:
  Analog3(uint32_t uid, uint16_t module_type, const char *name, HwController *can_handler);
  ~Analog3();

  inline const std::vector<Property>& GetProperties() const {
    return properties_;
  }

  inline std::vector<Property>* GetPropertiesMut() {
    return &properties_;
  }

  inline const uint32_t& GetUid() const {
    return uid_;
  }

  inline const uint16_t& GetModuleType() const {
    return module_type_;
  }

  inline const uint16_t& GetId() const {
    return id_;
  }

  inline const std::string& GetName() const {
    return name_;
  }

  inline std::string* GetNameMut() {
    return &name_;
  }

  const HwController& GetHwController() {
    return *hw_controller_;
  }

  void SignIn();
  void NotifyId();

  void HandleRxMessage(const CanRxMessage& message);
  void HandleMissionControlMessage(const CanRxMessage& message);

 private:
  void ProcessMissionControlCommand(uint8_t opcode, const uint8_t *data, uint8_t dlc);
  void Transfer(CanTxMessage *message);

  void HandleRequestName(const uint8_t *data, uint8_t dlc);
  void HandleContinueName(const uint8_t *data, uint8_t dlc);
  void HandleRequestConfig(const uint8_t *data, uint8_t dlc);
  void HandleContinueConfig(const uint8_t *data, uint8_t dlc);
};

}  // namespace analog3

#endif /* INCLUDE_ANALOG3_ANALOG3_H_ */
