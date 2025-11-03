/*
 * envelope_generator.cpp
 *
 *  Created on: Nov 3, 2025
 *      Author: naoki
 */

#include "analog3/analog3.h"
#include "analog3/definitions.h"
#include "analog3/stm32impl.h"
#include "envelope_generator.h"

extern analog3::Analog3 *a3;

namespace analog3 {

class EnvelopeGenerator {
 private:
  uint16_t value_ = 0;
  uint16_t dac_index_;

 public:
  EnvelopeGenerator(uint16_t dac_index) : dac_index_(dac_index) {}
  EnvelopeGenerator() = delete;
  ~EnvelopeGenerator() = default;

  void Initialize() {
    InitializeMcp47x6Dac(dac_index_, MCP47X6_VRL_VDD, MCP47X6_PD_NORMAL, MCP47X6_GAIN_1X);
    GateOff();
  }

  void GateOn(uint16_t velocity) {
    value_ = velocity;
    UpdateMcp47x6Dac(dac_index_, value_);
  }

  void GateOff() {
    value_ = 0;
    UpdateMcp47x6Dac(dac_index_, value_);
  }
};

static EnvelopeGenerator eg_voice_0{1};

class EgMessageHandler : public MessageHandler {
 public:
  EgMessageHandler() = default;
  ~EgMessageHandler() = default;

  void Handle(const CanRxMessage& message) {
    auto id = message.GetId();
    uint32_t voice_id = 0;
    if (id == A3_ID_MIDI_VOICE_BASE + voice_id) {
      uint8_t index = 0;
      do {
        index = ReadMessage(message.GetData(), voice_id, index);
      } while (index < message.GetDlc());
    }
  }

 private:
  uint8_t ReadMessage(const uint8_t *data, uint32_t voice_id, uint8_t index) {
    auto op = data[index++];
    switch (op) {
    case A3_VOICE_MSG_GATE_ON:
      if (index < 6) {
        uint16_t velocity = (data[index] << 8) + data[index + 1];
        index += 2;
        eg_voice_0.GateOn(velocity);
        put_string("gate on ");
        put_hex(&velocity, sizeof(velocity));
        put_string("\r\n");
      }
      break;
    case A3_VOICE_MSG_GATE_OFF:
      eg_voice_0.GateOff();
      put_string("gate off\r\n");
      break;
    }
    return index;
  }
};

}  // namespace analog3

static analog3::EgMessageHandler message_handler;

void InitializeEnvelopeGenerator() {
  analog3::eg_voice_0.Initialize();
  a3->InjectMessageHandler(&message_handler);
}

