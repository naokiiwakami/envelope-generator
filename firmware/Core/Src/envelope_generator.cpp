/*
 * envelope_generator.cpp
 *
 *  Created on: Nov 3, 2025
 *      Author: naoki
 */

#include <math.h>

#include "analog3/analog3.h"
#include "analog3/definitions.h"
#include "analog3/stm32impl.h"
#include "envelope_generator.h"

extern analog3::Analog3 *a3;

namespace analog3 {

class EnvelopeGenerator {
 private:
  uint16_t dac_index_;

  uint32_t current_value_ = 0;
  uint32_t target_value_ = 0;
  uint32_t switch_value_ = 0;

  uint16_t attack_time_ = 0;
  uint16_t decay_time_ = 0;
  uint16_t sustain_level_ = 0xffff;
  uint16_t release_time_ = 0;

  uint32_t attack_ratio_ = 0;
  uint32_t decay_ratio_ = 0;
  uint32_t release_ratio_ = 0;

  enum class Phase {
    ATTACKING,
    SUSTAINING,
    RELEASED,
  };

  Phase phase_ = Phase::RELEASED;
  void (*UpdateValue)(EnvelopeGenerator *instance) = UpdateRelease;

 public:
  EnvelopeGenerator(uint16_t dac_index) : dac_index_(dac_index) {}
  EnvelopeGenerator() = delete;
  ~EnvelopeGenerator() = default;

  void Initialize() {
    InitializeMcp47x6Dac(dac_index_, MCP47X6_VRL_VDD, MCP47X6_PD_NORMAL, MCP47X6_GAIN_1X);
    GateOff();
  }

  void GateOn(uint16_t velocity) {
    uint32_t level = (velocity * velocity) >> 16;
    target_value_ = level * 1.2;
    switch_value_ = level;
    phase_ = Phase::ATTACKING;
    UpdateValue = UpdateAttack;
  }

  void GateOff() {
    target_value_ = 0;
    phase_ = Phase::RELEASED;
    UpdateValue = UpdateRelease;
  }

  void Update() {
    UpdateValue(this);
    UpdateMcp47x6Dac(dac_index_, current_value_);
  }

  inline uint16_t GetReleaseTime() const { return release_time_; }

  void SetReleaseTime(uint16_t release_time) {
    release_time_ = release_time;
    float ratio = expf(-1.e-3 / expf(release_time * 0.000115 - 4.6));
    release_ratio_ = (uint32_t)(ratio * 65536);
    // release_ratio_ = 59989;
  }

 private:
  static void UpdateAttack(EnvelopeGenerator *self) {
    uint32_t diff = self->target_value_ - self->current_value_;
    diff *= self->attack_ratio_;
    diff >>= 16;
    self->current_value_ = self->target_value_ - diff;
    if (self->current_value_ >= self->switch_value_) {
      self->current_value_ = self->switch_value_;
      self->target_value_ = (self->switch_value_ * self->sustain_level_) >> 16;
      self->phase_ = Phase::SUSTAINING;
      self->UpdateValue = UpdateDecay;
    }
  }

  static void UpdateDecay(EnvelopeGenerator *self) {
    uint32_t diff = self->current_value_ - self->target_value_;
    diff *= self->decay_ratio_;
    diff >>= 16;
    self->current_value_ = self->target_value_ + diff;
  }

  static void UpdateRelease(EnvelopeGenerator *self) {
    uint32_t diff = self->current_value_ - self->target_value_;
    diff *= self->release_ratio_;
    diff >>= 16;
    self->current_value_ = diff;
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
  analog3::eg_voice_0.SetReleaseTime(16384);
  a3->InjectMessageHandler(&message_handler);
}

void UpdateEnvelopeGenerator() {
  analog3::eg_voice_0.Update();
}

void SetReleaseTime(uint16_t release_time) {
  uint16_t rounded_release_time = (release_time >> 6) << 6;
  auto current_release_time = analog3::eg_voice_0.GetReleaseTime();
  if (rounded_release_time == current_release_time) {
    return;
  }
  analog3::eg_voice_0.SetReleaseTime(rounded_release_time);
}
