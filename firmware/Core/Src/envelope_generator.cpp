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

struct EnvelopeGeneratorParams {
  uint16_t attack_time_param = 0;
  uint16_t decay_time_param = 0;
  uint16_t sustain_level_param = 0;
  uint16_t release_time_param = 0;

  uint32_t attack_ratio = 0;
  uint32_t decay_ratio = 0;
  uint32_t sustain_level = 0xffff;
  uint32_t release_ratio = 0;

  const float kDeltaT = 1.25e-4;

  void SetAttackTime(uint16_t new_attack_time) {
    uint16_t rounded_attack_time = (new_attack_time >> 6) << 6;
    if (rounded_attack_time == attack_time_param) {
      return;
    }

    attack_time_param = rounded_attack_time;
    float ratio = expf(-kDeltaT / expf(rounded_attack_time * 0.00015 - 7.5));
    attack_ratio = (uint32_t)(ratio * 65536);
  }

  void SetDecayTime(uint16_t new_decay_time) {
    uint16_t rounded_decay_time = (new_decay_time >> 6) << 6;
    if (rounded_decay_time == decay_time_param) {
      return;
    }

    decay_time_param = rounded_decay_time;
    float ratio = expf(-kDeltaT / expf(rounded_decay_time * 0.00015 - 7.5));
    decay_ratio = (uint32_t)(ratio * 65536);
  }

  void SetSustainLevel(uint16_t new_sustain_level) {
    uint16_t rounded_sustain_level = (new_sustain_level >> 6) << 6;
    if (rounded_sustain_level == sustain_level_param) {
      return;
    }
    sustain_level_param = rounded_sustain_level;
    sustain_level = ((uint32_t)rounded_sustain_level * (uint32_t)rounded_sustain_level) >> 16;
  }

  void SetReleaseTime(uint16_t new_release_time) {
    uint16_t rounded_release_time = (new_release_time >> 6) << 6;
    if (rounded_release_time == release_time_param) {
      return;
    }

    release_time_param = rounded_release_time;
    float ratio = expf(-kDeltaT / expf(rounded_release_time * 0.000115 - 4.6));
    release_ratio = (uint32_t)(ratio * 65536);
  }
};

class EnvelopeGenerator {
 private:
  uint16_t dac_index_;

  uint32_t current_value_ = 0;
  uint32_t target_value_ = 0;
  uint32_t peak_value_ = 0;

  const EnvelopeGeneratorParams &params_;

  enum class Phase {
    ATTACKING,
    SUSTAINING,
    RELEASED,
  };

  Phase phase_ = Phase::RELEASED;
  void (*UpdateValue)(EnvelopeGenerator *instance) = UpdateRelease;

 public:
  EnvelopeGenerator(uint16_t dac_index, const EnvelopeGeneratorParams &params)
      :
      dac_index_(dac_index),
      params_ { params } {
  }
  EnvelopeGenerator() = delete;
  ~EnvelopeGenerator() = default;

  void Initialize() {
    InitializeMcp47x6Dac(dac_index_, MCP47X6_VRL_VDD, MCP47X6_PD_NORMAL, MCP47X6_GAIN_1X);
    GateOff(0);
  }

  void GateOn(uint16_t velocity, uint32_t voice_id) {
    if (voice_id != (uint32_t)dac_index_ - 1) {
      return;
    }
    uint32_t level = ((uint32_t)velocity * (uint32_t)velocity) >> 16;
    target_value_ = level * 1.2;
    peak_value_ = level;
    phase_ = Phase::ATTACKING;
    UpdateValue = UpdateAttack;
  }

  void GateOff(uint32_t voice_id) {
    if (voice_id != (uint32_t)dac_index_ - 1) {
      return;
    }
    target_value_ = 0;
    phase_ = Phase::RELEASED;
    UpdateValue = UpdateRelease;
  }

  void Update() {
    UpdateValue(this);
    UpdateMcp47x6Dac(dac_index_, current_value_);
  }

 private:
  static void UpdateAttack(EnvelopeGenerator *self) {
    uint32_t diff = self->target_value_ - self->current_value_;
    diff *= self->params_.attack_ratio;
    diff >>= 16;
    self->current_value_ = self->target_value_ - diff;
    if (self->current_value_ >= self->peak_value_) {
      self->current_value_ = self->peak_value_;
      self->phase_ = Phase::SUSTAINING;
      self->UpdateValue = UpdateDecay;
    }
  }

  static void UpdateDecay(EnvelopeGenerator *self) {
    self->target_value_ = (self->peak_value_ * self->params_.sustain_level) >> 16;
    if (self->current_value_ > self->target_value_ ) {
      uint32_t diff = self->current_value_ - self->target_value_;
      diff *= self->params_.decay_ratio;
      diff >>= 16;
      self->current_value_ = self->target_value_ + diff;
    } else {
      uint32_t diff = self->target_value_ - self->current_value_;
      diff *= self->params_.decay_ratio;
      diff >>= 16;
      self->current_value_ = self->target_value_ - diff;
    }
  }

  static void UpdateRelease(EnvelopeGenerator *self) {
    uint32_t diff = self->current_value_ - self->target_value_;
    diff *= self->params_.release_ratio;
    diff >>= 16;
    self->current_value_ = diff;
  }
};

static EnvelopeGeneratorParams eg_params{};
static EnvelopeGenerator eg_voice_0{1, eg_params};

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
        eg_voice_0.GateOn(velocity, voice_id);
        put_string("gate on ");
        put_hex(&velocity, sizeof(velocity));
        put_string("\r\n");
      }
      break;
    case A3_VOICE_MSG_GATE_OFF:
      eg_voice_0.GateOff(voice_id);
      put_string("gate off\r\n");
      break;
    }
    return index;
  }
};

}  // namespace analog3

static analog3::EgMessageHandler message_handler;

// C API for the main program

void InitializeEnvelopeGenerator() {
  analog3::eg_voice_0.Initialize();
  a3->InjectMessageHandler(&message_handler);
}

void UpdateEnvelopeGenerator() {
  analog3::eg_voice_0.Update();
}

void SetAttackTime(uint16_t attack_time) {
  analog3::eg_params.SetAttackTime(attack_time);
}

void SetDecayTime(uint16_t decay_time) {
  analog3::eg_params.SetDecayTime(decay_time);
}

void SetSustainLevel(uint16_t sustain_level) {
  analog3::eg_params.SetSustainLevel(sustain_level);
}

void SetReleaseTime(uint16_t release_time) {
  analog3::eg_params.SetReleaseTime(release_time);
}
