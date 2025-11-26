/*
 * envelope_generator.cpp
 *
 *  Created on: Nov 3, 2025
 *      Author: naoki
 */

#include <math.h>

#include "main.h"

#include "analog3/analog3.h"
#include "analog3/definitions.h"
#include "analog3/stm32impl.h"
#include "envelope_generator.h"

#define DECAY_DISTORTION 1

extern analog3::Analog3 *a3;

namespace analog3 {

struct EnvelopeGeneratorParams {
  uint16_t attack_time_param = 0;
  uint16_t decay0_time_param = 0;
  uint16_t sustain0_level_param = 0;
  uint16_t decay_time_param = 0;
  uint16_t sustain_level_param = 0;
  uint16_t release_time_param = 0;

  double attack_time_constant = 0;
  uint64_t attack_ratio = 0;
  uint64_t decay0_ratio = 0;
  double decay_time_constant = 0;
  uint64_t decay_ratio = 0;
  uint64_t sustain0_level = 0xffffffff;
  uint64_t sustain_level = 0xffffffff;
  double release_time_constant = 0;
  uint64_t release_ratio = 0;

  double distortion_steepness = 0;
  double distortion_depth = 0;

  double delta_t = 0;

  void Initialize() {
    delta_t = (double)htim3.Init.Period * (double)EG_UPDATE_CYCLES / (double)HAL_RCC_GetSysClockFreq();
  }

  void SetAttackTime(uint16_t new_attack_time) {
    uint16_t rounded_attack_time = (new_attack_time >> 6) << 6;
    if (rounded_attack_time == attack_time_param) {
      return;
    }

    attack_time_param = rounded_attack_time;
    attack_time_constant = 1.0 + 3.5e-10 * rounded_attack_time * rounded_attack_time * rounded_attack_time;
    attack_ratio = 0xffffffff / attack_time_constant;

    // double ratio = exp(-delta_t / exp(rounded_attack_time * 0.00015 - 7.5));
    // attack_ratio = (uint64_t)(ratio * 4294967296.0);
  }

  void SetDecay0Time(uint16_t new_decay_time) {
    uint16_t rounded_decay_time = (new_decay_time >> 6) << 6;
    if (rounded_decay_time == decay0_time_param) {
      return;
    }

    decay0_time_param = rounded_decay_time;
#ifdef DECAY_DISTORTION
    distortion_steepness = (double)rounded_decay_time / 13107.2 + 3.0; // 3 to 8
#else
    double time_constant = exp(rounded_decay_time * 0.00015 - 7.5);
    double ratio = exp(-delta_t / time_constant) * 4294967296.0;
    decay0_ratio = (uint64_t)ratio;
#endif
  }

  void SetSustain0Level(uint16_t new_sustain_level) {
    uint16_t rounded_sustain_level = (new_sustain_level >> 6) << 6;
    if (rounded_sustain_level == sustain0_level_param) {
      return;
    }
    sustain0_level_param = rounded_sustain_level;
#ifdef DECAY_DISTORTION
    distortion_depth = (double)rounded_sustain_level / 128; // 0 to 512;
#else
    sustain0_level = ((uint64_t)rounded_sustain_level * (uint64_t)rounded_sustain_level);
#endif
  }

  void SetDecayTime(uint16_t new_decay_time) {
    uint16_t rounded_decay_time = (new_decay_time >> 6) << 6;
    if (rounded_decay_time == decay_time_param) {
      return;
    }

    decay_time_param = rounded_decay_time;
    // decay_time_constant = exp(rounded_decay_time * 0.00017 + 2);
    decay_time_constant = 7.5 + 7.0e-10 * rounded_decay_time * rounded_decay_time * rounded_decay_time;
    decay_ratio = 0xffffffff / decay_time_constant;
  }

  void SetSustainLevel(uint16_t new_sustain_level) {
    uint16_t rounded_sustain_level = (new_sustain_level >> 6) << 6;
    if (rounded_sustain_level == sustain_level_param) {
      return;
    }
    sustain_level_param = rounded_sustain_level;
    sustain_level = (((uint64_t)rounded_sustain_level >> 1) + 32768) * (uint64_t)rounded_sustain_level;
  }

  void SetReleaseTime(uint16_t new_release_time) {
    uint16_t rounded_release_time = (new_release_time >> 6) << 6;
    if (rounded_release_time == release_time_param) {
      return;
    }

    release_time_param = rounded_release_time;
    release_time_constant = 7.5 + 7.0e-10 * rounded_release_time * rounded_release_time * rounded_release_time;
    release_ratio = 0xffffffff / release_time_constant;
    // double ratio = exp(-delta_t / exp(rounded_release_time * 0.000115 - 4.6));
    // release_ratio = (uint64_t)(ratio * 4294967296.0);
  }
};

class EnvelopeGenerator {
 private:
  uint16_t dac_index_;
  GPIO_TypeDef *gpiox_;
  uint16_t gpio_pin_;

  int8_t trigger_ = 0;
  uint16_t velocity_ = 0;
  bool is_gate_on_ = false;

  uint64_t current_value_ = 0;
  uint64_t target_value_ = 0;
  uint64_t peak_value_ = 0;

  const EnvelopeGeneratorParams &params_;

  enum class Phase {
    ATTACKING,
    DECAYING,
    SUSTAINING,
    RELEASED,
  };

  Phase phase_ = Phase::RELEASED;
  void (*UpdateValue)(EnvelopeGenerator *instance) = UpdateRelease;

 public:
  EnvelopeGenerator(uint16_t dac_index, GPIO_TypeDef *gpiox, uint16_t gpio_pin, const EnvelopeGeneratorParams &params)
      :
      dac_index_(dac_index),
      gpiox_(gpiox),
      gpio_pin_(gpio_pin),
      params_ { params } {
  }
  EnvelopeGenerator() = delete;
  ~EnvelopeGenerator() = default;

  void Initialize() {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    GateOff();
  }

  uint32_t GetVoiceId() {
    return (uint32_t)dac_index_ - 1;
  }

  void GateOn(uint16_t velocity) {
    velocity_ = velocity;
    trigger_ = 1;
    is_gate_on_ = true;
  }

  void AnalogGateOn(uint16_t velocity) {
    velocity_ = velocity;
    trigger_ = 2;
    is_gate_on_ = true;
  }

  void GateOff() {
    trigger_ = -1;
    is_gate_on_ = false;
  }

  bool IsGateOn() {
    return is_gate_on_;
  }

  void Update() {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, current_value_ >> 20);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, current_value_ >> 20);
    if (trigger_ > 0) {
      Trigger();
    } else if (trigger_ < 0) {
      Release();
    }
    UpdateValue(this);
  }

 private:
  void Trigger() {
    uint64_t level;
    if (trigger_ == 1) {
      level = ((uint64_t)velocity_ * (uint64_t)velocity_) >> 1;
    } else {
      level = (uint64_t)velocity_ << 15;
    }
    trigger_ = 0;
    target_value_ = level * 1.2;
    peak_value_ = level;
    phase_ = Phase::ATTACKING;
    UpdateValue = UpdateAttack;
    HAL_GPIO_WritePin(gpiox_, gpio_pin_, GPIO_PIN_SET);
  }

  void Release() {
    trigger_ = 0;
    target_value_ = 0;
    phase_ = Phase::RELEASED;
    UpdateValue = UpdateRelease;
    HAL_GPIO_WritePin(gpiox_, gpio_pin_, GPIO_PIN_RESET);
  }

  static void UpdateAttack(EnvelopeGenerator *self) {
    uint64_t diff = self->target_value_ - self->current_value_;
    diff *= self->params_.attack_ratio;
    diff >>= 32;
    // self->current_value_ = self->target_value_ - diff;
    self->current_value_ += diff;
    if (self->current_value_ >= self->peak_value_) {
      self->current_value_ = self->peak_value_;
#ifdef DECAY_DISTORTION
      self->phase_ = Phase::SUSTAINING;
      self->UpdateValue = UpdateDecay;
#else
      self->phase_ = Phase::DECAYING;
      self->UpdateValue = UpdateDecay0;
#endif
    }
  }

  static void UpdateDecay0(EnvelopeGenerator *self) {
    uint64_t switch_value = (self->peak_value_ * self->params_.sustain0_level) >> 32;
    self->target_value_ = 0;
    uint64_t diff = self->current_value_ - self->target_value_;
    diff *= self->params_.decay0_ratio;
    diff >>= 32;
    self->current_value_ = self->target_value_ + diff;
    if (self->current_value_ <= switch_value) {
      self->phase_ = Phase::SUSTAINING;
      self->UpdateValue = UpdateDecay;
    }
  }

  static void UpdateDecay(EnvelopeGenerator *self) {
    self->target_value_ = (self->peak_value_ * self->params_.sustain_level) >> 32;
    if (self->target_value_ <= self->current_value_) {
      uint64_t diff = self->current_value_ - self->target_value_;
      diff *= self->params_.decay_ratio;
      self->current_value_ -= diff >> 32;
    } else {
      uint64_t diff = self->target_value_ - self->current_value_;
      diff *= self->params_.decay_ratio;
      self->current_value_ += diff >> 32;
    }
    /*
    if (diff > ((self->peak_value_ - self->target_value_) >> 1)) {
      diff *= self->params_.decay_ratio * (1.0 - ((double)diff / (self->peak_value_ - self->target_value_) - 0.5) * 8.0);
    } else {
      diff *= self->params_.decay_ratio;
    }
    */
    // self->current_value_ += diff >> 32;
#if 0
    if (self->current_value_ > self->target_value_ ) {
      uint64_t diff = self->current_value_ - self->target_value_;
#ifdef DECAY_DISTORTION
      uint64_t max_diff = self->peak_value_ - self->target_value_;
      double distortion = pow((double)diff / (double)max_diff, self->params_.distortion_steepness);
      double bent_time_constant = self->params_.decay_time_constant / (1.0 + distortion * self->params_.distortion_depth);
      uint64_t ratio = exp(-self->params_.delta_t / bent_time_constant) * 4294967296.0;
      diff *= ratio;
#else
      diff *= self->params_.decay_ratio;
#endif
      diff >>= 32;
      self->current_value_ = self->target_value_ + diff;
    } else {
      uint64_t diff = self->target_value_ - self->current_value_;
      diff *= self->params_.decay_ratio;
      diff >>= 32;
      self->current_value_ = self->target_value_ - diff;
    }
#endif
  }

  static void UpdateRelease(EnvelopeGenerator *self) {
    /*
    uint64_t diff = self->current_value_ - self->target_value_;
    diff *= self->params_.release_ratio;
    */
    uint64_t diff = self->current_value_ * self->params_.release_ratio;
    // diff >>= 32;
    self->current_value_ -= diff >> 32;
  }
};

static bool pulse = false;
static EnvelopeGeneratorParams eg_params{};
static EnvelopeGenerator eg_voice_0{1, IND_GATE_1_GPIO_Port, IND_GATE_1_Pin, eg_params};
static bool physical_gate_input_enabled = false;

class EgMessageHandler : public MessageHandler {
 public:
  EgMessageHandler() = default;
  ~EgMessageHandler() = default;

  void Handle(const CanRxMessage& message) {
    if (physical_gate_input_enabled) {
      return;
    }
    auto id = message.GetId();
    uint32_t voice_id = 0;
    if (id == A3_ID_MIDI_VOICE_BASE + voice_id) {
      uint8_t index = 0;
      do {
        index = ParseMessage(message.GetData(), voice_id, index);
      } while (index < message.GetDlc());
    }
  }

 private:
  uint8_t ParseMessage(const uint8_t *data, uint32_t voice_id, uint8_t index) {
    auto op = data[index++];
    switch (op) {
    case A3_VOICE_MSG_GATE_ON:
      if (index < 6) {
        uint16_t velocity = (data[index] << 8) + data[index + 1];
        index += 2;
        if (eg_voice_0.GetVoiceId() == voice_id) {
          eg_voice_0.GateOn(velocity);
        }

      }
      break;
    case A3_VOICE_MSG_GATE_OFF:
      if (eg_voice_0.GetVoiceId() == voice_id) {
        eg_voice_0.GateOff();
      }
      break;
    }
    return index;
  }
};

}  // namespace analog3

static analog3::EgMessageHandler message_handler;

// C API for the main program /////////////////////////////////////////////////////////////////////////

void InitializeEnvelopeGenerator() {
  analog3::eg_params.Initialize();
  analog3::eg_voice_0.Initialize();
  a3->InjectMessageHandler(&message_handler);
}

static uint32_t cycles = 0;
void NudgeEnvelopeGenerator() {
  if (cycles++ % EG_UPDATE_CYCLES == 0) {
    analog3::pulse = true;
  }
}

void UpdateEnvelopeGenerator() {
  if (analog3::pulse) {
    analog3::eg_voice_0.Update();
    analog3::pulse = false;
  }
}

void TogglePhysicalGateInput() {
  analog3::physical_gate_input_enabled = !analog3::physical_gate_input_enabled;
}

uint8_t IsPhysicalGateInputEnabled() {
  return analog3::physical_gate_input_enabled;
}

void SetAttackTime(void *arg) {
  analog3::eg_params.SetAttackTime(*reinterpret_cast<uint16_t*>(arg));
}

void SetDecay0Time(void *arg) {
  analog3::eg_params.SetDecay0Time(*reinterpret_cast<uint16_t*>(arg));
}

void SetSustain0Level(void *arg) {
  analog3::eg_params.SetSustain0Level(*reinterpret_cast<uint16_t*>(arg));
}

void SetDecayTime(void *arg) {
  analog3::eg_params.SetDecayTime(*reinterpret_cast<uint16_t*>(arg));
}

void SetSustainLevel(void *arg) {
  analog3::eg_params.SetSustainLevel(*reinterpret_cast<uint16_t*>(arg));
}

void SetReleaseTime(void *arg) {
  analog3::eg_params.SetReleaseTime(*reinterpret_cast<uint16_t*>(arg));
}

void CheckGate1(void *arg) {
  if (!analog3::physical_gate_input_enabled) {
    return;
  }
  // The range of the gate level is [0:65535] that projects gate voltage of range [8v:0v].
  // The gate-on minimum voltage is 3V that increases up to 8V.
  // Value of (voltage - 3) represents the output level.
  uint16_t gate_level = 65535 - *reinterpret_cast<uint16_t*>(arg);
  if (!analog3::eg_voice_0.IsGateOn()) {
    if (gate_level > 22527) { // about 2.75V
      const int32_t kThreshold = 24000; // about 3V
      int32_t velocity = (int32_t)gate_level - kThreshold;
      velocity = velocity * 32768 / ((65536 - kThreshold) / 2);
      if (velocity < 0) {
        velocity = 0;
      } else if (velocity > 65535) {
        velocity = 65535;
      }
      analog3::eg_voice_0.AnalogGateOn((uint16_t)velocity);
      // HAL_GPIO_WritePin(IND_GATE_1_GPIO_Port, IND_GATE_1_Pin, GPIO_PIN_SET);
    }
  } else if (gate_level < 16384) { // about 2.0V
    analog3::eg_voice_0.GateOff();
    // HAL_GPIO_WritePin(IND_GATE_1_GPIO_Port, IND_GATE_1_Pin, GPIO_PIN_RESET);
  }
}
