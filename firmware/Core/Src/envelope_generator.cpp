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
  double distortion_threshold = 0;

  double delta_t = 0;

  void Initialize() {
    delta_t = (double)htim3.Init.Period * (double)EG_UPDATE_CYCLES / (double)HAL_RCC_GetSysClockFreq();
  }

  void SetAttackTime(uint16_t new_attack_time) {
    attack_time_param = new_attack_time;
    attack_time_constant = 1.0 + 3.5e-10 * new_attack_time * new_attack_time * new_attack_time;
    attack_ratio = 0xffffffff / attack_time_constant;
  }

  void SetDecay0Time(uint16_t new_decay_time) {
    decay0_time_param = new_decay_time;
#ifdef DECAY_DISTORTION
    distortion_steepness = ((double)new_decay_time + 65536.0) * (double)new_decay_time / 67108864.0;
#else
    double time_constant = exp(rounded_decay_time * 0.00015 - 7.5);
    double ratio = exp(-delta_t / time_constant) * 4294967296.0;
    decay0_ratio = (uint64_t)ratio;
#endif
  }

  void SetSustain0Level(uint16_t new_sustain_level) {
    sustain0_level_param = new_sustain_level;
#ifdef DECAY_DISTORTION
    distortion_threshold = ((double)new_sustain_level + 65536.0) * (double)new_sustain_level / 8589934592.0; // 0 to 1;
#else
    sustain0_level = ((uint64_t)new_sustain_level * (uint64_t)new_sustain_level);
#endif
  }

  void SetDecayTime(uint16_t new_decay_time) {
    decay_time_param = new_decay_time;
    decay_time_constant = 7.5 + 7.0e-10 * new_decay_time * new_decay_time * new_decay_time;
    decay_ratio = 0xffffffff / decay_time_constant;
  }

  void SetSustainLevel(uint16_t new_sustain_level) {
    sustain_level_param = new_sustain_level;
    sustain_level = (((uint64_t)new_sustain_level >> 1) + 32768) * (uint64_t)new_sustain_level;
  }

  void SetReleaseTime(uint16_t new_release_time) {
    release_time_param = new_release_time;
    release_time_constant = 7.5 + 7.0e-10 * new_release_time * new_release_time * new_release_time;
    release_ratio = 0xffffffff / release_time_constant;
  }
};

class EnvelopeGenerator {
 private:
  TIM_HandleTypeDef *pwm_timer_;
  uint32_t pwm_timer_channel_;

  GPIO_TypeDef *gpiox_;
  uint16_t gpio_pin_;

  uint16_t voice_id_ = 0;

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
  EnvelopeGenerator(TIM_HandleTypeDef *pwm_timer, uint32_t pwm_timer_channel,
                    GPIO_TypeDef *gpiox, uint16_t gpio_pin, const EnvelopeGeneratorParams &params)
      :
      pwm_timer_(pwm_timer),
      pwm_timer_channel_(pwm_timer_channel),
      gpiox_(gpiox),
      gpio_pin_(gpio_pin),
      params_ { params } {
  }
  EnvelopeGenerator() = delete;
  ~EnvelopeGenerator() = default;

  void Initialize(uint16_t voice_id) {
    voice_id_ = voice_id;
    __HAL_TIM_SET_COMPARE(pwm_timer_, pwm_timer_channel_, 0);
    GateOff();
  }

  uint16_t GetVoiceId() {
    return voice_id_;
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
    __HAL_TIM_SET_COMPARE(pwm_timer_, pwm_timer_channel_, current_value_ >> 20);
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
      // Add an offset of 1/32 level to avoid silence with low velocity
      level = (((uint64_t)velocity_ * (uint64_t)velocity_) * 31 + 0xffffffff) >> 6;
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
    uint64_t diff;
    uint64_t max_diff;
    int64_t polarity;
    if (self->target_value_ <= self->current_value_) {
      diff = self->current_value_ - self->target_value_;
      max_diff = self->peak_value_ - self->target_value_;
      polarity = -1;
    } else {
      diff = self->target_value_ - self->current_value_;
      max_diff = self->target_value_;
      polarity = 1;
    }
    double thr = self->params_.distortion_threshold;
    if (diff > (max_diff * thr)) {
      uint64_t ratio = self->params_.decay_ratio * (1.0 + ((double)diff / max_diff - thr) * self->params_.distortion_steepness);
      if (ratio > 0xffff0000) {
        ratio = 0xffff0000;
      }
      diff *= ratio;
    } else {
      diff *= self->params_.decay_ratio;
    }
    self->current_value_ += (diff >> 32) * polarity;
#if 0
    if (self->current_value_ > self->target_value_ ) {
      uint64_t diff = self->current_value_ - self->target_value_;
#ifdef DECAY_DISTORTION
      uint64_t max_diff = self->peak_value_ - self->target_value_;
      double distortion = pow((double)diff / (double)max_diff, self->params_.distortion_steepness);
      double bent_time_constant = self->params_.decay_time_constant / (1.0 + distortion * self->params_.distortion_threshold);
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
    uint64_t diff = self->current_value_ * self->params_.release_ratio;
    self->current_value_ -= diff >> 32;
  }
};

// The output PWM overflow interrupt handler nudges for EG updates
static bool nudged = false;
static EnvelopeGeneratorParams eg_params{};
static EnvelopeGenerator eg_voice_1{&htim3, TIM_CHANNEL_1, IND_GATE_1_GPIO_Port, IND_GATE_1_Pin, eg_params};
static EnvelopeGenerator eg_voice_2{&htim3, TIM_CHANNEL_4, IND_GATE_2_GPIO_Port, IND_GATE_2_Pin, eg_params};
static bool physical_gate_input_enabled = false;

class EgMessageHandler : public MessageHandler {
 private:

 public:
  EgMessageHandler() = default;
  ~EgMessageHandler() = default;

  void Handle(const CanRxMessage& message) {
    if (physical_gate_input_enabled) {
      return;
    }
    auto id = message.GetId();
    if (id >= A3_ID_MIDI_VOICE_BASE && id < A3_ID_MIDI_REAL_TIME) {
      uint32_t voice_id = id - A3_ID_MIDI_VOICE_BASE;
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
    case A3_VOICE_MSG_GATE_ON: {
        uint16_t velocity = (data[index] << 8) + data[index + 1];
        index += 2;
        if (eg_voice_1.GetVoiceId() == voice_id) {
          eg_voice_1.GateOn(velocity);
        }
        if (eg_voice_2.GetVoiceId() == voice_id) {
          eg_voice_2.GateOn(velocity);
        }
      }
      break;
    case A3_VOICE_MSG_GATE_OFF:
      if (eg_voice_1.GetVoiceId() == voice_id) {
        eg_voice_1.GateOff();
      }
      if (eg_voice_2.GetVoiceId() == voice_id) {
        eg_voice_2.GateOff();
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
  analog3::eg_voice_1.Initialize(0);
  analog3::eg_voice_2.Initialize(1);
  a3->InjectMessageHandler(&message_handler);
}

static uint32_t cycles = 0;
void NudgeEnvelopeGenerator() {
  if (cycles++ % EG_UPDATE_CYCLES == 0) {
    analog3::nudged = true;
  }
}

void UpdateEnvelopeGenerator() {
  if (analog3::nudged) {
    analog3::eg_voice_1.Update();
    analog3::eg_voice_2.Update();
    analog3::nudged = false;
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

template <analog3::EnvelopeGenerator *EG>
void CheckGate(const uint16_t &adc_read) {
  if (!analog3::physical_gate_input_enabled) {
    return;
  }
  // The range of the gate level is [0:65535] that projects gate voltage of range [8v:0v].
  // The gate-on minimum voltage is 3V that increases up to 8V.
  // Value of (voltage - 3) represents the output level.
  uint16_t gate_level = 65535 - adc_read;
  if (!EG->IsGateOn()) {
    if (gate_level > 22527) { // about 2.75V
      const int32_t kThreshold = 24000; // about 3V
      int32_t velocity = (int32_t)gate_level - kThreshold;
      velocity = velocity * 32768 / ((65536 - kThreshold) / 2);
      if (velocity < 0) {
        velocity = 0;
      } else if (velocity > 65535) {
        velocity = 65535;
      }
      EG->AnalogGateOn((uint16_t)velocity);
    }
  } else if (gate_level < 16384) { // about 2.0V
    EG->GateOff();
  }
}



void CheckGate1(void *arg) {
  CheckGate<&analog3::eg_voice_1>(*reinterpret_cast<uint16_t*>(arg));
}

void CheckGate2(void *arg) {
  CheckGate<&analog3::eg_voice_2>(*reinterpret_cast<uint16_t*>(arg));
}
