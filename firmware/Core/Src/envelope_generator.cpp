/*
 * envelope_generator.cpp
 *
 *  Created on: Nov 3, 2025
 *      Author: naoki
 */

#include <math.h>

#include <algorithm>

#include "main.h"

#include "analog3/analog3.h"
#include "analog3/definitions.h"
#include "analog3/stm32impl.h"
#include "envelope_generator.h"

#define DECAY_DISTORTION 1

extern analog3::Analog3 *a3;

namespace analog3 {

const uint8_t kEgModeDefault = 0;
const uint8_t kEgModeTwoPhaseDecay = 1;
const uint8_t kEgModeLinear = 2;
const uint8_t kEgModeGritty = 3;
const uint8_t kNumEgModes = 4;

/**
 * Provides user interface to handle EG parameters.
 */
class EnvelopeGeneratorDefaultPanel {
 private:
  EnvelopeGeneratorParams *params_1_;
  EnvelopeGeneratorParams *params_2_;

 public:
  EnvelopeGeneratorDefaultPanel(EnvelopeGeneratorParams *params_1,
                                EnvelopeGeneratorParams *params_2)
      :
      params_1_(params_1),
      params_2_(params_2) {

  }

  void Initialize() {
    params_1_->voice_id = 0;
    params_2_->voice_id = 1;
  }

  bool IsPhysicalGateEnabled() const {
    return params_1_->physical_gate_enabled;
  }

  void TogglePhysicalGateInput() {
    params_1_->physical_gate_enabled = !params_1_->physical_gate_enabled;
    params_2_->physical_gate_enabled = params_1_->physical_gate_enabled;
  }

  void SetAttackTime(uint16_t new_attack_time) {
    params_1_->attack_time_param = new_attack_time;
    params_2_->attack_time_param = params_1_->attack_time_param;

    double attack_time_constant = 1.0 + 3.5e-10 * new_attack_time * new_attack_time * new_attack_time;
    params_1_->attack_ratio = 0xffffffff / attack_time_constant;
    params_2_->attack_ratio = params_1_->attack_ratio;
  }

  void SetDecay0Time(uint16_t new_decay_time) {
    params_1_->decay0_time_param = new_decay_time;
    params_2_->decay0_time_param = params_1_->decay0_time_param;

    switch (params_1_->mode) {
    case kEgModeTwoPhaseDecay: {
      double decay_time_constant = 7.5 + 7.0e-11 * new_decay_time * new_decay_time * new_decay_time;
      params_1_->decay0_ratio = 0xffffffff / decay_time_constant;
      params_2_->decay0_ratio = params_1_->decay0_ratio;
      break;
    }
    case kEgModeGritty: {
      double x = 65535.0 - new_decay_time;
      double time_constant = 3.5 + 1.2e-12 * x * x * x;
      params_1_->gritty_ratio = 0xffffffff / time_constant;
      params_2_->gritty_ratio = params_1_->gritty_ratio;
      break;
    }
    default:
      params_1_->distortion_steepness = ((double)new_decay_time + 65536.0) * (double)new_decay_time / 67108864.0;
      params_2_->distortion_steepness = params_1_->distortion_steepness;
    }
  }

  void SetSustain0Level(uint16_t new_sustain_level) {
    params_1_->sustain0_level_param = new_sustain_level;
    params_2_->sustain0_level_param = params_1_->sustain0_level_param;

    switch (params_1_->mode) {
      case kEgModeTwoPhaseDecay:
      case kEgModeGritty:
        params_1_->sustain0_level = (((uint64_t)new_sustain_level >> 1) + 32768) * (uint64_t)new_sustain_level;
        params_2_->sustain0_level = params_1_->sustain0_level;
        break;
      default:
        params_1_->distortion_threshold = ((double)new_sustain_level + 65536.0) * (double)new_sustain_level / 8589934592.0; // 0 to 1;
        params_2_->distortion_threshold = params_1_->distortion_threshold;
    }
  }

  void SetDecayTime(uint16_t new_decay_time) {
    params_1_->decay_time_param = new_decay_time;
    params_2_->decay_time_param = params_1_->decay_time_param;

    double decay_time_constant = 7.5 + 7.0e-10 * new_decay_time * new_decay_time * new_decay_time;
    params_1_->decay_ratio = 0xffffffff / decay_time_constant;
    params_2_->decay_ratio = params_1_->decay_ratio;
  }

  void SetSustainLevel(uint16_t new_sustain_level) {
    params_1_->sustain_level_param = new_sustain_level;
    params_2_->sustain_level_param = params_1_->sustain_level_param;

    params_1_->sustain_level = (((uint64_t)new_sustain_level >> 1) + 32768) * (uint64_t)new_sustain_level;
    params_2_->sustain_level = params_1_->sustain_level;
  }

  void SetReleaseTime(uint16_t new_release_time) {
    params_1_->release_time_param = new_release_time;
    params_2_->release_time_param = params_1_->release_time_param;

    double release_time_constant = 7.5 + 7.0e-10 * new_release_time * new_release_time * new_release_time;
    params_1_->release_ratio = 0xffffffff / release_time_constant;
    params_2_->release_ratio = params_1_->release_ratio;
  }

  void SwitchEnvelopeGenerationMode() {
    params_1_->mode = (params_1_->mode + 1) % kNumEgModes;
    params_2_->mode = params_1_->mode;
  }
};

class EnvelopeGenerator {
 private:
  TIM_HandleTypeDef *pwm_timer_;
  uint32_t pwm_timer_channel_;

  GPIO_TypeDef *gpiox_;
  uint16_t gpio_pin_;

  int8_t trigger_ = 0;
  uint16_t velocity_ = 0;
  bool is_gate_on_ = false;

  uint64_t current_value_ = 0;
  uint64_t target_value_ = 0;
  uint64_t peak_value_ = 0;

  uint32_t noise_register_ = ~0;

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

  void Initialize() {
    __HAL_TIM_SET_COMPARE(pwm_timer_, pwm_timer_channel_, 0);
    GateOff();
  }

  uint16_t GetVoiceId() const {
    return params_.voice_id;
  }

  bool IsPhysicalGateEnabled() const {
    return params_.physical_gate_enabled;
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
    __HAL_TIM_SET_COMPARE(pwm_timer_, pwm_timer_channel_, current_value_ >> 21);
    if (trigger_ > 0) {
      Trigger();
    } else if (trigger_ < 0) {
      Release();
    }
    UpdateValue(this);
  }

 private:
  void Trigger() {
    uint64_t velocity;
    switch (params_.mode) {
    case kEgModeLinear:
      velocity = ((uint64_t)velocity_ * params_.decay0_time_param + (0xfffful - params_.decay0_time_param) * 0xffff) >> 16;
      UpdateValue = UpdateAttackLinear;
      break;
    case kEgModeGritty:
      velocity = (uint64_t)velocity_;
      UpdateValue = UpdateAttackGritty;
      break;
    default:
      velocity = (uint64_t)velocity_;
      UpdateValue = UpdateAttack;
    }
    uint64_t level;
    if (trigger_ == 1) {
      // Add an offset of 1/32 level to avoid silence with low velocity
      level = ((velocity * velocity) * 31 + 0xffffffff) >> 6;
    } else {
      level = velocity << 15;
    }
    trigger_ = 0;
    target_value_ = level * 1.2;
    peak_value_ = level;
    phase_ = Phase::ATTACKING;
    HAL_GPIO_WritePin(gpiox_, gpio_pin_, GPIO_PIN_SET);
  }

  void Release() {
    trigger_ = 0;
    target_value_ = 0;
    peak_value_ = current_value_;
    phase_ = Phase::RELEASED;
    if (params_.mode == kEgModeLinear) {
      UpdateValue = UpdateReleaseLinear;
    } else {
      UpdateValue = UpdateRelease;
    }
    HAL_GPIO_WritePin(gpiox_, gpio_pin_, GPIO_PIN_RESET);
  }

  uint8_t UpdateNoise() {
    uint8_t temp = (noise_register_ >> 12) & 1;
    temp ^= (noise_register_ >> 30) & 1;
    noise_register_ <<= 1;
    noise_register_ += temp;
    return temp;
  }

  static void UpdateAttack(EnvelopeGenerator *self) {
    uint64_t diff = self->target_value_ - self->current_value_;
    diff *= self->params_.attack_ratio;
    diff >>= 32;
    self->current_value_ += diff;
    if (self->current_value_ >= self->peak_value_) {
      self->current_value_ = self->peak_value_;
      if (self->params_.mode == kEgModeTwoPhaseDecay) {
        self->phase_ = Phase::DECAYING;
        self->UpdateValue = UpdateDecay0;
      } else {
        self->phase_ = Phase::SUSTAINING;
        self->UpdateValue = UpdateDecay;
      }
    }
  }

  static void UpdateAttackLinear(EnvelopeGenerator *self) {
    uint64_t diff = (self->params_.attack_ratio * self->peak_value_) >> 34;
    self->current_value_ += diff;
    if (self->current_value_ >= self->peak_value_) {
      self->current_value_ = self->peak_value_;
      self->phase_ = Phase::SUSTAINING;
      self->UpdateValue = UpdateDecayLinear;
    }
  }

  static void UpdateAttackGritty(EnvelopeGenerator *self) {
    uint64_t diff = self->target_value_ - self->current_value_ * self->UpdateNoise();
    diff *= self->params_.attack_ratio;
    diff >>= 31;
    self->current_value_ += diff;
    if (self->current_value_ >= self->peak_value_) {
      self->current_value_ = self->peak_value_;
      self->phase_ = Phase::SUSTAINING;
      self->UpdateValue = UpdateDecayGritty;
    }
  }

  static void UpdateDecay0(EnvelopeGenerator *self) {
    uint64_t switch_value = (self->peak_value_ * self->params_.sustain0_level) >> 32;
    self->target_value_ = 0;

    uint64_t diff = self->current_value_ * self->params_.decay0_ratio;
    self->current_value_ -= diff >> 32;
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
    if (self->params_.mode != kEgModeTwoPhaseDecay && diff > (max_diff * thr)) {
      uint64_t ratio = self->params_.decay_ratio * (1.0 + ((double)diff / max_diff - thr) * self->params_.distortion_steepness);
      if (ratio > 0xffff0000) {
        ratio = 0xffff0000;
      }
      diff *= ratio;
    } else {
      diff *= self->params_.decay_ratio;
    }
    self->current_value_ += (diff >> 32) * polarity;
  }

  static void UpdateDecayLinear(EnvelopeGenerator *self) {
    self->target_value_ = (self->peak_value_ * self->params_.sustain_level) >> 32;
    uint64_t max_diff = self->peak_value_ - self->target_value_;
    uint64_t diff;
    int64_t polarity;
    if (self->target_value_ <= self->current_value_) {
      diff = std::min((self->params_.decay_ratio * max_diff) >> 34, self->current_value_ - self->target_value_);
      polarity = -1;
    } else {
      diff = std::min((self->params_.decay_ratio * max_diff) >> 34, self->target_value_ - self->current_value_);
      polarity = 1;
    }
    self->current_value_ += diff * polarity;
  }

  static void UpdateDecayGritty(EnvelopeGenerator *self) {
    self->target_value_ = (self->peak_value_ * self->params_.sustain_level) >> 32;
    uint8_t push = self->UpdateNoise();
    uint64_t diff;
    int64_t polarity;
    if (self->current_value_ >= self->target_value_) {
      diff = self->current_value_ - self->target_value_;
      polarity = -1;
    } else {
      diff = self->target_value_ - self->current_value_;
      polarity = 1;
    }
    diff *= self->params_.decay_ratio;
    uint64_t level = self->current_value_ + (diff >> 32) * polarity;
    uint64_t diff2 = (level * self->params_.sustain0_level) >> 32;
    uint64_t target;
    if (push) {
      target = level + diff2;
    } else {
      target = level - diff2;
    }
    int64_t polarity2;
    if (self->current_value_ >= target) {
      diff2 = self->current_value_ - target;
      polarity2 = -1;
    } else {
      diff2 = target - self->current_value_;
      polarity2 = 1;
    }
    diff2 *= self->params_.gritty_ratio;
    self->current_value_ = level + (diff2 >> 32) * polarity2;
    if (self->current_value_ > 0xffffffff) {
      self->current_value_ = 0xffffffff;
    }
  }

  static void UpdateRelease(EnvelopeGenerator *self) {
    uint64_t diff = self->current_value_ * self->params_.release_ratio;
    self->current_value_ -= diff >> 32;
  }

  static void UpdateReleaseLinear(EnvelopeGenerator *self) {
    uint64_t diff = std::min((self->params_.release_ratio * self->peak_value_) >> 34, self->current_value_);
    self->current_value_ -= diff;
  }
};

// The output PWM overflow interrupt handler nudges for EG updates
static bool update_envelope_generators = false;
static bool update_indicators = false;
EnvelopeGeneratorParams eg_params_1{};
EnvelopeGeneratorParams eg_params_2{};
static EnvelopeGeneratorDefaultPanel panel{&eg_params_1, &eg_params_2};
static EnvelopeGenerator eg_voice_1{&htim3, TIM_CHANNEL_1, IND_GATE_1_GPIO_Port, IND_GATE_1_Pin, eg_params_1};
static EnvelopeGenerator eg_voice_2{&htim3, TIM_CHANNEL_4, IND_GATE_2_GPIO_Port, IND_GATE_2_Pin, eg_params_2};

class EgMessageHandler : public MessageHandler {
 private:

 public:
  EgMessageHandler() = default;
  ~EgMessageHandler() = default;

  void Handle(const CanRxMessage& message) {
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
        if (!eg_voice_1.IsPhysicalGateEnabled() && eg_voice_1.GetVoiceId() == voice_id) {
          eg_voice_1.GateOn(velocity);
        }
        if (!eg_voice_2.IsPhysicalGateEnabled() && eg_voice_2.GetVoiceId() == voice_id) {
          eg_voice_2.GateOn(velocity);
        }
      }
      break;
    case A3_VOICE_MSG_GATE_OFF:
      if (!eg_voice_1.IsPhysicalGateEnabled() && eg_voice_1.GetVoiceId() == voice_id) {
        eg_voice_1.GateOff();
      }
      if (!eg_voice_2.IsPhysicalGateEnabled() && eg_voice_2.GetVoiceId() == voice_id) {
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
  analog3::panel.Initialize();
  analog3::eg_voice_1.Initialize();
  analog3::eg_voice_2.Initialize();
  a3->InjectMessageHandler(&message_handler);
}

static uint32_t cycles = 0;
void NudgeEnvelopeGenerator() {
  if (cycles % EG_UPDATE_CYCLES == 0) {
    analog3::update_envelope_generators = true;
  }
  if (cycles % INDICATORS_UPDATE_CYCLES == 0) {
    analog3::update_indicators = true;
  }
}

void UpdateEnvelopeGenerator() {
  if (analog3::update_envelope_generators) {
    analog3::eg_voice_1.Update();
    analog3::eg_voice_2.Update();
    analog3::update_envelope_generators = false;
  }
  if (analog3::update_indicators) {
    HAL_GPIO_WritePin(IND_ANALOG_GATE_GPIO_Port, IND_ANALOG_GATE_Pin,
                      analog3::eg_params_1.physical_gate_enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IND_EG_MODE_0_GPIO_Port, IND_EG_MODE_0_Pin, (analog3::eg_params_1.mode & 0x1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IND_EG_MODE_1_GPIO_Port, IND_EG_MODE_1_Pin, (analog3::eg_params_1.mode & 0x2) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IND_EG_MODE_2_GPIO_Port, IND_EG_MODE_2_Pin, (analog3::eg_params_1.mode & 0x4) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    analog3::update_indicators = false;
  }
}

void TogglePhysicalGateInput() {
  analog3::panel.TogglePhysicalGateInput();
}

uint8_t IsPhysicalGateEnabled() {
  return analog3::panel.IsPhysicalGateEnabled();
}

void SwitchEnvelopeGenerationMode() {
  analog3::panel.SwitchEnvelopeGenerationMode();
}

void SetAttackTime(void *arg) {
  analog3::panel.SetAttackTime(*reinterpret_cast<uint16_t*>(arg));
}

void SetDecay0Time(void *arg) {
  analog3::panel.SetDecay0Time(*reinterpret_cast<uint16_t*>(arg));
}

void SetSustain0Level(void *arg) {
  analog3::panel.SetSustain0Level(*reinterpret_cast<uint16_t*>(arg));
}

void SetDecayTime(void *arg) {
  analog3::panel.SetDecayTime(*reinterpret_cast<uint16_t*>(arg));
}

void SetSustainLevel(void *arg) {
  analog3::panel.SetSustainLevel(*reinterpret_cast<uint16_t*>(arg));
}

void SetReleaseTime(void *arg) {
  analog3::panel.SetReleaseTime(*reinterpret_cast<uint16_t*>(arg));
}

template <analog3::EnvelopeGenerator *EG>
void CheckGate(const uint16_t &adc_read) {
  if (!EG->IsPhysicalGateEnabled()) {
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
