/*
 * envelope_generator.h
 *
 *  Created on: Nov 3, 2025
 *      Author: naoki
 */

#ifndef INC_ENVELOPE_GENERATOR_H_
#define INC_ENVELOPE_GENERATOR_H_

#ifdef __cplusplus

namespace analog3 {

extern const uint8_t kEgModeDefault;
extern const uint8_t kEgModeTwoPhaseDecay;
extern const uint8_t kEgModeLinear;
extern const uint8_t kEgModeGritty;
extern const uint8_t kNumEgModes;

struct EnvelopeGeneratorParams {
  // config parameters
  uint16_t attack_time_param = 0;
  uint16_t decay0_time_param = 0;
  uint16_t sustain0_level_param = 0;
  uint16_t decay_time_param = 0;
  uint16_t sustain_level_param = 0;
  uint16_t release_time_param = 0;

  uint16_t voice_id = 0;
  bool physical_gate_enabled = false;

  uint8_t mode = kEgModeDefault;

  // derived parameters
  uint64_t attack_ratio = 0;
  uint64_t decay0_ratio = 0;
  uint64_t gritty_ratio = 0;
  uint64_t decay_ratio = 0;
  uint64_t sustain0_level = 0xffffffff;
  uint64_t sustain_level = 0xffffffff;
  uint64_t release_ratio = 0;

  double distortion_steepness = 0;
  double distortion_threshold = 0;
};

extern EnvelopeGeneratorParams eg_params_1;
extern EnvelopeGeneratorParams eg_params_2;

} // namespace analog3

extern "C" {
#endif

/*
 * The HAL main program is written in C for convenience.
 *
 * Because of that, EG needs C API as the interface to the main program
 * although it's implemented in C++ internally.
 */
extern void InitializeEnvelopeGenerator();
extern void NudgeEnvelopeGenerator();
extern void UpdateEnvelopeGenerator();
extern void TogglePhysicalGateInput();
extern uint8_t IsPhysicalGateEnabled();
extern void SwitchEnvelopeGenerationMode();

/*
 * These guys are tasks run in the main loop.
 * Put the pointer to a static ADC result (*uint16_t) as the argument.
 */
extern void SetAttackTime(void *arg);
extern void SetDecay0Time(void *arg);
extern void SetSustain0Level(void *arg);
extern void SetDecayTime(void *arg);
extern void SetSustainLevel(void *arg);
extern void SetReleaseTime(void *arg);
extern void CheckGate1(void *arg);
extern void CheckGate2(void *arg);

#ifdef __cplusplus
}
#endif


#endif /* INC_ENVELOPE_GENERATOR_H_ */
