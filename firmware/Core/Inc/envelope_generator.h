/*
 * envelope_generator.h
 *
 *  Created on: Nov 3, 2025
 *      Author: naoki
 */

#ifndef INC_ENVELOPE_GENERATOR_H_
#define INC_ENVELOPE_GENERATOR_H_

#ifdef __cplusplus
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
extern uint8_t IsPhysicalGateInputEnabled();

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
