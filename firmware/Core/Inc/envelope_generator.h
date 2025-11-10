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

extern void InitializeEnvelopeGenerator();
extern void NudgeEnvelopeGenerator();
extern void UpdateEnvelopeGenerator();

extern void SetAttackTime(void *arg);
extern void SetDecay0Time(void *arg);
extern void SetSustain0Level(void *arg);
extern void SetDecayTime(void *arg);
extern void SetSustainLevel(void *arg);
extern void SetReleaseTime(void *arg);

#ifdef __cplusplus
}
#endif


#endif /* INC_ENVELOPE_GENERATOR_H_ */
