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
extern void UpdateEnvelopeGenerator();
extern void SetReleaseTime(uint16_t release_time);

#ifdef __cplusplus
}
#endif


#endif /* INC_ENVELOPE_GENERATOR_H_ */
