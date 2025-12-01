/*
 * config.h
 *
 *  Created on: Jul 28, 2025
 *      Author: naoki
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#include "analog3/analog3.h"

// Property IDs
#define PROP_NUM_VOICES 3
#define PROP_VOICE_ID 4
#define PROP_PHYSICAL_GATE_ENABLED 5
#define PROP_ENVELOPE_GENERATION_MODE 6
#define PROP_ATTACK_TIME 7
#define PROP_DECAY_TIME 8
#define PROP_SUSTAIN_LEVEL 9
#define PROP_RELEASE_TIME 10
#define PROP_EXTRA_1 11
#define PROP_EXTRA_2 12
#define NUM_PROPS 13

#ifdef __cplusplus
extern "C" {
#endif

extern const uint8_t num_voices;

#ifdef __cplusplus
}
#endif

#endif /* INC_CONFIG_H_ */
