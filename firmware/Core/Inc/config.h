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
#define PROP_CURRENT_PROFILE 3
#define PROP_MAX_PROFILES 4
#define PROP_NUM_VOICES 5
#define PROP_CV_DEPTH 6
#define PROP_CV_OFFSET 7
#define NUM_PROPS 8

#ifdef __cplusplus
extern "C" {
#endif

extern uint8_t current_profile;
extern const uint8_t max_profiles;
extern const uint8_t num_voices;

extern uint16_t cv_depth;
extern uint16_t cv_offset;

#ifdef __cplusplus
}
#endif

#endif /* INC_CONFIG_H_ */
