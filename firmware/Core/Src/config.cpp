/*
 * config.cpp
 *
 *  Created on: Jul 28, 2025
 *      Author: naoki
 *
 * This file implements callbacks from stm32impl w.r.t. module configuration.
 */

#include <analog3/addresses-common.h>
#include "config.h"

#include "analog3/config-common.h"
#include "analog3/definitions.h"
#include "analog3/stm32impl.h"

uint8_t current_profile = 0xff;
const uint8_t max_profiles = 8;
const uint8_t num_voices = 2;

uint16_t cv_depth = 0;
uint16_t cv_offset = 0x8000;

// Implementation of required callback functions

const char *GetDefaultModuleName() {
  return "Humps";
}

uint16_t GetModuleType() {
  return MODULE_TYPE_HUMPS;
}

void SetModuleSpecificProperties(std::vector<analog3::Property> *props) {
  AddProperty(props, PROP_CURRENT_PROFILE, A3_U8, &current_profile, ADDR_UNSET);
  AddReadOnlyProperty(props, PROP_MAX_PROFILES, A3_U8, &max_profiles);
  AddReadOnlyProperty(props, PROP_NUM_VOICES, A3_U8, &num_voices);
}
