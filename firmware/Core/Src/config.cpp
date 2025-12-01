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
#include "analog3/property.h"
#include "analog3/stm32impl.h"
#include "addresses.h"
#include "envelope_generator.h"

const uint8_t num_voices = 2;

static uint16_t *voice_ids_data[num_voices] = {&analog3::eg_params_1.voice_id, &analog3::eg_params_2.voice_id};
static analog3::A3VectorP<uint16_t> voice_ids{num_voices, voice_ids_data};

static uint16_t *attack_time_data[num_voices] = {&analog3::eg_params_1.attack_time_param, &analog3::eg_params_2.attack_time_param};
static analog3::A3VectorP<uint16_t> attack_time{num_voices, attack_time_data};

// Implementation of required callback functions

const char *GetDefaultModuleName() {
  return "Humps";
}

uint16_t GetModuleType() {
  return MODULE_TYPE_HUMPS;
}

void SetModuleSpecificProperties(std::vector<analog3::Property> *props) {
  AddReadOnlyProperty(props, PROP_NUM_VOICES, A3_U8, &num_voices);
  AddProperty(props, PROP_VOICE_ID, A3_VECTOR_U16P, &voice_ids,  ADDR_VOICE_ID);
  AddProperty(props, PROP_ATTACK_TIME, A3_VECTOR_U16P, &attack_time,  ADDR_UNSET);
}
