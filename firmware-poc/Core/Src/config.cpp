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

using ::analog3::EnvelopeGeneratorParams;

template<typename T, T EnvelopeGeneratorParams::*Param>
struct EgProp {
  T *data[num_voices];
  analog3::A3VectorP<T> vector;

  EgProp()
      :
      data { &(analog3::eg_params_1.*Param), &(analog3::eg_params_2.*Param) },
      vector { num_voices, data } {
  }
};

static EgProp<uint16_t, &EnvelopeGeneratorParams::voice_id> voice_ids;
static EgProp<bool, &EnvelopeGeneratorParams::physical_gate_enabled> physical_gate_enabled;
static EgProp<uint8_t, &EnvelopeGeneratorParams::mode> mode;
static EgProp<uint16_t, &EnvelopeGeneratorParams::attack_time_param> attack_time;
static EgProp<uint16_t, &EnvelopeGeneratorParams::decay_time_param> decay_time;
static EgProp<uint16_t, &EnvelopeGeneratorParams::sustain_level_param> sustain_level;
static EgProp<uint16_t, &EnvelopeGeneratorParams::release_time_param> release_time;
static EgProp<uint16_t, &EnvelopeGeneratorParams::decay0_time_param> extra_1;
static EgProp<uint16_t, &EnvelopeGeneratorParams::sustain0_level_param> extra_2;

// Following implements required callback functions ///////////////////////////////////////////

const char* GetDefaultModuleName() {
  return "Humps";
}

uint16_t GetModuleType() {
  return MODULE_TYPE_HUMPS;
}

void SetModuleSpecificProperties(std::vector<analog3::Property> *props) {
  AddReadOnlyProperty(props, PROP_NUM_VOICES, A3_U8, &num_voices);
  AddProperty(props, PROP_VOICE_ID, A3_VECTOR_U16P, &voice_ids.vector, ADDR_VOICE_ID);
  AddProperty(props, PROP_PHYSICAL_GATE_ENABLED, A3_VECTOR_U8P, &physical_gate_enabled.vector,
              ADDR_PHYSICAL_GATE_ENABLED);
  AddProperty(props, PROP_ENVELOPE_GENERATION_MODE, A3_VECTOR_U8P, &mode.vector,
              ADDR_ENVELOPE_GENERATION_MODE);
  AddProperty(props, PROP_ATTACK_TIME, A3_VECTOR_U16P, &attack_time.vector, ADDR_UNSET);
  AddProperty(props, PROP_DECAY_TIME, A3_VECTOR_U16P, &decay_time.vector, ADDR_UNSET);
  AddProperty(props, PROP_SUSTAIN_LEVEL, A3_VECTOR_U16P, &sustain_level.vector, ADDR_UNSET);
  AddProperty(props, PROP_RELEASE_TIME, A3_VECTOR_U16P, &release_time.vector, ADDR_UNSET);
  AddProperty(props, PROP_EXTRA_1, A3_VECTOR_U16P, &extra_1.vector, ADDR_UNSET);
  AddProperty(props, PROP_EXTRA_2, A3_VECTOR_U16P, &extra_2.vector, ADDR_UNSET);
}
