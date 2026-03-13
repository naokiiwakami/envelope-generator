/*
 * config-common.h
 *
 *  Created on: Aug 19, 2025
 *      Author: naoki
 */

#ifndef INCLUDE_ANALOG3_CONFIG_COMMON_H_
#define INCLUDE_ANALOG3_CONFIG_COMMON_H_

#include <vector>

#include "analog3/property.h"

#ifdef __cplusplus
extern "C" {
#endif

// Callback functions that should be implemented in the module-specific config.cpp /////////////
extern const char *GetDefaultModuleName();
extern uint16_t GetModuleType();
extern void SetModuleSpecificProperties(std::vector<analog3::Property> *props);

// Find implementations in stm32impl.h
extern void AddReadOnlyProperty(std::vector<analog3::Property> *props, uint8_t id, uint8_t value_type, const void *data);
extern void AddProperty(std::vector<analog3::Property> *props, uint8_t id, uint8_t vlue_type, void *data, uint16_t save_addr);

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_ANALOG3_CONFIG_COMMON_H_ */
