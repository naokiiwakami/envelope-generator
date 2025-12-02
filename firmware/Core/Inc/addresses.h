/*
 * addresses.h
 *
 *  Created on: Dec 1, 2025
 *      Author: naoki
 */

#ifndef INC_ADDRESSES_H_
#define INC_ADDRESSES_H_

#define ADDR_VOICE_ID ADDR_MODULE_SPECIFIC_BASE
#define ADDR_PHYSICAL_GATE_ENABLED (ADDR_VOICE_ID + 4)
#define ADDR_ENVELOPE_GENERATION_MODE (ADDR_PHYSICAL_GATE_ENABLED + 2)

#endif /* INC_ADDRESSES_H_ */
