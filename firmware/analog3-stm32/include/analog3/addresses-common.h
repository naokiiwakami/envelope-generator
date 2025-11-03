/*
 * addresses.h
 *
 *  Created on: Jul 26, 2025
 *      Author: naoki
 */

/**
 * Defines FLASH addresses for parameter storages.
 *
 * The addresses are in the page 127 (the last) of STM32C92xx main flash memory.
 */

#ifndef ANALOG3_ADDRESSES_COMMON_H_
#define ANALOG3_ADDRESSES_COMMON_H_

// First 8 bytes are reserved for calibration

#define ADDR_MODULE_UID 0x8 // u32
#define ADDR_MODULE_NAME 0x10 // up to 64 chars
#define ADDR_PROFILE 0x50

#define ADDR_UNSET 0xffff

#endif /* ANALOG3_ADDRESSES_COMMON_H_ */
