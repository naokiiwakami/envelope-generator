/*
 * storage.h
 *
 *  Created on: Jul 30, 2025
 *      Author: naoki
 */

#ifndef INC_STORAGE_H_
#define INC_STORAGE_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// The last two pages of the flash memory are used for data storage
#define STORAGE_PAGE_0 ((uint32_t)(FLASH_PAGE_NB - 2))
#define STORAGE_PAGE_1 ((uint32_t)(FLASH_PAGE_NB - 1))

#define STORAGE_PAGE_0_ADDR ((uint32_t)(STORAGE_PAGE_0 * FLASH_PAGE_SIZE + FLASH_BASE))
#define STORAGE_PAGE_1_ADDR ((uint32_t)(STORAGE_PAGE_1 * FLASH_PAGE_SIZE + FLASH_BASE))

/* Exported functions prototypes ---------------------------------------------*/
extern void InitializeStorage();

extern void Save64(uint16_t address, uint64_t data);
extern uint64_t Load64(uint16_t address);

extern void Save32(uint16_t address, uint32_t data);
extern uint32_t Load32(uint16_t address);

extern void Save16(uint16_t address, uint16_t data);
extern uint16_t Load16(uint16_t address);

extern void Save8(uint16_t address, uint8_t data);
extern uint8_t Load8(uint16_t address);

extern void SaveString(uint16_t address, const char *data, size_t max_length);
extern void LoadString(uint16_t address, char *string, size_t max_length);

#ifdef __cplusplus
}
#endif

#endif /* INC_STORAGE_H_ */
