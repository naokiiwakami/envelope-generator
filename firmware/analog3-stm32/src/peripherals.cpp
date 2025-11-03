/*
 * peripherals.cpp
 *
 *  Created on: Jul 26, 2025
 *      Author: naoki
 */

#include "analog3/stm32impl.h"

#include <string.h>

#include <algorithm>

void put_string(const char* message)
{
  if (HAL_UART_Transmit(&huart1, (const uint8_t *)message, strlen(message), 1000) != HAL_OK) {
    Error_Handler();
  }
}

void put_hex(const void *data, size_t size)
{
    char message[17];
    message[0] = ' ';
    for (size_t i = 0; i < size; ++i) {
      uint8_t value = ((const uint8_t *)data)[size - i - 1];
      char elem = value >> 4;
      elem = elem < 10 ? '0' + elem : 'a' + elem - 10;
      message[2 * i + 1] = elem;
      elem = value & 0xf;
      elem = elem < 10 ? '0' + elem : 'a' + elem - 10;
      message[2 * i + 2] = elem;
    }
    if (HAL_UART_Transmit(&huart1, (const uint8_t *)message, size * 2 + 1, 1000) != HAL_OK) {
      Error_Handler();
    }
}

void adc_change_channel(uint32_t adc_channel)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = adc_channel;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }
}

uint32_t adc_run()
{
  HAL_ADC_Start(&hadc1);
  while (HAL_ADC_PollForConversion(&hadc1, 1000) != HAL_OK) {}
  uint32_t value = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);
  return value;
}

// MCP47x6 DAC handling

#define MCP47X6_COMMAND_WRITE_VOLATILE 0b010

/**
 * Initializes MCP47x6 DAC via hi2c1 HAL I2C handle.
 */
void InitializeMcp47x6Dac(uint16_t index, enum MCP47X6_VRL vrl, enum MCP47X6_POWER_DOWN pd,
                          enum MCP47X6_GAIN gain)
{
  uint8_t data[3];
  uint8_t command = 0b010; // write volatile memory
  data[0] = (command << 5) | (vrl << 3) | (pd << 1) | gain;
  data[1] = 0;
  data[2] = 0;

  uint16_t address = 0b1100000 + index;  // MCP47x6An, n = index
  if (HAL_I2C_Master_Transmit(&hi2c1, address << 1, data, 3, 1000) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * Updates MCP47x6 DAC via hi2c1 HAL I2C handle.
 */
void UpdateMcp47x6Dac(uint16_t index, uint16_t value)
{
  uint16_t address = 0b1100000 + index; // MCP47x6An, n = index
  uint8_t data[2];
  data[0] = value >> 12;
  data[1] = (value >> 4) & 0xff;
  if (HAL_I2C_Master_Transmit(&hi2c1, address << 1, data, 2, 1000) != HAL_OK) {
    Error_Handler();
  }
}
