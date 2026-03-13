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

void InitializeSwitchState(switch_state_t *state,
                           const GPIO_TypeDef *gpiox,
                           uint16_t gpio_pin,
                           void (*handle_switch_pressed)(const switch_state_t *))
{
  state->current_status = 0;
  state->prev_status = 0;
  state->debouncing = 0;
  state->change_time = 0;
  state->gpiox = gpiox;
  state->gpio_pin = gpio_pin;
  state->handle_switch_pressed = handle_switch_pressed;
}

void CheckSwitch(switch_state_t *state)
{
  uint32_t now = HAL_GetTick();
  if (state->debouncing) {
    if (now >= state->change_time + 20) {
      state->debouncing = 0;
    }
    return;
  }
  state->current_status = HAL_GPIO_ReadPin(state->gpiox, state->gpio_pin) == 0;
  if (state->current_status != state->prev_status) {
    state->debouncing = 1;
    state->change_time = now;
  }
  if (state->current_status) {
    state->handle_switch_pressed(state);
  }
  state->prev_status = state->current_status;
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
