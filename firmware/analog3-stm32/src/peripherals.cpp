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

static HAL_StatusTypeDef I2C_WaitOnFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Flag, FlagStatus Status)
{
  while (__HAL_I2C_GET_FLAG(hi2c, Flag) == Status)
  {
  }
  return HAL_OK;
}

static HAL_StatusTypeDef I2C_WaitOnTXISFlagUntilTimeout(I2C_HandleTypeDef *hi2c)
{
  while (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_TXIS) == RESET)
  {
  }
  return HAL_OK;
}

static HAL_StatusTypeDef I2C_WaitOnSTOPFlagUntilTimeout(I2C_HandleTypeDef *hi2c)
{
  while (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_STOPF) == RESET)
  {
  }
  return HAL_OK;
}

static void I2C_TransferConfig(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t Size, uint32_t Mode,
                               uint32_t Request)
{
  uint32_t tmp;

  /* Check the parameters */
  assert_param(IS_I2C_ALL_INSTANCE(hi2c->Instance));
  assert_param(IS_TRANSFER_MODE(Mode));
  assert_param(IS_TRANSFER_REQUEST(Request));

  /* Declaration of tmp to prevent undefined behavior of volatile usage */
  tmp = ((uint32_t)(((uint32_t)DevAddress & I2C_CR2_SADD) | \
                    (((uint32_t)Size << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES) | \
                    (uint32_t)Mode | (uint32_t)Request) & (~0x80000000U));

  /* update CR2 register */
  MODIFY_REG(hi2c->Instance->CR2, \
             ((I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_AUTOEND | \
               (I2C_CR2_RD_WRN & (uint32_t)(Request >> (31U - I2C_CR2_RD_WRN_Pos))) | \
               I2C_CR2_START | I2C_CR2_STOP)), tmp);
}

static HAL_StatusTypeDef My_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData,
                                                uint16_t Size, uint32_t Timeout)
{
  uint32_t xfermode;

  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hi2c);

    if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BUSY, SET) != HAL_OK)
    {
      return HAL_ERROR;
    }

    hi2c->State     = HAL_I2C_STATE_BUSY_TX;
    hi2c->Mode      = HAL_I2C_MODE_MASTER;
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr  = pData;
    hi2c->XferCount = Size;
    hi2c->XferISR   = NULL;

    hi2c->XferSize = hi2c->XferCount;
    xfermode = I2C_AUTOEND_MODE;

    if (hi2c->XferSize > 0U)
    {
      /* Preload TX register */
      /* Write data to TXDR */
      hi2c->Instance->TXDR = *hi2c->pBuffPtr;

      /* Increment Buffer pointer */
      hi2c->pBuffPtr++;

      hi2c->XferCount--;
      hi2c->XferSize--;

      /* Send Slave Address */
      /* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE and generate RESTART */
      I2C_TransferConfig(hi2c, DevAddress, (uint8_t)(hi2c->XferSize + 1U), xfermode,
                         I2C_GENERATE_START_WRITE);
    }
    else
    {
      /* Send Slave Address */
      /* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE and generate RESTART */
      I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, xfermode,
                         I2C_GENERATE_START_WRITE);
    }

    HAL_GPIO_TogglePin(DEBUG_OUT_GPIO_Port, DEBUG_OUT_Pin);
    while (hi2c->XferCount > 0U)
    {
      /* Wait until TXIS flag is set */
      if (I2C_WaitOnTXISFlagUntilTimeout(hi2c) != HAL_OK)
      {
        return HAL_ERROR;
      }
      HAL_GPIO_TogglePin(DEBUG_OUT_GPIO_Port, DEBUG_OUT_Pin);
      /* Write data to TXDR */
      hi2c->Instance->TXDR = *hi2c->pBuffPtr;

      /* Increment Buffer pointer */
      hi2c->pBuffPtr++;

      hi2c->XferCount--;
      hi2c->XferSize--;

      if ((hi2c->XferCount != 0U) && (hi2c->XferSize == 0U))
      {
        /* Wait until TCR flag is set */
        if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_TCR, RESET) != HAL_OK)
        {
          return HAL_ERROR;
        }
        HAL_GPIO_TogglePin(DEBUG_OUT_GPIO_Port, DEBUG_OUT_Pin);

        hi2c->XferSize = hi2c->XferCount;
        I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_AUTOEND_MODE,
                           I2C_NO_STARTSTOP);
      }
    }
    HAL_GPIO_TogglePin(DEBUG_OUT_GPIO_Port, DEBUG_OUT_Pin);

    /* No need to Check TC flag, with AUTOEND mode the stop is automatically generated */
    /* Wait until STOPF flag is set */
    if (I2C_WaitOnSTOPFlagUntilTimeout(hi2c) != HAL_OK)
    {
      return HAL_ERROR;
    }
    HAL_GPIO_TogglePin(DEBUG_OUT_GPIO_Port, DEBUG_OUT_Pin);

    /* Clear STOP Flag */
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

    /* Clear Configuration Register 2 */
    I2C_RESET_CR2(hi2c);

    hi2c->State = HAL_I2C_STATE_READY;
    hi2c->Mode  = HAL_I2C_MODE_NONE;

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
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
  auto result = HAL_I2C_Master_Transmit(&hi2c1, address << 1, data, 2, 1000);
  if (result != HAL_OK && result != HAL_BUSY) {
    Error_Handler();
  }
}
