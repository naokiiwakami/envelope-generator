/*
 * analog3/stm32impl.h
 *
 *  Created on: Jul 26, 2025
 *      Author: naoki
 */

#ifndef INCLUDE_ANALOG3_STM32IMPL_H_
#define INCLUDE_ANALOG3_STM32IMPL_H_

#include "main.h"

#include "analog3/definitions.h"

#ifdef DEBUG
extern char error_message[256];
#define __FILENAME__ (__builtin_strrchr(__FILE__, '/') ? __builtin_strrchr(__FILE__, '/') + 1 : __FILE__)
#define HandleError(format, ...) do {                                             \
      snprintf(error_message, sizeof(error_message),                              \
               "[%s:%d] " format "\r\n", __FILENAME__, __LINE__, ##__VA_ARGS__);  \
      Error_Handler();                                                            \
    } while (0)
#define DEBUG_PRINT(format, ...) do {                                             \
      char __debug_buf[128];                                                      \
      snprintf(__debug_buf, sizeof(__debug_buf),                                  \
               "[%s:%d] " format "\r\n", __FILENAME__, __LINE__, ##__VA_ARGS__);  \
      put_string(__debug_buf);                                                    \
    } while (0)
#else
#define HandleError(...) Error_Handler()
#define DEBUG_PRINT(format, ...)
#endif

/**
 * Set module specific properties. Implement this in the module firmware source area.
 */
#ifdef __cplusplus
extern "C" {
#endif

// button management
typedef struct SwitchState {
  uint8_t current_status;
  uint8_t prev_status;
  uint8_t debouncing;
  uint32_t change_time;
  const GPIO_TypeDef *gpiox;
  uint16_t gpio_pin;
  void (*handle_switch_pressed)(const struct SwitchState *);
} switch_state_t;
extern void InitializeSwitchState(switch_state_t *state,
                                  const GPIO_TypeDef *gpiox,
                                  uint16_t gpio_pin,
                                  void (*handle_switch_pressed)(const switch_state_t *state));
extern void CheckSwitch(switch_state_t *state);

// CAN message
#define MESSAGE_QUEUE_SIZE 16
typedef struct _stm32_can_message {
  FDCAN_RxHeaderTypeDef header;
  uint8_t data[CAN_STD_DATA_LENGTH];
} stm32_can_message_t;

extern stm32_can_message_t message_queue[MESSAGE_QUEUE_SIZE];
extern uint32_t q_head;
extern uint32_t q_tail;
extern uint8_t q_full;

// ADC
extern void adc_change_channel(uint32_t adc_channel);
extern uint32_t adc_run();

// MCP47x6 DAC
enum MCP47X6_VRL {  // Resistor Ladder Voltage Reference (Vrl) configuration
  MCP47X6_VRL_VDD = 0b00,      // VDD, unbuffered
  MCP47X6_VRL_VREF_UB = 0b10,  // Vref pin, unbuffered
  MCP47X6_VRL_VREF = 0b11,     // Vref pin, buffered
};

enum MCP47X6_POWER_DOWN {  // Power-Down configuration
  MCP47X6_PD_NORMAL = 0b00,  // Not powered down (nnormal operation).
  MCP47X6_PD_1K = 0b01,      // Powered down - Vout is loaded with 1kOhm R to GND.
  MCP47X6_PD_100K = 0b10,    // Powered down - Vout is loaded with 100kOhm R to GND.
  MCP47X6_PD_500K = 0b11,    // Powered down - Vout is loaded with 500kOhm R to GND.
};

enum MCP47X6_GAIN { // Gain configuration
  MCP47X6_GAIN_1X = 0,  // gain of 1
  MCP47X6_GAIN_2X = 1,  // gain of 2. N/A when Vdd is used as Vrl
};

/**
 * Initializes MCP47x6 DAC via hi2c1 HAL I2C handle.
 */
extern void InitializeMcp47x6Dac(uint16_t index, enum MCP47X6_VRL vrl, enum MCP47X6_POWER_DOWN pd,
                                 enum MCP47X6_GAIN gain);
/**
 * Updates MCP47x6 DAC via hi2c1 HAL I2C handle.
 */
extern void InitiateMcp47x6DacUpdate(uint16_t index, uint16_t value);
extern void CompleteMcp47x6DacUpdate();
extern void UpdateMcp47x6Dac(uint16_t index, uint16_t value);

// Serial
extern void put_string(const char* message);
extern void put_hex(const void *data, size_t size);

// Analog3 component
extern void InitializeAnalog3();
extern void SignIn();
extern void HandleCanRxMessage(const stm32_can_message_t *message);

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_ANALOG3_STM32IMPL_H_ */
