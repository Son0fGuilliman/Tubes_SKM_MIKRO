#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include "stm32f4xx.h"
#include <stdint.h>

#define TEMP_LED_MARGIN 5.0f /* ±5°C from setpoint */

/* Function Prototypes */
void SystemClock_Config(void);
void Error_Handler(void);
void MX_GPIO_Init_RegisterLevel(void);
void MX_TIM2_Interrupt_Init_RegisterLevel(void);
void Update_LEDs_Temperature(float temp, float setpoint);

#endif /* SYSTEM_CONFIG_H */
