#ifndef FAN_H
#define FAN_H

#include "stm32f4xx.h"
#include <stdint.h>

#define PWM_PERIOD 999

/* Global Fan State */
extern volatile uint8_t fan_speed_percent;

/* Function Prototypes */
void MX_TIM1_PWM_Init_RegisterLevel(void);
void Set_Fan_Speed_RegisterLevel(uint8_t percent);

#endif /* FAN_H */
