#ifndef TIMING_H
#define TIMING_H

#include "stm32f4xx.h"
#include <stdint.h>

/* Global Tick Counter */
extern volatile uint32_t systick_counter;

/* Function Prototypes */
void DWT_Init(void);
void SysTick_Init_RegisterLevel(void);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
uint32_t GetTick(void);

#endif /* TIMING_H */
