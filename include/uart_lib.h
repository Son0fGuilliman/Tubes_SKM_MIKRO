#ifndef UART_LIB_H
#define UART_LIB_H

#include "stm32f4xx.h"
#include <stdint.h>

#define APB2_CLOCK 16000000u /* Default HSI 16 MHz */
#define UART_BAUD_RATE 9600u

/* Function Prototypes */
void MX_USART1_Init_RegisterLevel(void);
void UART_SendChar(char c);
void UART_SendString(const char *str);
void UART_SendInt(int32_t value);
void UART_SendFloat(float value, uint8_t decimals);

#endif /* UART_LIB_H */
