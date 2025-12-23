#ifndef UART_LIB_H
#define UART_LIB_H

#include "stm32f4xx.h"
#include <stdint.h>

#define APB2_CLOCK 100000000 /* 100 MHz */
#define UART_BAUD_RATE 115200

/* Function Prototypes */
void MX_USART1_Init_RegisterLevel(void);
void UART_SendChar(char c);
void UART_SendString(const char *str);
void UART_SendInt(int32_t value);
void UART_SendFloat(float value, uint8_t decimals);

#endif /* UART_LIB_H */
