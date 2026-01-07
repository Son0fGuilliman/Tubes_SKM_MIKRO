#include "uart_lib.h"

/* ============================================================================
 * USART1 INIT - REGISTER LEVEL
 * ============================================================================
 */
void MX_USART1_Init_RegisterLevel(void) {
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

  /* PA9: TX (AF7) */
  GPIOA->MODER &= ~GPIO_MODER_MODER9;
  GPIOA->MODER |= GPIO_MODER_MODER9_1;
  GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL9;
  GPIOA->AFR[1] |= (7 << GPIO_AFRH_AFSEL9_Pos);
  GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED9;
  GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD9;
  GPIOA->PUPDR |= GPIO_PUPDR_PUPD9_0;

  /* PA10: RX (AF7) */
  GPIOA->MODER &= ~GPIO_MODER_MODER10;
  GPIOA->MODER |= GPIO_MODER_MODER10_1;
  GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL10;
  GPIOA->AFR[1] |= (7 << GPIO_AFRH_AFSEL10_Pos);
  GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD10;
  GPIOA->PUPDR |= GPIO_PUPDR_PUPD10_0;

  USART1->CR1 = 0;

  /* Baud Rate Calculation */
  /* BRR = fCK / baud (oversampling 16) */
  USART1->BRR = (APB2_CLOCK + (UART_BAUD_RATE / 2u)) / UART_BAUD_RATE;

  USART1->CR2 = 0;
  USART1->CR3 = 0;
  USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

/* ============================================================================
 * UART FUNCTIONS
 * ============================================================================
 */

/* Send 1 character via UART (Register Level) */
void UART_SendChar(char c) {
  /* Wait until TXE (Transmit Data Register Empty) = 1 */
  while (!(USART1->SR & USART_SR_TXE))
    ;

  /* Write data to Data Register */
  USART1->DR = c;
}

/* Send string via UART */
void UART_SendString(const char *str) {
  while (*str) {
    UART_SendChar(*str++);
  }
}

/* Send integer via UART */
void UART_SendInt(int32_t value) {
  char buffer[12];
  int i = 0;
  uint8_t negative = 0;

  if (value < 0) {
    negative = 1;
    value = -value;
  }

  if (value == 0) {
    UART_SendChar('0');
    return;
  }

  while (value > 0) {
    buffer[i++] = '0' + (value % 10);
    value /= 10;
  }

  if (negative)
    UART_SendChar('-');

  while (i > 0) {
    UART_SendChar(buffer[--i]);
  }
}

/* Send float via UART */
void UART_SendFloat(float value, uint8_t decimals) {
  if (value < 0) {
    UART_SendChar('-');
    value = -value;
  }

  int32_t int_part = (int32_t)value;
  UART_SendInt(int_part);

  UART_SendChar('.');

  float frac = value - int_part;
  for (uint8_t i = 0; i < decimals; i++) {
    frac *= 10;
    UART_SendChar('0' + (int)frac % 10);
  }
}
