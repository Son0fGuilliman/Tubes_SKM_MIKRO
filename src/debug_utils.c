#include "debug_utils.h"
#include "stm32f4xx.h"
#include "uart_lib.h"
#include <stdio.h>

/* ============================================================================
 * PRINT HEADER
 * ============================================================================
 */
void Print_Control_Header(void) {
  UART_SendString("\r\n");
  UART_SendString("  TIME | T_act | Setpt | FAN\r\n");
  UART_SendString("-------|-------|-------|----\r\n");
}

/* ============================================================================
 * PRINT STATUS
 * ============================================================================
 */
void Print_Status(uint32_t time_sec, float temp, float setpoint,
                  int fan_speed) {
  /* Time */
  UART_SendString("  ");
  if (time_sec < 10)
    UART_SendString("   ");
  else if (time_sec < 100)
    UART_SendString("  ");
  else if (time_sec < 1000)
    UART_SendString(" ");
  UART_SendInt(time_sec);
  UART_SendString("s | ");

  /* T_actual */
  UART_SendFloat(temp, 1);
  UART_SendString(" | ");

  /* Setpoint */
  UART_SendFloat(setpoint, 1);
  UART_SendString(" | ");

  /* Fan % */
  if (fan_speed < 10)
    UART_SendString(" ");
  if (fan_speed < 100)
    UART_SendString(" ");
  UART_SendInt(fan_speed);
  UART_SendString("%");

  UART_SendString("\r\n");
}

/* ============================================================================
 * PRINT ALL REGISTER STATUS
 * ============================================================================
 */
void Print_All_Register_Status(void) {
  UART_SendString(
      "================================================================\r\n");
  UART_SendString("                    REGISTER STATUS\r\n");
  UART_SendString(
      "================================================================\r\n");
  UART_SendString("\r\n");

  /* ===== SYSTICK & DWT REGISTERS ===== */
  UART_SendString("[ TIMING - REGISTER LEVEL (NO HAL!) ]\r\n");
  UART_SendString("------------------------------------------------\r\n");
  UART_SendString("  SysTick->CTRL = 0x");
  uint32_t ctrl = SysTick->CTRL;
  for (int i = 7; i >= 0; i--) {
    uint8_t nibble = (ctrl >> (i * 4)) & 0xF;
    UART_SendChar(nibble < 10 ? '0' + nibble : 'A' + nibble - 10);
  }
  UART_SendString("\r\n");
  UART_SendString("    - ENABLE: ");
  UART_SendInt(SysTick->CTRL & 1);
  UART_SendString(", TICKINT: ");
  UART_SendInt((SysTick->CTRL >> 1) & 1);
  UART_SendString(", CLKSOURCE: ");
  UART_SendInt((SysTick->CTRL >> 2) & 1);
  UART_SendString("\r\n");
  UART_SendString("  SysTick->LOAD = ");
  UART_SendInt(SysTick->LOAD);
  UART_SendString(" (for 1ms @ ");
  UART_SendInt(SystemCoreClock / 1000000);
  UART_SendString("MHz)\r\n");
  UART_SendString("  DWT->CTRL     = 0x");
  uint32_t dwt = DWT->CTRL;
  for (int i = 7; i >= 0; i--) {
    uint8_t nibble = (dwt >> (i * 4)) & 0xF;
    UART_SendChar(nibble < 10 ? '0' + nibble : 'A' + nibble - 10);
  }
  UART_SendString(" (CYCCNTENA for us delay)\r\n\r\n");

  /* ===== 2.2 UART REGISTERS ===== */
  UART_SendString("[ 2.2 SERIAL COMMUNICATION - USART1 ]\r\n");
  UART_SendString("------------------------------------------------\r\n");

  UART_SendString("  USART1->BRR  = 0x");
  uint32_t brr = USART1->BRR;
  for (int i = 7; i >= 0; i--) {
    uint8_t nibble = (brr >> (i * 4)) & 0xF;
    UART_SendChar(nibble < 10 ? '0' + nibble : 'A' + nibble - 10);
  }
  UART_SendString(" (Baud Rate)\r\n");

  UART_SendString("    - Mantissa: ");
  UART_SendInt((USART1->BRR >> 4) & 0xFFF);
  UART_SendString(", Fraction: ");
  UART_SendInt(USART1->BRR & 0xF);
  UART_SendString("\r\n");

  /* NB: APB2_CLOCK is defined in uart_lib.h */
  UART_SendString("    - Calculated: ");
  uint32_t mantissa = (USART1->BRR >> 4) & 0xFFF;
  uint32_t fraction = USART1->BRR & 0xF;
  uint32_t divider = (mantissa << 4) + fraction;
  uint32_t calc_baud = APB2_CLOCK / divider;
  UART_SendInt(calc_baud);
  UART_SendString(" bps\r\n");

  UART_SendString("  USART1->CR1  = 0x");
  uint32_t cr1 = USART1->CR1;
  for (int i = 7; i >= 0; i--) {
    uint8_t nibble = (cr1 >> (i * 4)) & 0xF;
    UART_SendChar(nibble < 10 ? '0' + nibble : 'A' + nibble - 10);
  }
  UART_SendString("\r\n");
  UART_SendString("    - UE: ");
  UART_SendInt((USART1->CR1 >> 13) & 1);
  UART_SendString(", TE: ");
  UART_SendInt((USART1->CR1 >> 3) & 1);
  UART_SendString(", RE: ");
  UART_SendInt((USART1->CR1 >> 2) & 1);
  UART_SendString("\r\n\r\n");

  /* ===== 2.3 TIMER REGISTERS ===== */
  UART_SendString("[ 2.3 TIMER DAN COUNTER ]\r\n");
  UART_SendString("------------------------------------------------\r\n");

  UART_SendString("  TIM1 (PWM - 25kHz):\r\n");
  UART_SendString("    TIM1->CR1  = 0x");
  UART_SendInt(TIM1->CR1);
  UART_SendString(" (CEN=");
  UART_SendInt(TIM1->CR1 & 1);
  UART_SendString(")\r\n");
  UART_SendString("    TIM1->PSC  = ");
  UART_SendInt(TIM1->PSC);
  UART_SendString("\r\n");
  UART_SendString("    TIM1->ARR  = ");
  UART_SendInt(TIM1->ARR);
  UART_SendString("\r\n");
  UART_SendString("    TIM1->CCR1 = ");
  UART_SendInt(TIM1->CCR1);
  UART_SendString(" (Duty)\r\n");
  UART_SendString("    TIM1->BDTR = 0x");
  UART_SendInt(TIM1->BDTR);
  UART_SendString(" (MOE)\r\n\r\n");

  UART_SendString("  TIM2 (1Hz Interrupt):\r\n");
  UART_SendString("    TIM2->CR1  = 0x");
  UART_SendInt(TIM2->CR1);
  UART_SendString(" (CEN=");
  UART_SendInt(TIM2->CR1 & 1);
  UART_SendString(")\r\n");
  UART_SendString("    TIM2->PSC  = ");
  UART_SendInt(TIM2->PSC);
  UART_SendString("\r\n");
  UART_SendString("    TIM2->ARR  = ");
  UART_SendInt(TIM2->ARR);
  UART_SendString("\r\n");
  UART_SendString("    TIM2->DIER = 0x");
  UART_SendInt(TIM2->DIER);
  UART_SendString(" (UIE=");
  UART_SendInt(TIM2->DIER & 1);
  UART_SendString(")\r\n\r\n");

  /* ===== 2.4 INTERRUPT REGISTERS ===== */
  UART_SendString("[ 2.4 INTERRUPT SYSTEM ]\r\n");
  UART_SendString("------------------------------------------------\r\n");

  UART_SendString("  EXTI->IMR  = 0x");
  UART_SendInt(EXTI->IMR);
  UART_SendString(" (Line4=");
  UART_SendInt((EXTI->IMR >> 4) & 1);
  UART_SendString(" (Line5=");
  UART_SendInt((EXTI->IMR >> 5) & 1);
  UART_SendString(")\r\n");
  UART_SendString("  EXTI->FTSR = 0x");
  UART_SendInt(EXTI->FTSR);
  UART_SendString(" (Falling)\r\n");
  UART_SendString("  NVIC->ISER[0] = 0x");
  uint32_t iser = NVIC->ISER[0];
  for (int i = 7; i >= 0; i--) {
    uint8_t nibble = (iser >> (i * 4)) & 0xF;
    UART_SendChar(nibble < 10 ? '0' + nibble : 'A' + nibble - 10);
  }
  UART_SendString("\r\n\r\n");

  /* ===== 2.5 GPIO REGISTERS ===== */
  UART_SendString("[ 2.5 GPIO CONFIGURATION ]\r\n");
  UART_SendString("------------------------------------------------\r\n");

  UART_SendString("  GPIOA->MODER   = 0x");
  uint32_t moder = GPIOA->MODER;
  for (int i = 7; i >= 0; i--) {
    uint8_t nibble = (moder >> (i * 4)) & 0xF;
    UART_SendChar(nibble < 10 ? '0' + nibble : 'A' + nibble - 10);
  }
  UART_SendString("\r\n");
  UART_SendString("  GPIOA->OTYPER  = 0x");
  UART_SendInt(GPIOA->OTYPER);
  UART_SendString(" (PA0=");
  UART_SendString((GPIOA->OTYPER & 1) ? "OD" : "PP");
  UART_SendString(")\r\n");

  UART_SendString("  GPIOB->MODER   = 0x");
  moder = GPIOB->MODER;
  for (int i = 7; i >= 0; i--) {
    uint8_t nibble = (moder >> (i * 4)) & 0xF;
    UART_SendChar(nibble < 10 ? '0' + nibble : 'A' + nibble - 10);
  }
  UART_SendString("\r\n");
  UART_SendString("  GPIOB->PUPDR   = 0x");
  UART_SendInt(GPIOB->PUPDR);
  UART_SendString(" (PB4,5=PU)\r\n");

  UART_SendString("\r\n");
  UART_SendString(
      "================================================================\r\n");
  UART_SendString("\r\n");
}
