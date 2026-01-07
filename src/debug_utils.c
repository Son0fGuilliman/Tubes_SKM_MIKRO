#include "debug_utils.h"
#include "stm32f4xx.h"
#include "uart_lib.h"

static void UART_SendSpaces(uint8_t count) {
  while (count--) {
    UART_SendChar(' ');
  }
}

static void UART_SendHex32(uint32_t value) {
  for (int i = 7; i >= 0; i--) {
    uint8_t nibble = (value >> (i * 4)) & 0xFu;
    UART_SendChar((nibble < 10u) ? ('0' + nibble) : ('A' + nibble - 10u));
  }
}

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
  uint8_t time_pad = 0;
  if (time_sec < 1000u)
    time_pad++;
  if (time_sec < 100u)
    time_pad++;
  if (time_sec < 10u)
    time_pad++;
  UART_SendSpaces(time_pad);
  UART_SendInt(time_sec);
  UART_SendString("s | ");

  /* T_actual */
  UART_SendFloat(temp, 1);
  UART_SendString(" | ");

  /* Setpoint */
  UART_SendFloat(setpoint, 1);
  UART_SendString(" | ");

  /* Fan % */
  uint8_t fan_pad = 0;
  if (fan_speed < 100)
    fan_pad++;
  if (fan_speed < 10)
    fan_pad++;
  UART_SendSpaces(fan_pad);
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
  UART_SendHex32(ctrl);
  UART_SendString("\r\n");
  UART_SendString("    - ENABLE: ");
  UART_SendInt(ctrl & 1);
  UART_SendString(", TICKINT: ");
  UART_SendInt((ctrl >> 1) & 1);
  UART_SendString(", CLKSOURCE: ");
  UART_SendInt((ctrl >> 2) & 1);
  UART_SendString("\r\n");
  UART_SendString("  SysTick->LOAD = ");
  UART_SendInt(SysTick->LOAD);
  UART_SendString(" (for 1ms @ ");
  UART_SendInt(SystemCoreClock / 1000000);
  UART_SendString("MHz)\r\n");
  UART_SendString("  DWT->CTRL     = 0x");
  UART_SendHex32(DWT->CTRL);
  UART_SendString(" (CYCCNTENA for us delay)\r\n\r\n");

  /* ===== 2.2 UART REGISTERS ===== */
  UART_SendString("[ 2.2 SERIAL COMMUNICATION - USART1 ]\r\n");
  UART_SendString("------------------------------------------------\r\n");

  UART_SendString("  USART1->BRR  = 0x");
  uint32_t brr = USART1->BRR;
  UART_SendHex32(brr);
  UART_SendString(" (Baud Rate)\r\n");

  UART_SendString("    - Mantissa: ");
  uint32_t mantissa = (brr >> 4) & 0xFFF;
  uint32_t fraction = brr & 0xF;
  UART_SendInt(mantissa);
  UART_SendString(", Fraction: ");
  UART_SendInt(fraction);
  UART_SendString("\r\n");

  /* NB: APB2_CLOCK is defined in uart_lib.h */
  UART_SendString("    - Calculated: ");
  uint32_t calc_baud = APB2_CLOCK / brr;
  UART_SendInt(calc_baud);
  UART_SendString(" bps\r\n");

  UART_SendString("  USART1->CR1  = 0x");
  uint32_t cr1 = USART1->CR1;
  UART_SendHex32(cr1);
  UART_SendString("\r\n");
  UART_SendString("    - UE: ");
  UART_SendInt((cr1 >> 13) & 1);
  UART_SendString(", TE: ");
  UART_SendInt((cr1 >> 3) & 1);
  UART_SendString(", RE: ");
  UART_SendInt((cr1 >> 2) & 1);
  UART_SendString("\r\n\r\n");

  /* ===== 2.3 TIMER REGISTERS ===== */
  UART_SendString("[ 2.3 TIMER DAN COUNTER ]\r\n");
  UART_SendString("------------------------------------------------\r\n");

  UART_SendString("  TIM1 (PWM - 100Hz):\r\n");
  UART_SendString("    TIM1->CR1  = 0x");
  UART_SendHex32(TIM1->CR1);
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
  UART_SendHex32(TIM1->BDTR);
  UART_SendString(" (MOE)\r\n\r\n");

  UART_SendString("  TIM2 (1Hz Interrupt):\r\n");
  UART_SendString("    TIM2->CR1  = 0x");
  UART_SendHex32(TIM2->CR1);
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
  UART_SendHex32(TIM2->DIER);
  UART_SendString(" (UIE=");
  UART_SendInt(TIM2->DIER & 1);
  UART_SendString(")\r\n\r\n");

  /* ===== 2.4 INTERRUPT REGISTERS ===== */
  UART_SendString("[ 2.4 INTERRUPT SYSTEM ]\r\n");
  UART_SendString("------------------------------------------------\r\n");

  UART_SendString("  EXTI->IMR  = 0x");
  uint32_t exti_imr = EXTI->IMR;
  UART_SendHex32(exti_imr);
  UART_SendString(" (Line4=");
  UART_SendInt((exti_imr >> 4) & 1);
  UART_SendString(", Line5=");
  UART_SendInt((exti_imr >> 5) & 1);
  UART_SendString(")\r\n");
  UART_SendString("  EXTI->FTSR = 0x");
  UART_SendHex32(EXTI->FTSR);
  UART_SendString(" (Falling)\r\n");
  UART_SendString("  NVIC->ISER[0] = 0x");
  UART_SendHex32(NVIC->ISER[0]);
  UART_SendString("\r\n\r\n");

  /* ===== 2.5 GPIO REGISTERS ===== */
  UART_SendString("[ 2.5 GPIO CONFIGURATION ]\r\n");
  UART_SendString("------------------------------------------------\r\n");

  UART_SendString("  GPIOA->MODER   = 0x");
  uint32_t moder = GPIOA->MODER;
  UART_SendHex32(moder);
  UART_SendString("\r\n");
  UART_SendString("  GPIOA->OTYPER  = 0x");
  UART_SendHex32(GPIOA->OTYPER);
  UART_SendString(" (PA0=");
  UART_SendString((GPIOA->OTYPER & 1) ? "OD" : "PP");
  UART_SendString(")\r\n");

  UART_SendString("  GPIOB->MODER   = 0x");
  moder = GPIOB->MODER;
  UART_SendHex32(moder);
  UART_SendString("\r\n");
  UART_SendString("  GPIOB->PUPDR   = 0x");
  UART_SendHex32(GPIOB->PUPDR);
  UART_SendString(" (PB4,5=PU)\r\n");

  UART_SendString("\r\n");
  UART_SendString(
      "================================================================\r\n");
  UART_SendString("\r\n");
}
