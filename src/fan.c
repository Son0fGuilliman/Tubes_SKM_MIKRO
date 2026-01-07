#include "fan.h"

volatile uint8_t fan_speed_percent = 0;

/* ============================================================================
 * TIM1 PWM INIT - REGISTER LEVEL
 * ============================================================================
 */
void MX_TIM1_PWM_Init_RegisterLevel(void) {
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

  /* PA8: TIM1_CH1 (AF1) */
  GPIOA->MODER &= ~GPIO_MODER_MODER8;
  GPIOA->MODER |= GPIO_MODER_MODER8_1;
  GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL8;
  GPIOA->AFR[1] |= (1 << GPIO_AFRH_AFSEL8_Pos);

  TIM1->CR1 = 0;
  TIM1->CR2 = 0;
  /* Default clock (HSI 16MHz, no PLL): 16MHz / (160 * 1000) = 100Hz */
  TIM1->PSC = 159;
  TIM1->ARR = PWM_PERIOD; /* 100Hz */
  TIM1->CCR1 = 0;

  TIM1->CCMR1 &= ~TIM_CCMR1_OC1M;
  TIM1->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1);
  TIM1->CCMR1 |= TIM_CCMR1_OC1PE;

  TIM1->CCER |= TIM_CCER_CC1E;
  TIM1->BDTR |= TIM_BDTR_MOE;
  TIM1->CR1 |= TIM_CR1_CEN;
}

/* ============================================================================
 * SET FAN SPEED - REGISTER LEVEL
 * ============================================================================
 */
void Set_Fan_Speed_RegisterLevel(uint8_t percent) {
  if (percent > 100)
    percent = 100;
  fan_speed_percent = percent;

  TIM1->CCR1 = (uint32_t)((percent * PWM_PERIOD) / 100);
}
