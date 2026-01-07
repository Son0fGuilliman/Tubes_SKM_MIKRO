#include "system_config.h"
#include "stm32f4xx.h" /* CMSIS Header */

/* ============================================================================
 * SYSTEM CLOCK CONFIG (MINIMAL)
 * Use default HSI 16MHz, no PLL.
 * ============================================================================
 */
void SystemClock_Config(void) {
  RCC->CR |= RCC_CR_HSION;
  while (!(RCC->CR & RCC_CR_HSIRDY))
    ;

  /* Ensure PLL off (default) */
  RCC->CR &= ~RCC_CR_PLLON;
  while (RCC->CR & RCC_CR_PLLRDY)
    ;

  /* Default prescalers (AHB/APB1/APB2 all /1) and select HSI */
  RCC->CFGR = 0;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI)
    ;

  SystemCoreClock = 16000000u;
}

void Error_Handler(void) {
  __disable_irq();
  while (1) {
    GPIOB->ODR ^= GPIO_PIN_2;
    for (volatile int i = 0; i < 500000; i++)
      ;
  }
}

/* ============================================================================
 * GPIO INIT 
 * ============================================================================
 */
void MX_GPIO_Init_RegisterLevel(void) {
  /* Enable Clock */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

  /* PA0: DS18B20 (Open-Drain Output) */
  GPIOA->MODER &= ~GPIO_MODER_MODER0;
  GPIOA->MODER |= GPIO_MODER_MODER0_0;
  GPIOA->OTYPER |= GPIO_OTYPER_OT0;
  GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED0;
  GPIOA->BSRR = GPIO_PIN_0;

  /* PB0, PB1, PB2: LED (Push-Pull Output) */
  GPIOB->MODER &= ~GPIO_MODER_MODER0;
  GPIOB->MODER |= GPIO_MODER_MODER0_0;
  GPIOB->OTYPER &= ~GPIO_OTYPER_OT0;

  GPIOB->MODER &= ~GPIO_MODER_MODER1;
  GPIOB->MODER |= GPIO_MODER_MODER1_0;
  GPIOB->OTYPER &= ~GPIO_OTYPER_OT1;

  GPIOB->MODER &= ~GPIO_MODER_MODER2;
  GPIOB->MODER |= GPIO_MODER_MODER2_0;
  GPIOB->OTYPER &= ~GPIO_OTYPER_OT2;

  GPIOB->BSRR = (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2) << 16;

  /* PB4, PB5: Buttons (Input Pull-up + EXTI) */
  GPIOB->MODER &= ~GPIO_MODER_MODER4;
  GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD4;
  GPIOB->PUPDR |= GPIO_PUPDR_PUPD4_0;

  GPIOB->MODER &= ~GPIO_MODER_MODER5;
  GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD5;
  GPIOB->PUPDR |= GPIO_PUPDR_PUPD5_0;

  /* EXTI Configuration */
  SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR2_EXTI4;
  SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PB;

  SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR2_EXTI5;
  SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI5_PB;

  EXTI->IMR |= EXTI_IMR_IM4 | EXTI_IMR_IM5;
  EXTI->FTSR |= EXTI_FTSR_TR4 | EXTI_FTSR_TR5;

  NVIC_SetPriority(EXTI4_IRQn, 2);
  NVIC->ISER[0] |= (1 << EXTI4_IRQn);

  NVIC_SetPriority(EXTI9_5_IRQn, 2);
  NVIC->ISER[0] |= (1 << EXTI9_5_IRQn);
}

/* ============================================================================
 * TIM2 TIMER INTERRUPT INIT 
 * ============================================================================
 */
void MX_TIM2_Interrupt_Init_RegisterLevel(void) {
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

  TIM2->CR1 = 0;
  TIM2->CR2 = 0;
  TIM2->PSC = 15999; /* 16MHz / 16000 = 1kHz */
  TIM2->ARR = 999;   /* 1kHz / 1000 = 1Hz */
  TIM2->CNT = 0;

  TIM2->DIER |= TIM_DIER_UIE;
  TIM2->SR &= ~TIM_SR_UIF;

  NVIC_SetPriority(TIM2_IRQn, 1);
  NVIC->ISER[0] |= (1 << TIM2_IRQn);

  TIM2->CR1 |= TIM_CR1_CEN;
}

/* ============================================================================
 * UPDATE LEDs - REGISTER LEVEL
 * ============================================================================
 */
void Update_LEDs_Temperature(float temp, float setpoint) {
  GPIOB->BSRR = (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2) << 16;

  float upper = setpoint + TEMP_LED_MARGIN;

  if (temp <= setpoint)
    GPIOB->BSRR = GPIO_PIN_0; /* Hijau (Safe / Fan Off) */
  else if (temp <= upper)
    GPIOB->BSRR = GPIO_PIN_1; /* Kuning (Controlling / Fan Ramping) */
  else
    GPIOB->BSRR = GPIO_PIN_2; /* Merah (Hot / Fan Max) */
}
