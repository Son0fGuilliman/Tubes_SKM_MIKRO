#include "system_config.h"
#include "stm32f4xx.h" /* CMSIS Header */

/* ============================================================================
 * SYSTEM CLOCK CONFIG (REGISTER LEVEL - NO HAL)
 * Target: 100 MHz (HSI 16MHz -> PLL)
 * ============================================================================
 */
void SystemClock_Config(void) {
  /* 1. Enable HSI and wait for it to be ready */
  RCC->CR |= RCC_CR_HSION;
  while (!(RCC->CR & RCC_CR_HSIRDY))
    ;

  /* 2. Configure Power Budget */
  RCC->APB1ENR |= RCC_APB1ENR_PWREN;
  PWR->CR |= PWR_CR_VOS; /* Scale 1 (allows up to 100 MHz) */

  /* 3. Configure Flash Prefetch, Instruction Cache, Data Cache and Wait States
   */
  /* For 100 MHz at 3.3V, we need 3 Wait States (LATENCY_3) */
  FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN |
               FLASH_ACR_LATENCY_3WS;

  /* 4. Configure PLL */
  /* Disable PLL first */
  RCC->CR &= ~RCC_CR_PLLON;
  while (RCC->CR & RCC_CR_PLLRDY)
    ;

/* Formula: PLL_VCO = (HSE_OR_HSI / PLLM) * PLLN
 *          SYSCLK  = PLL_VCO / PLLP
 *          HSI = 16 MHz
 *          Target = 100 MHz
 *          PLLM = 16 (16MHz / 16 = 1MHz) -> Input to PLL must be 1-2MHz
 * (ideally 1MHz to avoid jitter) PLLN = 200 (1MHz * 200 = 200MHz) -> VCO output
 * (must be 100-432MHz) PLLP = 2   (200MHz / 2   = 100MHz) -> SYSCLK PLLQ = 4
 * (200MHz / 4   = 50MHz)  -> USB/SDIO/RNG (48MHz usually required, but we don't
 * use them)
 */
#define PLL_M 16
#define PLL_N 200
#define PLL_P 2 /* 00: PLLP=2, 01: PLLP=4, 10: PLLP=6, 11: PLLP=8 */
#define PLL_Q 4

  RCC->PLLCFGR = (PLL_M << 0) | (PLL_N << 6) | (((PLL_P >> 1) - 1) << 16) |
                 (RCC_PLLCFGR_PLLSRC_HSI) | (PLL_Q << 24);

  /* 5. Enable PLL and wait */
  RCC->CR |= RCC_CR_PLLON;
  while (!(RCC->CR & RCC_CR_PLLRDY))
    ;

  /* 6. Configure AHB/APB Prescalers */
  /* AHB  = SYSCLK / 1 = 100 MHz */
  /* APB1 = SYSCLK / 2 = 50 MHz  (Max 50MHz) */
  /* APB2 = SYSCLK / 1 = 100 MHz (Max 100MHz) */
  RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
  RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
  RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;

  /* 7. Switch System Clock to PLL */
  RCC->CFGR &= ~RCC_CFGR_SW;
  RCC->CFGR |= RCC_CFGR_SW_PLL;

  /* 8. Wait for Switch Status */
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
    ;

  /* System Core Clock Update (Optional, defines global variable) */
  SystemCoreClock = 100000000;
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
 * GPIO INIT - REGISTER LEVEL
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
 * TIM2 TIMER INTERRUPT INIT - REGISTER LEVEL
 * ============================================================================
 */
void MX_TIM2_Interrupt_Init_RegisterLevel(void) {
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

  TIM2->CR1 = 0;
  TIM2->CR2 = 0;
  TIM2->PSC = 9999; /* 50MHz / 10000 = 5kHz */
  TIM2->ARR = 4999; /* 5kHz / 5000 = 1Hz */
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

  float lower = setpoint - TEMP_LED_MARGIN;
  float upper = setpoint + TEMP_LED_MARGIN;

  if (temp < lower)
    GPIOB->BSRR = GPIO_PIN_0; /* Hijau */
  else if (temp > upper)
    GPIOB->BSRR = GPIO_PIN_2; /* Merah */
  else
    GPIOB->BSRR = GPIO_PIN_1; /* Kuning */
}
