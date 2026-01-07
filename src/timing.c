#include "timing.h"

volatile uint32_t systick_counter = 0;

/* ============================================================================
 * TIMING FUNCTIONS 
 * ============================================================================
 */

void DWT_Init(void) {
  /* Enable TRC (Trace) - Register: CoreDebug->DEMCR */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  /* Reset cycle counter - Register: DWT->CYCCNT */
  DWT->CYCCNT = 0;

  /* Enable cycle counter - Register: DWT->CTRL bit 0 */
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void SysTick_Init_RegisterLevel(void) {
  /* Disable SysTick first */
  SysTick->CTRL = 0;

  /* Set reload value for 1ms interrupt */
  /* LOAD = (SystemCoreClock / 1000) - 1 */
  SysTick->LOAD = (SystemCoreClock / 1000) - 1;

  /* Clear current value */
  SysTick->VAL = 0;

  /* Set priority (optional) */
  NVIC_SetPriority(SysTick_IRQn, 0);

  /* Enable SysTick with:
   * Bit 2: CLKSOURCE = 1 (Processor clock / AHB)
   * Bit 1: TICKINT = 1 (Enable interrupt)
   * Bit 0: ENABLE = 1 (Enable counter)
   */
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk |
                  SysTick_CTRL_ENABLE_Msk;
}

void delay_us(uint32_t us) {
  uint32_t startTick = DWT->CYCCNT;
  uint32_t delayTicks = us * (SystemCoreClock / 1000000);

  while ((DWT->CYCCNT - startTick) < delayTicks)
    ;
}

void delay_ms(uint32_t ms) {
  uint32_t start = systick_counter;
  while ((systick_counter - start) < ms)
    ;
}

uint32_t GetTick(void) { return systick_counter; }

/* SysTick Handler - Increment tick counter */
void SysTick_Handler(void) { systick_counter++; }
