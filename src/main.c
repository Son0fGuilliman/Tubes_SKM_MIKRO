#include "stm32f4xx.h"

/* Modules */
#include "ds18b20.h"
#include "fan.h"
#include "system_config.h"
#include "timing.h"
#include "uart_lib.h"

/* ============================================================================
 * CONTROL CONFIGURATION (PROPORTIONAL CONTROL)
 * ============================================================================
 */
#define SETPOINT_DEFAULT 30.0f /* Default target temperature (°C) */
#define SETPOINT_MIN 20.0f     /* Minimum setpoint */
#define SETPOINT_MAX 50.0f     /* Maximum setpoint */

#define PID_KP 10.0f /* Proportional Gain */

#define OUTPUT_MAX 100.0f /* Maximum PWM output (%) */

#define DEBOUNCE_MS 200u

/* ============================================================================
 * GLOBAL VARIABLES
 * ============================================================================
 */

/* Control Variables */
static volatile float setpoint = SETPOINT_DEFAULT;

/* System State */
/* fan_speed_percent is extern in fan.h */
static volatile uint32_t last_button_time = 0;
static volatile uint8_t flag_read_temp = 0;
static volatile uint8_t setpoint_changed = 0;

static inline uint8_t Both_Buttons_Pressed(void) {
  return (GPIOB->IDR & (GPIO_PIN_4 | GPIO_PIN_5)) == 0;
}

static void UART_LogTelemetry(uint32_t time_sec, float temp, float setpoint,
                              uint8_t fan_percent) {
  UART_SendInt((int32_t)time_sec);
  UART_SendChar(',');
  UART_SendFloat(temp, 1);
  UART_SendChar(',');
  UART_SendFloat(setpoint, 1);
  UART_SendChar(',');
  UART_SendInt((int32_t)fan_percent);
  UART_SendString("\r\n");
}

/* ============================================================================
 * MAIN FUNCTION
 * ============================================================================
 */
int main(void) {
  /* FPU Settings (Explicitly Enable FPU) */
  /* CPACR is located at address 0xE000ED88 */
  SCB->CPACR |= ((3UL << 10 * 2) |
                 (3UL << 11 * 2)); /* Access rights for Coprocessor 10 and 11 */

  /* System Configuration */
  SystemClock_Config();

  /* Inisialisasi timing */
  DWT_Init();
  SysTick_Init_RegisterLevel();

  /* Inisialisasi Peripheral */
  MX_GPIO_Init_RegisterLevel();
  MX_USART1_Init_RegisterLevel();
  MX_TIM1_PWM_Init_RegisterLevel();
  MX_TIM2_Interrupt_Init_RegisterLevel();

  /* Set speed awal 0% */
  Set_Fan_Speed_RegisterLevel(0);

  /* Optional: one-time CSV header for UART logging */
  UART_SendString("t,temp,setpoint,fan\r\n");

  /* Test LED */
  GPIOB->BSRR = GPIO_PIN_0; /* LED Hijau ON */
  delay_ms(200);
  GPIOB->BSRR = GPIO_PIN_0 << 16; /* LED Hijau OFF */
  GPIOB->BSRR = GPIO_PIN_1;       /* LED Kuning ON */
  delay_ms(200);
  GPIOB->BSRR = GPIO_PIN_1 << 16;
  GPIOB->BSRR = GPIO_PIN_2; /* LED Merah ON */
  delay_ms(200);
  GPIOB->BSRR = GPIO_PIN_2 << 16;

  /* Check DS18B20 */
  if (!DS18B20_Reset()) {
    while (1) {
      GPIOB->ODR ^= GPIO_PIN_2;
      delay_ms(100);
    }
  }

  /* Baca suhu pertama kali */
  float current_temperature = DS18B20_ReadTemperature();
  uint32_t seconds_counter = 0;

  /* ==================== MAIN LOOP ==================== */
  while (1) {
    /* Cek jika setpoint berubah (dari interrupt) */
    if (setpoint_changed) {
      setpoint_changed = 0;
    }

    /* Cek flag dari Timer Interrupt (setiap 1 detik) */
    if (flag_read_temp) {
      flag_read_temp = 0;

      /* ========== 1. Baca suhu dari DS18B20 ========== */
      current_temperature = DS18B20_ReadTemperature();

      /* ========== 2. Hitung Output (Proportional Control) ========== */
      float current_setpoint = setpoint;
      float error = current_temperature - current_setpoint;
      float output = (error > 0.0f) ? (error * PID_KP) : 0.0f;

      /* Clamp Output */
      if (output > OUTPUT_MAX)
        output = OUTPUT_MAX;

      /* ========== 3. Set Fan Speed ========== */
      Set_Fan_Speed_RegisterLevel((uint8_t)output);

      /* ========== 4. UART telemetry log ========== */
      UART_LogTelemetry(seconds_counter, current_temperature, current_setpoint,
                        fan_speed_percent);

      seconds_counter++;
    }

    /* Update LED */
    Update_LEDs_Temperature(current_temperature, setpoint);

    delay_ms(10);
  }
}

/* ============================================================================
 * INTERRUPT HANDLERS
 * ============================================================================
 */

/* Timer 2 Interrupt */
void TIM2_IRQHandler(void) {
  if (TIM2->SR & TIM_SR_UIF) {
    TIM2->SR &= ~TIM_SR_UIF;
    flag_read_temp = 1;
  }
}

static inline void Handle_Setpoint_Button(uint32_t exti_pr_bit, float delta) {
  if (!(EXTI->PR & exti_pr_bit)) {
    return;
  }
  EXTI->PR = exti_pr_bit;

  if (Both_Buttons_Pressed())
    return;

  uint32_t current_time = GetTick();
  if ((current_time - last_button_time) < DEBOUNCE_MS) {
    return;
  }
  last_button_time = current_time;

  float new_setpoint = setpoint + delta;
  if ((new_setpoint >= SETPOINT_MIN) && (new_setpoint <= SETPOINT_MAX)) {
    setpoint = new_setpoint;
    setpoint_changed = 1;
  }
}

/* EXTI4 Interrupt - Button 1 (Setpoint UP) */
void EXTI4_IRQHandler(void) {
  Handle_Setpoint_Button(EXTI_PR_PR4, 1.0f);
}

/* EXTI9_5 Interrupt - Button 2 (Setpoint DOWN) */
void EXTI9_5_IRQHandler(void) {
  Handle_Setpoint_Button(EXTI_PR_PR5, -1.0f);
}

/* Standard Exception Handlers */
void NMI_Handler(void) {
  while (1)
    ;
}
void HardFault_Handler(void) {
  while (1)
    ;
}
void MemManage_Handler(void) {
  while (1)
    ;
}
void BusFault_Handler(void) {
  while (1)
    ;
}
void UsageFault_Handler(void) {
  while (1)
    ;
}
void SVC_Handler(void) {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void) {}
