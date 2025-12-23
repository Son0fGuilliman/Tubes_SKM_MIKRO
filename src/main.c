#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>

/* Modules */
#include "debug_utils.h"
#include "ds18b20.h"
#include "fan.h"
#include "system_config.h"
#include "timing.h"
#include "uart_lib.h"

/* ============================================================================
 * CONTROL CONFIGURATION (SIMPLE ON/OFF)
 * ============================================================================
 */
#define SETPOINT_DEFAULT 30.0f /* Default target temperature (°C) */
#define SETPOINT_MIN 20.0f     /* Minimum setpoint */
#define SETPOINT_MAX 50.0f     /* Maximum setpoint */

#define OUTPUT_MIN 0.0f   /* Minimum PWM output (%) */
#define OUTPUT_MAX 100.0f /* Maximum PWM output (%) */

#define DEBOUNCE_MS 200

/* ============================================================================
 * GLOBAL VARIABLES
 * ============================================================================
 */

/* Control Variables */
float setpoint = SETPOINT_DEFAULT;

/* System State */
/* fan_speed_percent is extern in fan.h */
volatile uint32_t last_button_time = 0;
volatile uint8_t flag_read_temp = 0;
volatile uint8_t setpoint_changed = 0;

float current_temperature = 0.0f;
uint8_t sensor_ok = 0;
uint32_t seconds_counter = 0;

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

  /* ==================== WELCOME MESSAGE ==================== */
  UART_SendString("\r\n");
  UART_SendString(
      "================================================================\r\n");
  UART_SendString("   AUTOMATIC FAN SPEED CONTROL (ON/OFF CONTROL)\r\n");
  UART_SendString("   TUGAS BESAR MIKROPROSESOR & SISTEM KENDALI MEKANIKA\r\n");
  UART_SendString("   STM32F411 Black Pill - Register Level Programming\r\n");
  UART_SendString(
      "================================================================\r\n");
  UART_SendString("\r\n");

  /* Print Register Configuration */
  Print_All_Register_Status();

  /* Print Control Configuration */
  UART_SendString("[ CONTROL CONFIGURATION ]\r\n");
  UART_SendString("------------------------------------------------\r\n");
  UART_SendString("  Setpoint = ");
  UART_SendFloat(setpoint, 1);
  UART_SendString(" C\r\n");
  UART_SendString("  Type: Simple On/Off\r\n");
  UART_SendString("\r\n\r\n");

  /* Test LED */
  UART_SendString("[BOOT] Testing LEDs...\r\n");
  GPIOB->BSRR = GPIO_PIN_0; /* LED Hijau ON */
  delay_ms(200);
  GPIOB->BSRR = GPIO_PIN_0 << 16; /* LED Hijau OFF */
  GPIOB->BSRR = GPIO_PIN_1;       /* LED Kuning ON */
  delay_ms(200);
  GPIOB->BSRR = GPIO_PIN_1 << 16;
  GPIOB->BSRR = GPIO_PIN_2; /* LED Merah ON */
  delay_ms(200);
  GPIOB->BSRR = GPIO_PIN_2 << 16;
  UART_SendString("[BOOT] LED test OK\r\n");

  /* Check DS18B20 */
  UART_SendString("[BOOT] Detecting DS18B20 sensor...\r\n");
  sensor_ok = DS18B20_Reset();

  if (!sensor_ok) {
    UART_SendString("\r\n[ERROR] DS18B20 NOT DETECTED!\r\n");
    UART_SendString(
        "Check: DQ -> PA0, VDD -> 3.3V, GND -> GND, 4.7K pull-up\r\n");

    while (1) {
      GPIOB->ODR ^= GPIO_PIN_2;
      delay_ms(100);
    }
  }

  UART_SendString("[BOOT] DS18B20 detected OK!\r\n\r\n");

  /* Print Control Info */
  UART_SendString("CONTROLS (Simple On/Off):\r\n");
  UART_SendString("  BTN1 (PB4): Increase setpoint +1 C\r\n");
  UART_SendString("  BTN2 (PB5): Decrease setpoint -1 C\r\n\r\n");

  UART_SendString("BEHAVIOR:\r\n");
  UART_SendString("  Temp > Setpoint -> Fan MAX\r\n");
  UART_SendString("  Temp <= Setpoint -> Fan OFF\r\n\r\n");

  UART_SendString(">>> Starting Control Loop...\r\n\r\n");

  Print_Control_Header();

  /* Baca suhu pertama kali */
  current_temperature = DS18B20_ReadTemperature();

  /* ==================== MAIN LOOP ==================== */
  while (1) {
    /* Cek jika setpoint berubah (dari interrupt) */
    if (setpoint_changed) {
      setpoint_changed = 0;
      UART_SendString("\r\n>>> Setpoint changed to: ");
      UART_SendFloat(setpoint, 1);
      UART_SendString(" C\r\n");

      Print_Control_Header();
    }

    /* Cek flag dari Timer Interrupt (setiap 1 detik) */
    if (flag_read_temp) {
      flag_read_temp = 0;

      /* ========== 1. Baca suhu dari DS18B20 ========== */
      current_temperature = DS18B20_ReadTemperature();

      /* ========== 2. Hitung Output (On/Off) ========== */
      float output = 0.0f;
      if (current_temperature > setpoint) {
        output = OUTPUT_MAX;
      } else {
        output = OUTPUT_MIN;
      }

      /* ========== 3. Set Fan Speed ========== */
      Set_Fan_Speed_RegisterLevel((uint8_t)output);

      /* ========== 4. Print status ========== */
      Print_Status(seconds_counter, current_temperature, setpoint,
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

/* EXTI4 Interrupt - Button 1 (Setpoint UP) */
void EXTI4_IRQHandler(void) {
  if (EXTI->PR & EXTI_PR_PR4) {
    EXTI->PR = EXTI_PR_PR4;

    /* Debounce menggunakan GetTick() */
    uint32_t current_time = GetTick();
    if ((current_time - last_button_time) < DEBOUNCE_MS)
      return;
    last_button_time = current_time;

    if (setpoint < SETPOINT_MAX) {
      setpoint += 1.0f;
      setpoint_changed = 1;
    }
  }
}

/* EXTI9_5 Interrupt - Button 2 (Setpoint DOWN) */
void EXTI9_5_IRQHandler(void) {
  if (EXTI->PR & EXTI_PR_PR5) {
    EXTI->PR = EXTI_PR_PR5;

    /* Debounce menggunakan GetTick() */
    uint32_t current_time = GetTick();
    if ((current_time - last_button_time) < DEBOUNCE_MS)
      return;
    last_button_time = current_time;

    if (setpoint > SETPOINT_MIN) {
      setpoint -= 1.0f;
      setpoint_changed = 1;
    }
  }
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
