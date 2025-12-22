#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>

/* ============================================================================
 * KONFIGURASI SISTEM
 * ============================================================================
 */
#define PWM_PERIOD 999
#define DEBOUNCE_MS 200

/* ============================================================================
 * PID CONTROLLER CONFIGURATION
 * ============================================================================
 */
#define PID_KP 15.0f /* Proportional Gain (Increased from 10.0) */
#define PID_KI 0.5f  /* Integral Gain */
#define PID_KD 2.0f  /* Derivative Gain */

#define PID_SETPOINT_DEFAULT 30.0f /* Default target temperature (°C) */
#define PID_SETPOINT_MIN 20.0f     /* Minimum setpoint */
#define PID_SETPOINT_MAX 50.0f     /* Maximum setpoint */

#define PID_OUTPUT_MIN 0.0f   /* Minimum PWM output (%) */
#define PID_OUTPUT_MAX 100.0f /* Maximum PWM output (%) */

#define PID_INTEGRAL_MIN                                                       \
  -10.0f /* Anti-windup lower limit (Reduced from -100.0) */
#define PID_INTEGRAL_MAX 100.0f /* Anti-windup upper limit */

/* Temperature Thresholds for LED (relative to setpoint) */
#define TEMP_LED_MARGIN 5.0f /* ±5°C from setpoint */

/* UART Configuration */
#define UART_BAUD_RATE 115200
#define APB2_CLOCK 100000000 /* 100 MHz */

/* DS18B20 Commands */
#define DS18B20_CMD_SKIP_ROM 0xCC
#define DS18B20_CMD_CONVERT_T 0x44
#define DS18B20_CMD_READ_SCRATCHPAD 0xBE

/* ============================================================================
 * SYSTEM TICK - REGISTER LEVEL
 * ============================================================================
 *
 * Menggunakan SysTick untuk timing (BUKAN HAL_GetTick/HAL_Delay):
 * - SysTick_CTRL : Control and Status Register
 * - SysTick_LOAD : Reload Value Register
 * - SysTick_VAL  : Current Value Register
 *
 * SysTick dikonfigurasi untuk interrupt setiap 1ms
 *
 * ============================================================================
 */
volatile uint32_t systick_counter =
    0; /* Tick counter (ms) - replaces HAL tick */

/* ============================================================================
 * PID CONTROLLER STRUCTURE
 * ============================================================================
 */
typedef struct {
  /* Gains */
  float Kp; /* Proportional gain */
  float Ki; /* Integral gain */
  float Kd; /* Derivative gain */

  /* Setpoint */
  float setpoint; /* Target temperature */

  /* State variables */
  float integral;   /* Accumulated integral term */
  float prev_error; /* Previous error (for derivative) */

  /* Output limits */
  float output_min;
  float output_max;

  /* Anti-windup limits */
  float integral_min;
  float integral_max;

  /* Last computed values (for logging) */
  float last_error;
  float last_P;
  float last_I;
  float last_D;
  float last_output;

} PID_Controller_t;

/* ============================================================================
 * GLOBAL VARIABLES
 * ============================================================================
 */

/* PID Controller Instance */
PID_Controller_t pid = {.Kp = PID_KP,
                        .Ki = PID_KI,
                        .Kd = PID_KD,
                        .setpoint = PID_SETPOINT_DEFAULT,
                        .integral = 0.0f,
                        .prev_error = 0.0f,
                        .output_min = PID_OUTPUT_MIN,
                        .output_max = PID_OUTPUT_MAX,
                        .integral_min = PID_INTEGRAL_MIN,
                        .integral_max = PID_INTEGRAL_MAX,
                        .last_error = 0.0f,
                        .last_P = 0.0f,
                        .last_I = 0.0f,
                        .last_D = 0.0f,
                        .last_output = 0.0f};

/* System State */
volatile uint8_t fan_speed_percent = 0;
volatile uint32_t last_button_time = 0;
volatile uint8_t flag_read_temp = 0;
volatile uint8_t setpoint_changed = 0;

float current_temperature = 0.0f;
uint8_t sensor_ok = 0;
uint32_t seconds_counter = 0;

/* ============================================================================
 * FUNCTION PROTOTYPES
 * ============================================================================
 */

/* System Init */
void SystemClock_Config(void);
static void SysTick_Init_RegisterLevel(void);
static void MX_GPIO_Init_RegisterLevel(void);
static void MX_TIM1_PWM_Init_RegisterLevel(void);
static void MX_TIM2_Interrupt_Init_RegisterLevel(void);
static void MX_USART1_Init_RegisterLevel(void);

/* Timing Functions - Register Level (NO HAL!) */
void DWT_Init(void);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
uint32_t GetTick(void);

/* UART Functions */
void UART_SendChar(char c);
void UART_SendString(const char *str);
void UART_SendFloat(float value, uint8_t decimals);
void UART_SendInt(int32_t value);

/* PID Controller Functions */
void PID_Init(PID_Controller_t *pid);
void PID_Reset(PID_Controller_t *pid);
float PID_Compute(PID_Controller_t *pid, float current_value);
void PID_SetSetpoint(PID_Controller_t *pid, float setpoint);
void PID_SetTunings(PID_Controller_t *pid, float Kp, float Ki, float Kd);

/* System Functions */
void Set_Fan_Speed_RegisterLevel(uint8_t percent);
void Update_LEDs_Temperature(float temp, float setpoint);
void Print_Status(void);
void Print_PID_Header(void);
void Print_All_Register_Status(void);
void Error_Handler(void);

/* DS18B20 Functions */
void DS18B20_SetPinOutput(void);
void DS18B20_SetPinInput(void);
uint8_t DS18B20_Reset(void);
void DS18B20_WriteBit(uint8_t bit);
uint8_t DS18B20_ReadBit(void);
void DS18B20_WriteByte(uint8_t byte);
uint8_t DS18B20_ReadByte(void);
float DS18B20_ReadTemperature(void);

/* ============================================================================
 * TIMING FUNCTIONS - REGISTER LEVEL (Replaces HAL_Delay & HAL_GetTick)
 * ============================================================================
 *
 * 1. DWT (Data Watchpoint and Trace) untuk delay_us():
 *    - Register: CoreDebug->DEMCR, DWT->CYCCNT, DWT->CTRL
 *    - CYCCNT menghitung clock cycles
 *    - Resolusi: 1 cycle = 10ns @ 100MHz
 *
 * 2. SysTick untuk delay_ms() dan GetTick():
 *    - Register: SysTick->CTRL, SysTick->LOAD, SysTick->VAL
 *    - Dikonfigurasi interrupt setiap 1ms
 *    - systick_counter di-increment di SysTick_Handler
 *
 * ============================================================================
 */

/* Inisialisasi DWT untuk microsecond delay */
void DWT_Init(void) {
  /* Enable TRC (Trace) - Register: CoreDebug->DEMCR */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  /* Reset cycle counter - Register: DWT->CYCCNT */
  DWT->CYCCNT = 0;

  /* Enable cycle counter - Register: DWT->CTRL bit 0 */
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* Delay microseconds menggunakan DWT CYCCNT (Register Level) */
void delay_us(uint32_t us) {
  uint32_t startTick = DWT->CYCCNT;
  uint32_t delayTicks = us * (SystemCoreClock / 1000000);

  while ((DWT->CYCCNT - startTick) < delayTicks)
    ;
}

/* Delay milliseconds menggunakan SysTick counter (Register Level) */
/* Replaces HAL_Delay() */
void delay_ms(uint32_t ms) {
  uint32_t start = systick_counter;
  while ((systick_counter - start) < ms)
    ;
}

/* Get current tick count (Register Level) */
/* Replaces HAL_GetTick() */
uint32_t GetTick(void) { return systick_counter; }

/* ============================================================================
 * SYSTICK INIT - REGISTER LEVEL
 * ============================================================================
 *
 * SysTick Configuration (replaces HAL SysTick):
 * - Clock source: AHB (100MHz)
 * - Reload value: untuk 1ms interrupt
 * - LOAD = (SYSCLK / 1000) - 1 = 99999
 *
 * Register:
 * - SysTick->CTRL : [2]=CLKSOURCE, [1]=TICKINT, [0]=ENABLE
 * - SysTick->LOAD : Reload value (24-bit)
 * - SysTick->VAL  : Current value (write any value to clear)
 *
 * ============================================================================
 */
static void SysTick_Init_RegisterLevel(void) {
  /* Disable SysTick dulu */
  SysTick->CTRL = 0;

  /* Set reload value untuk 1ms interrupt */
  /* LOAD = (SystemCoreClock / 1000) - 1 = 99999 untuk 100MHz */
  SysTick->LOAD = (SystemCoreClock / 1000) - 1;

  /* Clear current value */
  SysTick->VAL = 0;

  /* Set priority (optional) */
  NVIC_SetPriority(SysTick_IRQn, 0);

  /* Enable SysTick dengan:
   * Bit 2: CLKSOURCE = 1 (Processor clock / AHB)
   * Bit 1: TICKINT = 1 (Enable interrupt)
   * Bit 0: ENABLE = 1 (Enable counter)
   */
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk |
                  SysTick_CTRL_ENABLE_Msk;
}

/* ============================================================================
 * UART FUNCTIONS - REGISTER LEVEL
 * ============================================================================
 */

/* Kirim 1 karakter via UART (Register Level) */
void UART_SendChar(char c) {
  /* Tunggu sampai TXE (Transmit Data Register Empty) = 1 */
  while (!(USART1->SR & USART_SR_TXE))
    ;

  /* Tulis data ke Data Register */
  USART1->DR = c;
}

/* Kirim string via UART */
void UART_SendString(const char *str) {
  while (*str) {
    UART_SendChar(*str++);
  }
}

/* Kirim integer via UART */
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

/* Kirim float via UART */
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

/* ============================================================================
 * PID CONTROLLER FUNCTIONS
 * ============================================================================
 */

/* Inisialisasi PID Controller */
void PID_Init(PID_Controller_t *pid_ctrl) {
  pid_ctrl->integral = 0.0f;
  pid_ctrl->prev_error = 0.0f;
  pid_ctrl->last_error = 0.0f;
  pid_ctrl->last_P = 0.0f;
  pid_ctrl->last_I = 0.0f;
  pid_ctrl->last_D = 0.0f;
  pid_ctrl->last_output = 0.0f;
}

/* Reset PID state */
void PID_Reset(PID_Controller_t *pid_ctrl) {
  pid_ctrl->integral = 0.0f;
  pid_ctrl->prev_error = 0.0f;
}

/* Set setpoint baru */
void PID_SetSetpoint(PID_Controller_t *pid_ctrl, float setpoint) {
  if (setpoint < PID_SETPOINT_MIN)
    setpoint = PID_SETPOINT_MIN;
  if (setpoint > PID_SETPOINT_MAX)
    setpoint = PID_SETPOINT_MAX;

  pid_ctrl->setpoint = setpoint;
}

/* Set PID tuning parameters */
void PID_SetTunings(PID_Controller_t *pid_ctrl, float Kp, float Ki, float Kd) {
  if (Kp < 0 || Ki < 0 || Kd < 0)
    return;

  pid_ctrl->Kp = Kp;
  pid_ctrl->Ki = Ki;
  pid_ctrl->Kd = Kd;
}

/* Compute PID output */
float PID_Compute(PID_Controller_t *pid_ctrl, float current_value) {
  float output;

  /* ========== 1. Hitung Error ========== */
  float error = current_value - pid_ctrl->setpoint;

  /* ========== 2. Proportional Term ========== */
  float P = pid_ctrl->Kp * error;

  /* ========== 3. Integral Term dengan Anti-Windup ========== */
  pid_ctrl->integral += error;

  /* Anti-windup: batasi akumulasi integral */
  if (pid_ctrl->integral > pid_ctrl->integral_max)
    pid_ctrl->integral = pid_ctrl->integral_max;
  else if (pid_ctrl->integral < pid_ctrl->integral_min)
    pid_ctrl->integral = pid_ctrl->integral_min;

  float I = pid_ctrl->Ki * pid_ctrl->integral;

  /* ========== 4. Derivative Term ========== */
  float D = pid_ctrl->Kd * (error - pid_ctrl->prev_error);
  pid_ctrl->prev_error = error;

  /* ========== 5. Hitung Total Output ========== */
  output = P + I + D;

  /* ========== 6. Clamp Output ========== */
  if (output > pid_ctrl->output_max)
    output = pid_ctrl->output_max;
  else if (output < pid_ctrl->output_min)
    output = pid_ctrl->output_min;

  /* ========== 7. Simpan untuk logging ========== */
  pid_ctrl->last_error = error;
  pid_ctrl->last_P = P;
  pid_ctrl->last_I = I;
  pid_ctrl->last_D = D;
  pid_ctrl->last_output = output;

  return output;
}

/* ============================================================================
 * MAIN FUNCTION
 * ============================================================================
 */
int main(void) {
  /* HAL Init - hanya untuk basic system (NVIC priority grouping) */
  HAL_Init();
  SystemClock_Config();

  /* Inisialisasi timing - REGISTER LEVEL (bukan HAL!) */
  DWT_Init();
  SysTick_Init_RegisterLevel();

  /* Inisialisasi Peripheral - SEMUA REGISTER LEVEL */
  MX_GPIO_Init_RegisterLevel();
  MX_USART1_Init_RegisterLevel();
  MX_TIM1_PWM_Init_RegisterLevel();
  MX_TIM2_Interrupt_Init_RegisterLevel();

  /* Inisialisasi PID Controller */
  PID_Init(&pid);

  /* Set speed awal 0% */
  Set_Fan_Speed_RegisterLevel(0);

  /* ==================== WELCOME MESSAGE ==================== */
  UART_SendString("\r\n");
  UART_SendString(
      "================================================================\r\n");
  UART_SendString("   AUTOMATIC FAN SPEED CONTROL WITH PID CONTROLLER\r\n");
  UART_SendString("   TUGAS BESAR MIKROPROSESOR & SISTEM KENDALI MEKANIKA\r\n");
  UART_SendString("   STM32F411 Black Pill - Register Level Programming\r\n");
  UART_SendString(
      "================================================================\r\n");
  UART_SendString("\r\n");

  /* Print Register Configuration */
  Print_All_Register_Status();

  /* Print PID Configuration */
  UART_SendString("[ PID CONTROLLER CONFIGURATION ]\r\n");
  UART_SendString("------------------------------------------------\r\n");
  UART_SendString("  Kp = ");
  UART_SendFloat(pid.Kp, 2);
  UART_SendString("  (Proportional Gain)\r\n");
  UART_SendString("  Ki = ");
  UART_SendFloat(pid.Ki, 2);
  UART_SendString("   (Integral Gain)\r\n");
  UART_SendString("  Kd = ");
  UART_SendFloat(pid.Kd, 2);
  UART_SendString("   (Derivative Gain)\r\n");
  UART_SendString("  Setpoint = ");
  UART_SendFloat(pid.setpoint, 1);
  UART_SendString(" C\r\n");
  UART_SendString("  Output Range: ");
  UART_SendFloat(pid.output_min, 0);
  UART_SendString(" - ");
  UART_SendFloat(pid.output_max, 0);
  UART_SendString(" %\r\n");
  UART_SendString("  Anti-Windup: ");
  UART_SendFloat(pid.integral_min, 0);
  UART_SendString(" to ");
  UART_SendFloat(pid.integral_max, 0);
  UART_SendString("\r\n\r\n");

  /* Test LED - menggunakan delay_ms() bukan HAL_Delay() */
  UART_SendString("[BOOT] Testing LEDs...\r\n");
  GPIOB->BSRR = GPIO_PIN_0;       /* LED Hijau ON */
  delay_ms(200);                  /* Register-based delay */
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
      delay_ms(100); /* Register-based delay */
    }
  }

  UART_SendString("[BOOT] DS18B20 detected OK!\r\n\r\n");

  /* Print Control Info */
  UART_SendString("CONTROLS (Closed-Loop PID):\r\n");
  UART_SendString("  BTN1 (PB4): Increase setpoint +1 C\r\n");
  UART_SendString("  BTN2 (PB5): Decrease setpoint -1 C\r\n\r\n");

  UART_SendString("PID BEHAVIOR:\r\n");
  UART_SendString("  Error = T_actual - Setpoint\r\n");
  UART_SendString("  Error > 0 -> Fan speeds up (cooling needed)\r\n");
  UART_SendString("  Error < 0 -> Fan slows down (already cool)\r\n");
  UART_SendString("  Error = 0 -> Maintains current speed\r\n\r\n");

  UART_SendString(">>> Starting PID Control Loop...\r\n\r\n");

  Print_PID_Header();

  /* Baca suhu pertama kali */
  current_temperature = DS18B20_ReadTemperature();

  /* Set prev_error untuk menghindari derivative kick saat start */
  pid.prev_error = current_temperature - pid.setpoint;

  /* ==================== MAIN LOOP ==================== */
  while (1) {
    /* Cek jika setpoint berubah (dari interrupt) */
    if (setpoint_changed) {
      setpoint_changed = 0;
      UART_SendString("\r\n>>> Setpoint changed to: ");
      UART_SendFloat(pid.setpoint, 1);
      UART_SendString(" C\r\n");

      Print_PID_Header();
    }

    /* Cek flag dari Timer Interrupt (setiap 1 detik) */
    if (flag_read_temp) {
      flag_read_temp = 0;

      /* ========== 1. Baca suhu dari DS18B20 ========== */
      current_temperature = DS18B20_ReadTemperature();

      /* ========== 2. Hitung PID Output ========== */
      float pid_output = PID_Compute(&pid, current_temperature);

      /* ========== 3. Set Fan Speed ========== */
      Set_Fan_Speed_RegisterLevel((uint8_t)pid_output);

      /* ========== 4. Print status ========== */
      Print_Status();

      seconds_counter++;
    }

    /* Update LED */
    Update_LEDs_Temperature(current_temperature, pid.setpoint);

    delay_ms(10); /* Register-based delay */
  }
}

/* ============================================================================
 * PRINT PID HEADER
 * ============================================================================
 */
void Print_PID_Header(void) {
  UART_SendString("\r\n");
  UART_SendString("  TIME | T_act | Setpt | Error |    P    |    I    |    D   "
                  " | Output | FAN\r\n");
  UART_SendString("-------|-------|-------|-------|---------|---------|--------"
                  "-|--------|----\r\n");
}

/* ============================================================================
 * PRINT STATUS
 * ============================================================================
 */
void Print_Status(void) {
  /* Time */
  UART_SendString("  ");
  if (seconds_counter < 10)
    UART_SendString("   ");
  else if (seconds_counter < 100)
    UART_SendString("  ");
  else if (seconds_counter < 1000)
    UART_SendString(" ");
  UART_SendInt(seconds_counter);
  UART_SendString("s | ");

  /* T_actual */
  UART_SendFloat(current_temperature, 1);
  UART_SendString(" | ");

  /* Setpoint */
  UART_SendFloat(pid.setpoint, 1);
  UART_SendString(" | ");

  /* Error */
  if (pid.last_error >= 0)
    UART_SendString(" ");
  UART_SendFloat(pid.last_error, 2);
  UART_SendString(" | ");

  /* P term */
  if (pid.last_P >= 0)
    UART_SendString(" ");
  UART_SendFloat(pid.last_P, 2);
  UART_SendString(" | ");

  /* I term */
  if (pid.last_I >= 0)
    UART_SendString(" ");
  UART_SendFloat(pid.last_I, 2);
  UART_SendString(" | ");

  /* D term */
  if (pid.last_D >= 0)
    UART_SendString(" ");
  UART_SendFloat(pid.last_D, 2);
  UART_SendString(" | ");

  /* Output */
  if (pid.last_output < 10)
    UART_SendString("  ");
  else if (pid.last_output < 100)
    UART_SendString(" ");
  UART_SendFloat(pid.last_output, 1);
  UART_SendString(" | ");

  /* Fan % */
  if (fan_speed_percent < 10)
    UART_SendString(" ");
  if (fan_speed_percent < 100)
    UART_SendString(" ");
  UART_SendInt(fan_speed_percent);
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
  UART_SendString(", Line5=");
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

/* ============================================================================
 * GPIO INIT - REGISTER LEVEL
 * ============================================================================
 */
static void MX_GPIO_Init_RegisterLevel(void) {
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
 * USART1 INIT - REGISTER LEVEL
 * ============================================================================
 */
static void MX_USART1_Init_RegisterLevel(void) {
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
  USART1->BRR = 0x364; /* 115200 baud @ 100MHz */
  USART1->CR2 = 0;
  USART1->CR3 = 0;
  USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

/* ============================================================================
 * TIM1 PWM INIT - REGISTER LEVEL
 * ============================================================================
 */
static void MX_TIM1_PWM_Init_RegisterLevel(void) {
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

  /* PA8: TIM1_CH1 (AF1) */
  GPIOA->MODER &= ~GPIO_MODER_MODER8;
  GPIOA->MODER |= GPIO_MODER_MODER8_1;
  GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL8;
  GPIOA->AFR[1] |= (1 << GPIO_AFRH_AFSEL8_Pos);

  TIM1->CR1 = 0;
  TIM1->CR2 = 0;
  /*
   * PERUBAHAN FREKUENSI PWM:
   * Sebelumnya: PSC=3 (25MHz), ARR=999 -> 25kHz (Terlalu cepat untuk
   * transistor/kipas biasa) Sekarang:   PSC=999 (100kHz), ARR=999 -> 100Hz
   *
   * Rumus: Freq = SystemCoreClock / ((PSC + 1) * (ARR + 1))
   *        100  = 100,000,000 / (1000 * 1000)
   */
  TIM1->PSC = 999;        /* 100MHz / 1000 = 100kHz */
  TIM1->ARR = PWM_PERIOD; /* 100kHz / 1000 = 100Hz */
  TIM1->CCR1 = 0;

  TIM1->CCMR1 &= ~TIM_CCMR1_OC1M;
  TIM1->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1);
  TIM1->CCMR1 |= TIM_CCMR1_OC1PE;

  TIM1->CCER |= TIM_CCER_CC1E;
  TIM1->BDTR |= TIM_BDTR_MOE;
  TIM1->CR1 |= TIM_CR1_CEN;
}

/* ============================================================================
 * TIM2 TIMER INTERRUPT INIT - REGISTER LEVEL
 * ============================================================================
 */
static void MX_TIM2_Interrupt_Init_RegisterLevel(void) {
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
 * SET FAN SPEED - REGISTER LEVEL
 * ============================================================================
 */
void Set_Fan_Speed_RegisterLevel(uint8_t percent) {
  if (percent > 100)
    percent = 100;
  fan_speed_percent = percent;

  TIM1->CCR1 = (uint32_t)((percent * PWM_PERIOD) / 100);
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

/* ============================================================================
 * INTERRUPT HANDLERS
 * ============================================================================
 */

/* SysTick Handler - Increment tick counter (Register Level) */
void SysTick_Handler(void) { systick_counter++; }

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

    /* Debounce menggunakan GetTick() - bukan HAL_GetTick() */
    uint32_t current_time = GetTick();
    if ((current_time - last_button_time) < DEBOUNCE_MS)
      return;
    last_button_time = current_time;

    PID_SetSetpoint(&pid, pid.setpoint + 1.0f);
    setpoint_changed = 1;
  }
}

/* EXTI9_5 Interrupt - Button 2 (Setpoint DOWN) */
void EXTI9_5_IRQHandler(void) {
  if (EXTI->PR & EXTI_PR_PR5) {
    EXTI->PR = EXTI_PR_PR5;

    /* Debounce menggunakan GetTick() - bukan HAL_GetTick() */
    uint32_t current_time = GetTick();
    if ((current_time - last_button_time) < DEBOUNCE_MS)
      return;
    last_button_time = current_time;

    PID_SetSetpoint(&pid, pid.setpoint - 1.0f);
    setpoint_changed = 1;
  }
}

/* ============================================================================
 * DS18B20 FUNCTIONS - REGISTER LEVEL
 * ============================================================================
 */

void DS18B20_SetPinOutput(void) {
  GPIOA->MODER &= ~GPIO_MODER_MODER0;
  GPIOA->MODER |= GPIO_MODER_MODER0_0;
}

void DS18B20_SetPinInput(void) { GPIOA->MODER &= ~GPIO_MODER_MODER0; }

uint8_t DS18B20_Reset(void) {
  uint8_t presence = 0;
  DS18B20_SetPinOutput();
  GPIOA->BSRR = GPIO_PIN_0 << 16;
  delay_us(480);
  DS18B20_SetPinInput();
  delay_us(70);
  if (!(GPIOA->IDR & GPIO_PIN_0))
    presence = 1;
  delay_us(410);
  return presence;
}

void DS18B20_WriteBit(uint8_t bit) {
  DS18B20_SetPinOutput();
  GPIOA->BSRR = GPIO_PIN_0 << 16;
  if (bit) {
    delay_us(1);
    DS18B20_SetPinInput();
    delay_us(60);
  } else {
    delay_us(60);
    DS18B20_SetPinInput();
    delay_us(1);
  }
}

uint8_t DS18B20_ReadBit(void) {
  uint8_t bit = 0;
  DS18B20_SetPinOutput();
  GPIOA->BSRR = GPIO_PIN_0 << 16;
  delay_us(2);
  DS18B20_SetPinInput();
  delay_us(10);
  if (GPIOA->IDR & GPIO_PIN_0)
    bit = 1;
  delay_us(50);
  return bit;
}

void DS18B20_WriteByte(uint8_t byte) {
  for (uint8_t i = 0; i < 8; i++) {
    DS18B20_WriteBit(byte & 0x01);
    byte >>= 1;
  }
}

uint8_t DS18B20_ReadByte(void) {
  uint8_t byte = 0;
  for (uint8_t i = 0; i < 8; i++) {
    byte >>= 1;
    if (DS18B20_ReadBit())
      byte |= 0x80;
  }
  return byte;
}

float DS18B20_ReadTemperature(void) {
  uint8_t temp_lsb, temp_msb;
  int16_t temp_raw;

  if (!DS18B20_Reset())
    return -999.0f;
  DS18B20_WriteByte(DS18B20_CMD_SKIP_ROM);
  DS18B20_WriteByte(DS18B20_CMD_CONVERT_T);
  delay_ms(750); /* Register-based delay - bukan HAL_Delay! */

  if (!DS18B20_Reset())
    return -999.0f;
  DS18B20_WriteByte(DS18B20_CMD_SKIP_ROM);
  DS18B20_WriteByte(DS18B20_CMD_READ_SCRATCHPAD);

  temp_lsb = DS18B20_ReadByte();
  temp_msb = DS18B20_ReadByte();

  temp_raw = (temp_msb << 8) | temp_lsb;
  return (float)temp_raw / 16.0f;
}

/* ============================================================================
 * SYSTEM CLOCK CONFIG (HAL - required for complex clock tree)
 * ============================================================================
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
    Error_Handler();
}

void Error_Handler(void) {
  __disable_irq();
  while (1) {
    GPIOB->ODR ^= GPIO_PIN_2;
    for (volatile int i = 0; i < 500000; i++)
      ;
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