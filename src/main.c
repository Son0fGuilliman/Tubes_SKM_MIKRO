#include "stm32f4xx.h"

/* Modules */
#include "ds18b20.h"
#include "fan.h"
#include "system_config.h"
#include "timing.h"
#include "uart_lib.h"

/* ============================================================================
 * CONTROL CONFIGURATION (PID CONTROL)
 * ============================================================================
 */
#define SETPOINT_DEFAULT 30.0f /* Default target temperature (°C) */
#define SETPOINT_MIN 20.0f     /* Minimum setpoint */
#define SETPOINT_MAX 50.0f     /* Maximum setpoint */

#ifndef OPEN_LOOP_SWEEP_TEST
#define OPEN_LOOP_SWEEP_TEST 0
#endif

#ifndef OPEN_LOOP_FIXED_TEST
#define OPEN_LOOP_FIXED_TEST 0
#endif

#ifndef UART_PLOTTER_OUTPUT
#define UART_PLOTTER_OUTPUT 0
#endif

/* If 1: emit a single line like `>temp:32.1,setpoint:30.0,fan:55` (best for
 * "serial-plotter"). If 0: emit one line per variable like `>temp:32.1`
 * (Teleplot-compatible). */
#ifndef UART_PLOTTER_COMBINED_LINE
#define UART_PLOTTER_COMBINED_LINE 0
#endif

#ifndef UART_PLOTTER_FLOAT_DECIMALS
#define UART_PLOTTER_FLOAT_DECIMALS 1
#endif

#if (OPEN_LOOP_SWEEP_TEST && OPEN_LOOP_FIXED_TEST)
#error "Choose only one: OPEN_LOOP_SWEEP_TEST or OPEN_LOOP_FIXED_TEST"
#endif

#define OPEN_LOOP_TEST (OPEN_LOOP_SWEEP_TEST || OPEN_LOOP_FIXED_TEST)

#ifndef PID_KP
#define PID_KP 10.0f /* Proportional gain */
#endif
#ifndef PID_KI
#define PID_KI 0.0f /* Integral gain (set >0 after tuning) */
#endif
#ifndef PID_KD
#define PID_KD 0.0f /* Derivative gain (set >0 after tuning) */
#endif

#ifndef SCENARIO_NAME
#define SCENARIO_NAME DEFAULT
#endif

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

#define OUTPUT_MAX 100.0f /* Maximum PWM output (%) */

#define CONTROL_DT_SEC 1.0f /* TIM2 configured to 1 Hz (see system_config.c) */

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
static volatile uint8_t flag_setpoint_changed = 0;

#if !OPEN_LOOP_TEST
typedef struct {
  float kp;
  float ki;
  float kd;

  float integral_term; /* already multiplied by Ki (output unit: %) */
  float prev_error;
  float d_filtered;
} PID_Controller_t;

static PID_Controller_t pid = {
    .kp = PID_KP,
    .ki = PID_KI,
    .kd = PID_KD,
    .integral_term = 0.0f,
    .prev_error = 0.0f,
    .d_filtered = 0.0f,
};
#endif

#if OPEN_LOOP_SWEEP_TEST
#define OPEN_LOOP_STEP_DURATION_SEC 60u /* 60 seconds per duty */
static const uint8_t open_loop_duty_steps[] = {0, 20, 40, 60, 80, 100};
static volatile uint8_t open_loop_started = 0;
static uint8_t open_loop_step_idx = 0;
static uint32_t open_loop_step_start_time_sec = 0;
#endif

#if OPEN_LOOP_FIXED_TEST
#define OPEN_LOOP_RUN_DURATION_SEC 60u /* 60 seconds per run */
#define OPEN_LOOP_FIXED_DUTY_STEPS_COUNT 6u
static const uint8_t open_loop_fixed_duty_steps[OPEN_LOOP_FIXED_DUTY_STEPS_COUNT] = {
    0, 20, 40, 60, 80, 100,
};
static volatile uint8_t open_loop_started = 0;
static volatile uint8_t open_loop_duty_idx = 0;
static volatile uint8_t open_loop_duty_percent = 0;
static uint8_t open_loop_run_id = 0;
static uint32_t open_loop_run_start_time_sec = 0;
static volatile uint8_t flag_duty_changed = 0;
#endif

#if OPEN_LOOP_TEST
static volatile uint8_t flag_open_loop_start_request = 0;
#endif

static inline uint8_t Both_Buttons_Pressed(void) {
  return (GPIOB->IDR & (GPIO_PIN_4 | GPIO_PIN_5)) == 0;
}

#if OPEN_LOOP_TEST
static inline void OpenLoop_RequestStart(void) {
  uint32_t current_time = GetTick();
  if ((current_time - last_button_time) < DEBOUNCE_MS) {
    return;
  }
  last_button_time = current_time;
  flag_open_loop_start_request = 1;
}
#endif

#if !OPEN_LOOP_TEST
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

#if UART_PLOTTER_OUTPUT
#if UART_PLOTTER_COMBINED_LINE
  UART_SendChar('>');
  UART_SendString("temp:");
  UART_SendFloat(temp, UART_PLOTTER_FLOAT_DECIMALS);
  UART_SendChar(',');
  UART_SendString("setpoint:");
  UART_SendFloat(setpoint, UART_PLOTTER_FLOAT_DECIMALS);
  UART_SendChar(',');
  UART_SendString("fan:");
  UART_SendInt((int32_t)fan_percent);
  UART_SendString("\r\n");
#else
  UART_SendChar('>');
  UART_SendString("temp:");
  UART_SendFloat(temp, UART_PLOTTER_FLOAT_DECIMALS);
  UART_SendString("\r\n");

  UART_SendChar('>');
  UART_SendString("setpoint:");
  UART_SendFloat(setpoint, UART_PLOTTER_FLOAT_DECIMALS);
  UART_SendString("\r\n");

  UART_SendChar('>');
  UART_SendString("fan:");
  UART_SendInt((int32_t)fan_percent);
  UART_SendString("\r\n");
#endif
#endif
}
#endif

#if OPEN_LOOP_TEST
static void UART_LogTelemetry_OpenLoop(uint32_t time_sec, float temp,
                                       uint8_t duty_percent, uint8_t step_idx) {
  UART_SendInt((int32_t)time_sec);
  UART_SendChar(',');
  UART_SendFloat(temp, 1);
  UART_SendChar(',');
  UART_SendInt((int32_t)duty_percent);
  UART_SendChar(',');
  UART_SendInt((int32_t)step_idx);
  UART_SendString("\r\n");

#if UART_PLOTTER_OUTPUT
#if UART_PLOTTER_COMBINED_LINE
  UART_SendChar('>');
  UART_SendString("temp:");
  UART_SendFloat(temp, UART_PLOTTER_FLOAT_DECIMALS);
  UART_SendChar(',');
  UART_SendString("duty:");
  UART_SendInt((int32_t)duty_percent);
  UART_SendChar(',');
  UART_SendString("step:");
  UART_SendInt((int32_t)step_idx);
  UART_SendString("\r\n");
#else
  UART_SendChar('>');
  UART_SendString("temp:");
  UART_SendFloat(temp, UART_PLOTTER_FLOAT_DECIMALS);
  UART_SendString("\r\n");

  UART_SendChar('>');
  UART_SendString("duty:");
  UART_SendInt((int32_t)duty_percent);
  UART_SendString("\r\n");
#endif
#endif
}

static void UART_OpenLoopMarker(const char *tag, uint32_t a, uint32_t b) {
  UART_SendString("#OPEN_LOOP,");
  UART_SendString(tag);
  UART_SendChar(',');
  UART_SendInt((int32_t)a);
  UART_SendChar(',');
  UART_SendInt((int32_t)b);
  UART_SendString("\r\n");
}
#endif

static inline float ClampFloat(float value, float min_value, float max_value) {
  if (value < min_value)
    return min_value;
  if (value > max_value)
    return max_value;
  return value;
}

#if !OPEN_LOOP_TEST
static float PID_ComputeCoolingOnly(PID_Controller_t *controller, float temp,
                                    float setpoint, uint8_t reset_state) {
  float error = temp - setpoint; /* >0 means too hot => need cooling */
  float dt = CONTROL_DT_SEC;

  if (reset_state) {
    controller->integral_term = 0.0f;
    controller->prev_error = error; /* avoid derivative kick on reset */
    controller->d_filtered = 0.0f;
  }

  float p = controller->kp * error;

  float d_raw = (error - controller->prev_error) / dt;
#ifdef PID_D_FILTER_ALPHA
  controller->d_filtered =
      (PID_D_FILTER_ALPHA * controller->d_filtered) + ((1.0f - PID_D_FILTER_ALPHA) * d_raw);
#else
  controller->d_filtered = d_raw;
#endif
  float d = controller->kd * controller->d_filtered;

  if (error <= 0.0f) {
    controller->integral_term = 0.0f;
  } else {
    float output_unsat = p + controller->integral_term + d;
    uint8_t allow_integrate =
        (output_unsat > 0.0f) && (output_unsat < OUTPUT_MAX);
    if (allow_integrate) {
      controller->integral_term += (controller->ki * error * dt);
      controller->integral_term =
          ClampFloat(controller->integral_term, 0.0f, OUTPUT_MAX);
    }
  }

  float output = p + controller->integral_term + d;
  controller->prev_error = error;

  output = ClampFloat(output, 0.0f, OUTPUT_MAX);
  return output;
}
#endif

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
#if OPEN_LOOP_TEST
  UART_SendString("t,temp,duty,step\r\n");
#if OPEN_LOOP_SWEEP_TEST
  UART_SendString("#OPEN_LOOP,INFO,mode=sweep;press_both_buttons_to_start,0\r\n");
#else
  UART_SendString(
      "#OPEN_LOOP,INFO,mode=fixed;PB4:next_duty;PB5:start_run,0\r\n");
#endif
#else
  UART_SendString("t,temp,setpoint,fan\r\n");
#endif

#if !OPEN_LOOP_TEST
  UART_SendString("#SCENARIO,");
  UART_SendString(STR(SCENARIO_NAME));
  UART_SendString(",Kp=");
  UART_SendString(STR(PID_KP));
  UART_SendString(",Ki=");
  UART_SendString(STR(PID_KI));
  UART_SendString(",Kd=");
  UART_SendString(STR(PID_KD));
  UART_SendString("\r\n");
#endif

  /* Check DS18B20 */
  if (!DS18B20_Reset()) {
    UART_SendString("DS18B20 not detected\r\n");
  }

  /* Baca suhu pertama kali */
  float current_temperature = DS18B20_ReadTemperature();
  uint32_t seconds_counter = 0;
#if !OPEN_LOOP_TEST
  pid.prev_error = current_temperature - setpoint;
#endif

#if OPEN_LOOP_FIXED_TEST
  open_loop_duty_idx = 0;
  open_loop_duty_percent = open_loop_fixed_duty_steps[open_loop_duty_idx];
  flag_duty_changed = 1;
#endif

  /* ==================== MAIN LOOP ==================== */
  while (1) {
    /* Cek flag dari Timer Interrupt (setiap 1 detik) */
    if (flag_read_temp) {
      flag_read_temp = 0;

      /* ========== 1. Baca suhu dari DS18B20 ========== */
      current_temperature = DS18B20_ReadTemperature();

#if OPEN_LOOP_SWEEP_TEST
      if (!open_loop_started) {
        if (flag_open_loop_start_request || Both_Buttons_Pressed()) {
          flag_open_loop_start_request = 0;
          open_loop_started = 1;
          open_loop_step_idx = 0;
          open_loop_step_start_time_sec = seconds_counter;
          UART_OpenLoopMarker("START", seconds_counter,
                              open_loop_duty_steps[open_loop_step_idx]);
        }

        Set_Fan_Speed_RegisterLevel(0);
        UART_LogTelemetry_OpenLoop(seconds_counter, current_temperature, 0, 255);
        seconds_counter++;
        continue;
      }

      uint8_t duty = open_loop_duty_steps[open_loop_step_idx];
      Set_Fan_Speed_RegisterLevel(duty);
      UART_LogTelemetry_OpenLoop(seconds_counter, current_temperature, duty,
                                 open_loop_step_idx);

      uint32_t elapsed_samples =
          (seconds_counter + 1u) - open_loop_step_start_time_sec;
      if (elapsed_samples >= OPEN_LOOP_STEP_DURATION_SEC) {
        open_loop_step_idx++;
        open_loop_step_start_time_sec = seconds_counter + 1;

        uint8_t steps_count =
            (uint8_t)(sizeof(open_loop_duty_steps) / sizeof(open_loop_duty_steps[0]));
        if (open_loop_step_idx >= steps_count) {
          UART_OpenLoopMarker("DONE", seconds_counter, 0);
          Set_Fan_Speed_RegisterLevel(0);
          open_loop_started = 0;
          open_loop_step_idx = 0;
        } else {
          UART_OpenLoopMarker("STEP", seconds_counter,
                              open_loop_duty_steps[open_loop_step_idx]);
        }
      }

      seconds_counter++;
#elif OPEN_LOOP_FIXED_TEST
      if (flag_duty_changed) {
        flag_duty_changed = 0;
        UART_OpenLoopMarker("DUTY", seconds_counter, open_loop_duty_percent);
      }

      if (!open_loop_started) {
        if (flag_open_loop_start_request) {
          flag_open_loop_start_request = 0;
          open_loop_started = 1;
          open_loop_run_start_time_sec = seconds_counter;
          open_loop_run_id++;
          UART_OpenLoopMarker("START", seconds_counter, open_loop_duty_percent);
        }

        Set_Fan_Speed_RegisterLevel(0);
        UART_LogTelemetry_OpenLoop(seconds_counter, current_temperature,
                                   open_loop_duty_percent, 255);
        seconds_counter++;
        continue;
      }

      Set_Fan_Speed_RegisterLevel(open_loop_duty_percent);
      UART_LogTelemetry_OpenLoop(seconds_counter, current_temperature,
                                 open_loop_duty_percent, open_loop_run_id);

      uint32_t elapsed_samples =
          (seconds_counter + 1u) - open_loop_run_start_time_sec;
      if (elapsed_samples >= OPEN_LOOP_RUN_DURATION_SEC) {
        UART_OpenLoopMarker("DONE", seconds_counter, open_loop_duty_percent);
        Set_Fan_Speed_RegisterLevel(0);
        open_loop_started = 0;
      }

      seconds_counter++;
#else
      /* ========== 2. Hitung Output (PID Control) ========== */
      float current_setpoint = setpoint;
      uint8_t reset_pid_state = 0;
      if (flag_setpoint_changed) {
        flag_setpoint_changed = 0;
        reset_pid_state = 1;
      }
      float output = PID_ComputeCoolingOnly(&pid, current_temperature,
                                           current_setpoint, reset_pid_state);

      /* ========== 3. Set Fan Speed ========== */
      Set_Fan_Speed_RegisterLevel((uint8_t)output);

      /* ========== 4. UART telemetry log ========== */
      UART_LogTelemetry(seconds_counter, current_temperature, current_setpoint,
                        fan_speed_percent);

      seconds_counter++;
#endif
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

#if OPEN_LOOP_FIXED_TEST
static inline void OpenLoopFixed_NextDuty(void) {
  uint32_t current_time = GetTick();
  if ((current_time - last_button_time) < DEBOUNCE_MS) {
    return;
  }
  last_button_time = current_time;

  uint8_t next_idx = (uint8_t)(open_loop_duty_idx + 1u);
  if (next_idx >= OPEN_LOOP_FIXED_DUTY_STEPS_COUNT) {
    next_idx = 0;
  }

  open_loop_duty_idx = next_idx;
  open_loop_duty_percent = open_loop_fixed_duty_steps[next_idx];
  flag_duty_changed = 1;
}
#else
static inline void Handle_Setpoint_Button(uint32_t exti_pr_bit, float delta) {
  if (!(EXTI->PR & exti_pr_bit)) {
    return;
  }
  EXTI->PR = exti_pr_bit;

  if (Both_Buttons_Pressed()) {
#if OPEN_LOOP_SWEEP_TEST
    OpenLoop_RequestStart();
#endif
    return;
  }

  uint32_t current_time = GetTick();
  if ((current_time - last_button_time) < DEBOUNCE_MS) {
    return;
  }
  last_button_time = current_time;

  float new_setpoint = setpoint + delta;
  if ((new_setpoint >= SETPOINT_MIN) && (new_setpoint <= SETPOINT_MAX)) {
    setpoint = new_setpoint;
    flag_setpoint_changed = 1;
  }
}
#endif

/* EXTI4 Interrupt - Button 1 (Setpoint UP) */
void EXTI4_IRQHandler(void) {
#if OPEN_LOOP_FIXED_TEST
  if (!(EXTI->PR & EXTI_PR_PR4)) {
    return;
  }
  EXTI->PR = EXTI_PR_PR4;

  if (open_loop_started) {
    return;
  }

  OpenLoopFixed_NextDuty();
#else
  Handle_Setpoint_Button(EXTI_PR_PR4, 1.0f);
#endif
}

/* EXTI9_5 Interrupt - Button 2 (Setpoint DOWN) */
void EXTI9_5_IRQHandler(void) {
#if OPEN_LOOP_FIXED_TEST
  if (!(EXTI->PR & EXTI_PR_PR5)) {
    return;
  }
  EXTI->PR = EXTI_PR_PR5;

  if (open_loop_started) {
    return;
  }

  OpenLoop_RequestStart();
#else
  Handle_Setpoint_Button(EXTI_PR_PR5, -1.0f);
#endif
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
