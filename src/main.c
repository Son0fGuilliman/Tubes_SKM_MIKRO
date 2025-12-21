/**
 * ============================================================================
 * AUTOMATIC FAN SPEED CONTROL - TEMPERATURE BASED
 * STM32F411 (PlatformIO + STM32Cube Framework)
 * ============================================================================
 * 
 * TUGAS BESAR MIKROPROSESOR DAN ANTARMUKA
 * 
 * FITUR PERIPHERAL YANG DIGUNAKAN:
 * ┌────────────────────────────────────────────────────────────────────────┐
 * │ 2.3 TIMER DAN COUNTER                                                  │
 * │     - TIM1: PWM Generation (kontrol kecepatan kipas)                   │
 * │     - TIM2: Timer Interrupt (sampling suhu periodik 1 detik)           │
 * │     - Register: TIMx_CR1, TIMx_PSC, TIMx_ARR, TIMx_CNT, TIMx_DIER     │
 * ├────────────────────────────────────────────────────────────────────────┤
 * │ 2.4 INTERRUPT SYSTEM                                                   │
 * │     - EXTI4: External Interrupt untuk Button 1 (PB4)                   │
 * │     - EXTI5: External Interrupt untuk Button 2 (PB5)                   │
 * │     - TIM2_IRQn: Timer Interrupt untuk sampling suhu                   │
 * │     - NVIC: Priority configuration                                     │
 * │     - Register: EXTI_IMR, EXTI_RTSR, EXTI_FTSR, NVIC_ISER             │
 * └────────────────────────────────────────────────────────────────────────┘
 * 
 * PIN MAPPING:
 * - PA0  : DS18B20 Data (1-Wire) + Pull-up 4.7K ke 3.3V
 * - PA8  : PWM Output (TIM1_CH1) → 2N2222 Base (via R 1K) → Kipas 5V
 * - PA9  : UART TX → USB-TTL RX
 * - PA10 : UART RX → USB-TTL TX
 * - PB4  : Button 1 (EXTI4) → Toggle AUTO/MANUAL mode
 * - PB5  : Button 2 (EXTI5) → Adjust speed (MANUAL mode)
 * - PB0  : LED Hijau  → Suhu DINGIN (< 30°C)
 * - PB1  : LED Kuning → Suhu HANGAT (30-40°C)
 * - PB2  : LED Merah  → Suhu PANAS (> 40°C)
 * 
 * ============================================================================
 */

#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>

/* ============================================================================
 * KONFIGURASI
 * ============================================================================ */
#define PWM_PERIOD          999
#define DEBOUNCE_MS         200

/* Temperature Thresholds */
#define TEMP_FAN_MIN        25.0f
#define TEMP_FAN_MAX        45.0f
#define TEMP_COLD           30.0f
#define TEMP_HOT            40.0f

/* Fan Speed Limits */
#define FAN_SPEED_MIN       0
#define FAN_SPEED_MAX       100

/* DS18B20 Pin */
#define DS18B20_PORT        GPIOA
#define DS18B20_PIN         GPIO_PIN_0

/* DS18B20 Commands */
#define DS18B20_CMD_SKIP_ROM        0xCC
#define DS18B20_CMD_CONVERT_T       0x44
#define DS18B20_CMD_READ_SCRATCHPAD 0xBE

/* ============================================================================
 * GLOBAL VARIABLES
 * ============================================================================ */
TIM_HandleTypeDef htim1;        /* PWM Timer */
TIM_HandleTypeDef htim2;        /* Sampling Timer (1 detik interrupt) */
UART_HandleTypeDef huart1;

volatile uint8_t fan_speed_percent = 0;
volatile uint32_t last_button_time = 0;
volatile uint8_t auto_mode = 1;
volatile uint8_t flag_read_temp = 0;    /* Flag dari Timer Interrupt */

float current_temperature = 0.0f;
uint8_t sensor_ok = 0;
uint32_t seconds_counter = 0;

/* ============================================================================
 * FUNCTION PROTOTYPES
 * ============================================================================ */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_GPIO_Init_RegisterLevel(void);       /* EXTI dengan register langsung */
static void MX_TIM1_Init(void);                     /* PWM Timer */
static void MX_TIM1_Init_RegisterLevel(void);       /* PWM dengan register langsung */
static void MX_TIM2_Init_RegisterLevel(void);       /* Timer Interrupt dengan register */
static void MX_USART1_UART_Init(void);

void Set_Fan_Speed(uint8_t percent);
void Set_Fan_Speed_RegisterLevel(uint8_t percent);  /* PWM dengan register langsung */
uint8_t Calculate_Fan_Speed(float temp);
void Update_LEDs_Temperature(float temp);
void Print_Status(float temp, uint8_t speed);
void Print_Register_Status(void);                   /* Print status register */
void Error_Handler(void);

/* DS18B20 Functions */
void DWT_Init(void);
void delay_us(uint32_t us);
void DS18B20_SetPinOutput(void);
void DS18B20_SetPinInput(void);
uint8_t DS18B20_Reset(void);
void DS18B20_WriteBit(uint8_t bit);
uint8_t DS18B20_ReadBit(void);
void DS18B20_WriteByte(uint8_t byte);
uint8_t DS18B20_ReadByte(void);
float DS18B20_ReadTemperature(void);

/* ============================================================================
 * PRINTF REDIRECT TO UART
 * ============================================================================ */
#ifdef __GNUC__
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}
#endif

/* ============================================================================
 * MAIN FUNCTION
 * ============================================================================ */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    DWT_Init();
    
    /* Inisialisasi Peripheral */
    MX_GPIO_Init_RegisterLevel();       /* GPIO + EXTI dengan register */
    MX_TIM1_Init_RegisterLevel();       /* PWM dengan register */
    MX_TIM2_Init_RegisterLevel();       /* Timer Interrupt dengan register */
    MX_USART1_UART_Init();
    
    /* Set speed awal 0% */
    Set_Fan_Speed_RegisterLevel(0);
    
    /* ==================== WELCOME MESSAGE ==================== */
    printf("\r\n");
    printf("╔══════════════════════════════════════════════════════════════╗\r\n");
    printf("║   AUTOMATIC FAN SPEED CONTROL - TEMPERATURE BASED            ║\r\n");
    printf("║   TUGAS BESAR MIKROPROSESOR DAN ANTARMUKA                    ║\r\n");
    printf("║   STM32F411 Black Pill                                       ║\r\n");
    printf("╚══════════════════════════════════════════════════════════════╝\r\n");
    printf("\r\n");
    
    /* Print Register Configuration */
    Print_Register_Status();
    
    /* Test LED */
    printf("[BOOT] Testing LEDs...\r\n");
    GPIOB->BSRR = GPIO_PIN_0;   /* LED Hijau ON (register langsung) */
    HAL_Delay(200);
    GPIOB->BSRR = GPIO_PIN_0 << 16;  /* LED Hijau OFF */
    GPIOB->BSRR = GPIO_PIN_1;   /* LED Kuning ON */
    HAL_Delay(200);
    GPIOB->BSRR = GPIO_PIN_1 << 16;
    GPIOB->BSRR = GPIO_PIN_2;   /* LED Merah ON */
    HAL_Delay(200);
    GPIOB->BSRR = GPIO_PIN_2 << 16;
    printf("[BOOT] LED test OK\r\n");
    
    /* Check DS18B20 */
    printf("[BOOT] Detecting DS18B20 sensor...\r\n");
    sensor_ok = DS18B20_Reset();
    
    if (!sensor_ok)
    {
        printf("\r\n[ERROR] DS18B20 NOT DETECTED!\r\n");
        printf("Check: DQ -> PA0, VDD -> 3.3V, GND -> GND, 4.7K pull-up\r\n");
        
        while (1)
        {
            GPIOB->ODR ^= GPIO_PIN_2;   /* Toggle LED Merah (register) */
            HAL_Delay(100);
        }
    }
    
    printf("[BOOT] DS18B20 detected OK!\r\n\r\n");
    
    /* Print Configuration */
    printf("┌────────────────────────────────────────────────────────────┐\r\n");
    printf("│  PERIPHERAL CONFIGURATION                                  │\r\n");
    printf("├────────────────────────────────────────────────────────────┤\r\n");
    printf("│  TIM1 (PWM):                                               │\r\n");
    printf("│    - Channel 1 (PA8) untuk kontrol kipas                   │\r\n");
    printf("│    - Frequency: 1 kHz (PSC=99, ARR=999)                    │\r\n");
    printf("│  TIM2 (Timer Interrupt):                                   │\r\n");
    printf("│    - Update Interrupt setiap 1 detik                       │\r\n");
    printf("│    - Trigger pembacaan suhu DS18B20                        │\r\n");
    printf("│  EXTI (External Interrupt):                                │\r\n");
    printf("│    - EXTI4 (PB4): Toggle AUTO/MANUAL mode                  │\r\n");
    printf("│    - EXTI5 (PB5): Adjust speed (MANUAL mode)               │\r\n");
    printf("└────────────────────────────────────────────────────────────┘\r\n");
    printf("\r\n");
    
    printf(">>> Starting in AUTO mode...\r\n\r\n");
    printf("  TIME(s) │ TEMP(°C) │ FAN(%%) │ MODE   │ STATUS\r\n");
    printf("──────────┼──────────┼────────┼────────┼────────\r\n");
    
    /* Baca suhu pertama kali */
    current_temperature = DS18B20_ReadTemperature();
    
    /* ==================== MAIN LOOP ==================== */
    while (1)
    {
        /* Cek flag dari Timer Interrupt (setiap 1 detik) */
        if (flag_read_temp)
        {
            flag_read_temp = 0;     /* Reset flag */
            
            /* Baca suhu dari DS18B20 */
            current_temperature = DS18B20_ReadTemperature();
            
            /* Hitung dan set fan speed (AUTO mode) */
            if (auto_mode)
            {
                uint8_t calculated_speed = Calculate_Fan_Speed(current_temperature);
                Set_Fan_Speed_RegisterLevel(calculated_speed);
            }
            
            /* Print status */
            Print_Status(current_temperature, fan_speed_percent);
            seconds_counter++;
        }
        
        /* Update LED */
        Update_LEDs_Temperature(current_temperature);
        
        HAL_Delay(10);
    }
}

/* ============================================================================
 * PRINT REGISTER STATUS - Untuk dokumentasi laporan
 * ============================================================================ */
void Print_Register_Status(void)
{
    printf("┌────────────────────────────────────────────────────────────┐\r\n");
    printf("│  REGISTER STATUS (Untuk Laporan)                           │\r\n");
    printf("├────────────────────────────────────────────────────────────┤\r\n");
    printf("│  TIM1 (PWM Generator):                                     │\r\n");
    printf("│    TIM1->CR1  = 0x%04lX  (Control Register 1)              │\r\n", TIM1->CR1);
    printf("│    TIM1->PSC  = %4lu    (Prescaler)                        │\r\n", TIM1->PSC);
    printf("│    TIM1->ARR  = %4lu    (Auto-Reload Register)             │\r\n", TIM1->ARR);
    printf("│    TIM1->CCR1 = %4lu    (Capture/Compare Register 1)       │\r\n", TIM1->CCR1);
    printf("├────────────────────────────────────────────────────────────┤\r\n");
    printf("│  TIM2 (Timer Interrupt):                                   │\r\n");
    printf("│    TIM2->CR1  = 0x%04lX  (Control Register 1)              │\r\n", TIM2->CR1);
    printf("│    TIM2->PSC  = %4lu    (Prescaler)                        │\r\n", TIM2->PSC);
    printf("│    TIM2->ARR  = %4lu    (Auto-Reload Register)             │\r\n", TIM2->ARR);
    printf("│    TIM2->DIER = 0x%04lX  (DMA/Interrupt Enable)            │\r\n", TIM2->DIER);
    printf("├────────────────────────────────────────────────────────────┤\r\n");
    printf("│  EXTI (External Interrupt):                                │\r\n");
    printf("│    EXTI->IMR  = 0x%08lX  (Interrupt Mask Register)       │\r\n", EXTI->IMR);
    printf("│    EXTI->RTSR = 0x%08lX  (Rising Trigger Selection)      │\r\n", EXTI->RTSR);
    printf("│    EXTI->FTSR = 0x%08lX  (Falling Trigger Selection)     │\r\n", EXTI->FTSR);
    printf("├────────────────────────────────────────────────────────────┤\r\n");
    printf("│  NVIC (Interrupt Controller):                              │\r\n");
    printf("│    NVIC_ISER[0] = 0x%08lX  (Interrupt Set-Enable)        │\r\n", NVIC->ISER[0]);
    printf("└────────────────────────────────────────────────────────────┘\r\n");
    printf("\r\n");
}

/* ============================================================================
 * GPIO + EXTI INIT - REGISTER LEVEL
 * ============================================================================
 * 
 * Register yang digunakan:
 * - RCC->AHB1ENR   : Enable clock GPIO
 * - GPIOx->MODER   : Mode register (Input/Output/AF/Analog)
 * - GPIOx->OTYPER  : Output type (Push-pull/Open-drain)
 * - GPIOx->PUPDR   : Pull-up/Pull-down
 * - GPIOx->BSRR    : Bit set/reset register
 * - SYSCFG->EXTICR : EXTI configuration
 * - EXTI->IMR      : Interrupt mask register
 * - EXTI->FTSR     : Falling trigger selection
 * - NVIC_ISER      : Interrupt set-enable register
 * 
 * ============================================================================ */
static void MX_GPIO_Init_RegisterLevel(void)
{
    /* ========== Enable Clock untuk GPIO ========== */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;    /* Enable GPIOA clock */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;    /* Enable GPIOB clock */
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;   /* Enable SYSCFG clock untuk EXTI */
    
    /* ========== DS18B20 (PA0) - Open Drain Output ========== */
    GPIOA->MODER &= ~GPIO_MODER_MODER0;         /* Clear mode bits */
    GPIOA->MODER |= GPIO_MODER_MODER0_0;        /* Output mode (01) */
    GPIOA->OTYPER |= GPIO_OTYPER_OT0;           /* Open-drain */
    GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED0;     /* High speed */
    GPIOA->BSRR = GPIO_PIN_0;                   /* Set HIGH (idle state) */
    
    /* ========== LED Outputs (PB0, PB1, PB2) - Push-Pull ========== */
    /* PB0 - LED Hijau */
    GPIOB->MODER &= ~GPIO_MODER_MODER0;
    GPIOB->MODER |= GPIO_MODER_MODER0_0;        /* Output mode */
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT0;          /* Push-pull */
    
    /* PB1 - LED Kuning */
    GPIOB->MODER &= ~GPIO_MODER_MODER1;
    GPIOB->MODER |= GPIO_MODER_MODER1_0;
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT1;
    
    /* PB2 - LED Merah */
    GPIOB->MODER &= ~GPIO_MODER_MODER2;
    GPIOB->MODER |= GPIO_MODER_MODER2_0;
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT2;
    
    /* Matikan semua LED awal */
    GPIOB->BSRR = (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2) << 16;
    
    /* ========== Button Inputs (PB4, PB5) dengan EXTI ========== */
    /* PB4 - Button 1 (Input dengan Pull-up) */
    GPIOB->MODER &= ~GPIO_MODER_MODER4;         /* Input mode (00) */
    GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD4;
    GPIOB->PUPDR |= GPIO_PUPDR_PUPD4_0;         /* Pull-up (01) */
    
    /* PB5 - Button 2 (Input dengan Pull-up) */
    GPIOB->MODER &= ~GPIO_MODER_MODER5;
    GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD5;
    GPIOB->PUPDR |= GPIO_PUPDR_PUPD5_0;
    
    /* ========== EXTI Configuration ========== */
    /* 
     * EXTI_IMR  : Interrupt Mask Register - enable interrupt line
     * EXTI_FTSR : Falling Trigger Selection Register - trigger pada falling edge
     * SYSCFG_EXTICR : Pilih port mana yang connect ke EXTI line
     */
    
    /* Connect PB4 ke EXTI4 */
    SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR2_EXTI4;
    SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PB;   /* PB4 -> EXTI4 */
    
    /* Connect PB5 ke EXTI5 */
    SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR2_EXTI5;
    SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI5_PB;   /* PB5 -> EXTI5 */
    
    /* Enable EXTI line 4 dan 5 */
    EXTI->IMR |= EXTI_IMR_IM4;      /* Unmask EXTI4 */
    EXTI->IMR |= EXTI_IMR_IM5;      /* Unmask EXTI5 */
    
    /* Trigger pada Falling Edge (tombol ditekan = LOW) */
    EXTI->FTSR |= EXTI_FTSR_TR4;    /* Falling trigger untuk EXTI4 */
    EXTI->FTSR |= EXTI_FTSR_TR5;    /* Falling trigger untuk EXTI5 */
    
    /* ========== NVIC Configuration ========== */
    /* 
     * NVIC_ISER : Interrupt Set-Enable Register
     * NVIC_IPR  : Interrupt Priority Register
     */
    
    /* Enable EXTI4 interrupt di NVIC */
    NVIC_SetPriority(EXTI4_IRQn, 2);
    NVIC->ISER[0] |= (1 << EXTI4_IRQn);     /* Enable EXTI4_IRQn */
    
    /* Enable EXTI9_5 interrupt di NVIC (EXTI5 ada di EXTI9_5) */
    NVIC_SetPriority(EXTI9_5_IRQn, 2);
    NVIC->ISER[0] |= (1 << EXTI9_5_IRQn);   /* Enable EXTI9_5_IRQn */
}

/* ============================================================================
 * TIM1 INIT (PWM) - REGISTER LEVEL
 * ============================================================================
 * 
 * PWM Frequency = TIM_CLK / ((PSC + 1) * (ARR + 1))
 *               = 100MHz / ((99 + 1) * (999 + 1))
 *               = 100MHz / 100000 = 1 kHz
 * 
 * Register yang digunakan:
 * - TIM1->CR1   : Control Register 1 (enable timer)
 * - TIM1->PSC   : Prescaler (pembagi clock)
 * - TIM1->ARR   : Auto-Reload Register (period)
 * - TIM1->CCR1  : Capture/Compare Register 1 (duty cycle)
 * - TIM1->CCMR1 : Capture/Compare Mode Register (PWM mode)
 * - TIM1->CCER  : Capture/Compare Enable Register
 * - TIM1->BDTR  : Break and Dead-Time Register (untuk Advanced Timer)
 * 
 * ============================================================================ */
static void MX_TIM1_Init_RegisterLevel(void)
{
    /* Enable TIM1 clock */
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    
    /* Configure PA8 sebagai TIM1_CH1 (Alternate Function) */
    GPIOA->MODER &= ~GPIO_MODER_MODER8;
    GPIOA->MODER |= GPIO_MODER_MODER8_1;        /* Alternate Function (10) */
    GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL8;
    GPIOA->AFR[1] |= (1 << GPIO_AFRH_AFSEL8_Pos);  /* AF1 = TIM1_CH1 */
    
    /* Reset TIM1 */
    TIM1->CR1 = 0;
    TIM1->CR2 = 0;
    
    /* Set Prescaler: 100MHz / 100 = 1MHz timer clock */
    TIM1->PSC = 99;
    
    /* Set Auto-Reload: 1MHz / 1000 = 1kHz PWM frequency */
    TIM1->ARR = PWM_PERIOD;
    
    /* Set initial duty cycle = 0 */
    TIM1->CCR1 = 0;
    
    /* Configure PWM Mode 1 pada Channel 1 */
    /* CCMR1: OC1M = 110 (PWM Mode 1), OC1PE = 1 (Preload enable) */
    TIM1->CCMR1 &= ~TIM_CCMR1_OC1M;
    TIM1->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1);  /* PWM Mode 1 */
    TIM1->CCMR1 |= TIM_CCMR1_OC1PE;     /* Output Compare Preload Enable */
    
    /* Enable Channel 1 output */
    TIM1->CCER |= TIM_CCER_CC1E;        /* Capture/Compare 1 output enable */
    
    /* TIM1 adalah Advanced Timer, perlu enable MOE (Main Output Enable) */
    TIM1->BDTR |= TIM_BDTR_MOE;         /* Main Output Enable */
    
    /* Enable Counter */
    TIM1->CR1 |= TIM_CR1_CEN;           /* Counter Enable */
}

/* ============================================================================
 * TIM2 INIT (TIMER INTERRUPT) - REGISTER LEVEL
 * ============================================================================
 * 
 * Timer Interrupt setiap 1 detik untuk trigger pembacaan suhu
 * 
 * Frequency = TIM_CLK / ((PSC + 1) * (ARR + 1))
 *           = 50MHz / ((9999 + 1) * (4999 + 1))
 *           = 50MHz / 50000000 = 1 Hz (1 detik)
 * 
 * Note: TIM2 ada di APB1 (50MHz setelah APB1 prescaler /2)
 * 
 * Register yang digunakan:
 * - TIM2->CR1  : Control Register 1
 * - TIM2->PSC  : Prescaler
 * - TIM2->ARR  : Auto-Reload Register
 * - TIM2->DIER : DMA/Interrupt Enable Register
 * - TIM2->SR   : Status Register (untuk clear flag)
 * - TIM2->CNT  : Counter Register
 * 
 * ============================================================================ */
static void MX_TIM2_Init_RegisterLevel(void)
{
    /* Enable TIM2 clock */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    
    /* Reset TIM2 */
    TIM2->CR1 = 0;
    TIM2->CR2 = 0;
    
    /* Set Prescaler: 50MHz / 10000 = 5kHz */
    TIM2->PSC = 9999;
    
    /* Set Auto-Reload: 5kHz / 5000 = 1Hz (1 detik) */
    TIM2->ARR = 4999;
    
    /* Reset counter */
    TIM2->CNT = 0;
    
    /* Enable Update Interrupt */
    /* DIER: UIE = 1 (Update Interrupt Enable) */
    TIM2->DIER |= TIM_DIER_UIE;
    
    /* Clear any pending interrupt flag */
    TIM2->SR &= ~TIM_SR_UIF;
    
    /* Enable TIM2 interrupt di NVIC */
    NVIC_SetPriority(TIM2_IRQn, 1);     /* Priority lebih tinggi dari EXTI */
    NVIC->ISER[0] |= (1 << TIM2_IRQn);
    
    /* Enable Counter */
    TIM2->CR1 |= TIM_CR1_CEN;
}

/* ============================================================================
 * SET FAN SPEED - REGISTER LEVEL
 * ============================================================================ */
void Set_Fan_Speed_RegisterLevel(uint8_t percent)
{
    if (percent > 100) percent = 100;
    
    fan_speed_percent = percent;
    
    /* Hitung nilai CCR1 untuk duty cycle */
    uint32_t ccr_value = (uint32_t)((percent * PWM_PERIOD) / 100);
    
    /* Set TIM1->CCR1 langsung */
    TIM1->CCR1 = ccr_value;
}

/* Wrapper function */
void Set_Fan_Speed(uint8_t percent)
{
    Set_Fan_Speed_RegisterLevel(percent);
}

/* ============================================================================
 * CALCULATE FAN SPEED - Linear Mapping
 * ============================================================================ */
uint8_t Calculate_Fan_Speed(float temp)
{
    if (temp <= TEMP_FAN_MIN) return FAN_SPEED_MIN;
    if (temp >= TEMP_FAN_MAX) return FAN_SPEED_MAX;
    
    float temp_range = TEMP_FAN_MAX - TEMP_FAN_MIN;
    float speed_range = FAN_SPEED_MAX - FAN_SPEED_MIN;
    float speed = ((temp - TEMP_FAN_MIN) / temp_range) * speed_range + FAN_SPEED_MIN;
    
    return (uint8_t)speed;
}

/* ============================================================================
 * PRINT STATUS
 * ============================================================================ */
void Print_Status(float temp, uint8_t speed)
{
    char status[10];
    char mode[8];
    
    if (temp < TEMP_COLD) strcpy(status, "COLD");
    else if (temp < TEMP_HOT) strcpy(status, "WARM");
    else strcpy(status, "HOT!");
    
    strcpy(mode, auto_mode ? "AUTO" : "MANUAL");
    
    printf("  %7lu │  %6.2f  │  %3u   │ %-6s │ %s\r\n", 
           seconds_counter, temp, speed, mode, status);
}

/* ============================================================================
 * UPDATE LEDs - Menggunakan Register BSRR
 * ============================================================================ */
void Update_LEDs_Temperature(float temp)
{
    /* Matikan semua LED dulu menggunakan BSRR (bit 16-31 untuk reset) */
    GPIOB->BSRR = (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2) << 16;
    
    /* Nyalakan LED sesuai suhu menggunakan BSRR (bit 0-15 untuk set) */
    if (temp < TEMP_COLD)
    {
        GPIOB->BSRR = GPIO_PIN_0;       /* LED Hijau ON */
    }
    else if (temp < TEMP_HOT)
    {
        GPIOB->BSRR = GPIO_PIN_1;       /* LED Kuning ON */
    }
    else
    {
        GPIOB->BSRR = GPIO_PIN_2;       /* LED Merah ON */
    }
}

/* ============================================================================
 * TIM2 INTERRUPT HANDLER - Setiap 1 detik
 * ============================================================================ */
void TIM2_IRQHandler(void)
{
    /* Check Update Interrupt Flag */
    if (TIM2->SR & TIM_SR_UIF)
    {
        /* Clear flag dengan menulis 0 ke UIF */
        TIM2->SR &= ~TIM_SR_UIF;
        
        /* Set flag untuk main loop */
        flag_read_temp = 1;
    }
}

/* ============================================================================
 * EXTI INTERRUPT HANDLERS
 * ============================================================================ */
void EXTI4_IRQHandler(void)
{
    /* Check Pending Register */
    if (EXTI->PR & EXTI_PR_PR4)
    {
        /* Clear pending bit dengan menulis 1 */
        EXTI->PR = EXTI_PR_PR4;
        
        /* Debouncing */
        uint32_t current_time = HAL_GetTick();
        if ((current_time - last_button_time) < DEBOUNCE_MS) return;
        last_button_time = current_time;
        
        /* Toggle AUTO/MANUAL mode */
        auto_mode = !auto_mode;
        
        printf("\r\n>>> Switched to %s mode\r\n\r\n", auto_mode ? "AUTO" : "MANUAL");
        printf("  TIME(s) │ TEMP(°C) │ FAN(%%) │ MODE   │ STATUS\r\n");
        printf("──────────┼──────────┼────────┼────────┼────────\r\n");
    }
}

void EXTI9_5_IRQHandler(void)
{
    /* Check Pending Register untuk EXTI5 */
    if (EXTI->PR & EXTI_PR_PR5)
    {
        /* Clear pending bit */
        EXTI->PR = EXTI_PR_PR5;
        
        /* Debouncing */
        uint32_t current_time = HAL_GetTick();
        if ((current_time - last_button_time) < DEBOUNCE_MS) return;
        last_button_time = current_time;
        
        /* Adjust speed (hanya di MANUAL mode) */
        if (!auto_mode)
        {
            if (fan_speed_percent < 25) Set_Fan_Speed_RegisterLevel(25);
            else if (fan_speed_percent < 50) Set_Fan_Speed_RegisterLevel(50);
            else if (fan_speed_percent < 75) Set_Fan_Speed_RegisterLevel(75);
            else if (fan_speed_percent < 100) Set_Fan_Speed_RegisterLevel(100);
            else Set_Fan_Speed_RegisterLevel(0);
            
            printf("\r\n>>> [MANUAL] Fan speed: %u%%\r\n\r\n", fan_speed_percent);
        }
    }
}

/* ============================================================================
 * DWT INIT
 * ============================================================================ */
void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(uint32_t us)
{
    uint32_t startTick = DWT->CYCCNT;
    uint32_t delayTicks = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - startTick) < delayTicks);
}

/* ============================================================================
 * DS18B20 FUNCTIONS
 * ============================================================================ */
void DS18B20_SetPinOutput(void)
{
    GPIOA->MODER &= ~GPIO_MODER_MODER0;
    GPIOA->MODER |= GPIO_MODER_MODER0_0;    /* Output */
}

void DS18B20_SetPinInput(void)
{
    GPIOA->MODER &= ~GPIO_MODER_MODER0;     /* Input (00) */
}

uint8_t DS18B20_Reset(void)
{
    uint8_t presence = 0;
    
    DS18B20_SetPinOutput();
    GPIOA->BSRR = GPIO_PIN_0 << 16;     /* LOW */
    delay_us(480);
    
    DS18B20_SetPinInput();
    delay_us(70);
    
    if (!(GPIOA->IDR & GPIO_PIN_0)) presence = 1;
    
    delay_us(410);
    return presence;
}

void DS18B20_WriteBit(uint8_t bit)
{
    DS18B20_SetPinOutput();
    GPIOA->BSRR = GPIO_PIN_0 << 16;     /* LOW */
    
    if (bit)
    {
        delay_us(1);
        DS18B20_SetPinInput();
        delay_us(60);
    }
    else
    {
        delay_us(60);
        DS18B20_SetPinInput();
        delay_us(1);
    }
}

uint8_t DS18B20_ReadBit(void)
{
    uint8_t bit = 0;
    
    DS18B20_SetPinOutput();
    GPIOA->BSRR = GPIO_PIN_0 << 16;
    delay_us(2);
    
    DS18B20_SetPinInput();
    delay_us(10);
    
    if (GPIOA->IDR & GPIO_PIN_0) bit = 1;
    
    delay_us(50);
    return bit;
}

void DS18B20_WriteByte(uint8_t byte)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        DS18B20_WriteBit(byte & 0x01);
        byte >>= 1;
    }
}

uint8_t DS18B20_ReadByte(void)
{
    uint8_t byte = 0;
    for (uint8_t i = 0; i < 8; i++)
    {
        byte >>= 1;
        if (DS18B20_ReadBit()) byte |= 0x80;
    }
    return byte;
}

float DS18B20_ReadTemperature(void)
{
    uint8_t temp_lsb, temp_msb;
    int16_t temp_raw;
    
    if (!DS18B20_Reset()) return -999.0f;
    DS18B20_WriteByte(DS18B20_CMD_SKIP_ROM);
    DS18B20_WriteByte(DS18B20_CMD_CONVERT_T);
    
    HAL_Delay(750);
    
    if (!DS18B20_Reset()) return -999.0f;
    DS18B20_WriteByte(DS18B20_CMD_SKIP_ROM);
    DS18B20_WriteByte(DS18B20_CMD_READ_SCRATCHPAD);
    
    temp_lsb = DS18B20_ReadByte();
    temp_msb = DS18B20_ReadByte();
    
    temp_raw = (temp_msb << 8) | temp_lsb;
    return (float)temp_raw / 16.0f;
}

/* ============================================================================
 * SYSTEM CLOCK CONFIG
 * ============================================================================ */
void SystemClock_Config(void)
{
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
    
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
    
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) Error_Handler();
}

/* ============================================================================
 * USART1 INIT
 * ============================================================================ */
static void MX_USART1_UART_Init(void)
{
    __HAL_RCC_USART1_CLK_ENABLE();
    
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    
    if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
}

/* Dummy GPIO Init untuk kompatibilitas */
static void MX_GPIO_Init(void) {}
static void MX_TIM1_Init(void) {}

/* ============================================================================
 * ERROR HANDLER
 * ============================================================================ */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        GPIOB->ODR ^= GPIO_PIN_2;
        for (volatile int i = 0; i < 500000; i++);
    }
}

/* ============================================================================
 * STANDARD INTERRUPT HANDLERS
 * ============================================================================ */
void SysTick_Handler(void)     { HAL_IncTick(); }
void NMI_Handler(void)         { while(1); }
void HardFault_Handler(void)   { while(1); }
void MemManage_Handler(void)   { while(1); }
void BusFault_Handler(void)    { while(1); }
void UsageFault_Handler(void)  { while(1); }
void SVC_Handler(void)         {}
void DebugMon_Handler(void)    {}
void PendSV_Handler(void)      {}