/**
 * ============================================================================
 * AUTOMATIC FAN SPEED CONTROL - TEMPERATURE BASED
 * STM32F411 (PlatformIO + STM32Cube Framework)
 * ============================================================================
 * 
 * TUGAS BESAR MIKROPROSESOR DAN ANTARMUKA
 * 
 * ╔════════════════════════════════════════════════════════════════════════════╗
 * ║  PERIPHERAL YANG DIGUNAKAN (Register Level)                                ║
 * ╠════════════════════════════════════════════════════════════════════════════╣
 * ║  2.2 SERIAL COMMUNICATION (UART)                                           ║
 * ║      - USART1 untuk Serial Monitor                                         ║
 * ║      - Baud Rate: 115200 (calculated dari APB2 clock)                      ║
 * ║      - Frame: 8 data bits, No parity, 1 stop bit (8N1)                     ║
 * ║      - Register: USART_CR1, USART_CR2, USART_BRR, USART_SR, USART_DR      ║
 * ╠════════════════════════════════════════════════════════════════════════════╣
 * ║  2.3 TIMER DAN COUNTER                                                     ║
 * ║      - TIM1: PWM Generation (kontrol kecepatan kipas)                      ║
 * ║      - TIM2: Timer Interrupt (sampling suhu periodik 1 detik)              ║
 * ║      - Register: TIMx_CR1, TIMx_PSC, TIMx_ARR, TIMx_CNT, TIMx_DIER        ║
 * ║      - Register: TIMx_CCMR1, TIMx_CCER, TIMx_CCR1, TIMx_BDTR              ║
 * ╠════════════════════════════════════════════════════════════════════════════╣
 * ║  2.4 INTERRUPT SYSTEM                                                      ║
 * ║      - EXTI4: External Interrupt untuk Button 1 (PB4)                      ║
 * ║      - EXTI5: External Interrupt untuk Button 2 (PB5)                      ║
 * ║      - TIM2_IRQn: Timer Interrupt untuk sampling suhu                      ║
 * ║      - Register: EXTI_IMR, EXTI_RTSR, EXTI_FTSR, EXTI_PR                  ║
 * ║      - Register: NVIC_ISER, NVIC_IPR                                       ║
 * ╠════════════════════════════════════════════════════════════════════════════╣
 * ║  2.5 GPIO CONFIGURATION                                                    ║
 * ║      - Input: Pull-up (Buttons PB4, PB5)                                   ║
 * ║      - Output Push-Pull: LEDs (PB0, PB1, PB2)                              ║
 * ║      - Output Open-Drain: DS18B20 1-Wire (PA0)                             ║
 * ║      - Alternate Function: UART TX/RX (PA9, PA10), PWM (PA8)               ║
 * ║      - Register: GPIOx_MODER, GPIOx_OTYPER, GPIOx_OSPEEDR                 ║
 * ║      - Register: GPIOx_PUPDR, GPIOx_IDR, GPIOx_ODR, GPIOx_BSRR, GPIOx_AFR ║
 * ╚════════════════════════════════════════════════════════════════════════════╝
 * 
 * PIN MAPPING:
 * - PA0  : DS18B20 Data (1-Wire, Open-Drain) + Pull-up 4.7K ke 3.3V
 * - PA8  : PWM Output (TIM1_CH1, AF1) → 2N2222 Base (via R 1K) → Kipas 5V
 * - PA9  : UART TX (USART1, AF7) → USB-TTL RX
 * - PA10 : UART RX (USART1, AF7) → USB-TTL TX
 * - PB4  : Button 1 (EXTI4, Input Pull-up) → Toggle AUTO/MANUAL mode
 * - PB5  : Button 2 (EXTI5, Input Pull-up) → Adjust speed (MANUAL mode)
 * - PB0  : LED Hijau  (Output Push-Pull) → Suhu DINGIN (< 30°C)
 * - PB1  : LED Kuning (Output Push-Pull) → Suhu HANGAT (30-40°C)
 * - PB2  : LED Merah  (Output Push-Pull) → Suhu PANAS (> 40°C)
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

/* UART Configuration */
#define UART_BAUD_RATE      115200
#define APB2_CLOCK          100000000   /* 100 MHz */

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
volatile uint8_t fan_speed_percent = 0;
volatile uint32_t last_button_time = 0;
volatile uint8_t auto_mode = 1;
volatile uint8_t flag_read_temp = 0;

float current_temperature = 0.0f;
uint8_t sensor_ok = 0;
uint32_t seconds_counter = 0;

/* ============================================================================
 * FUNCTION PROTOTYPES
 * ============================================================================ */
void SystemClock_Config(void);
static void MX_GPIO_Init_RegisterLevel(void);
static void MX_TIM1_PWM_Init_RegisterLevel(void);
static void MX_TIM2_Interrupt_Init_RegisterLevel(void);
static void MX_USART1_Init_RegisterLevel(void);

void UART_SendChar(char c);
void UART_SendString(const char *str);
void UART_SendFloat(float value, uint8_t decimals);
void UART_SendInt(int32_t value);

void Set_Fan_Speed_RegisterLevel(uint8_t percent);
uint8_t Calculate_Fan_Speed(float temp);
void Update_LEDs_Temperature(float temp);
void Print_Status(float temp, uint8_t speed);
void Print_All_Register_Status(void);
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
 * UART FUNCTIONS - REGISTER LEVEL
 * ============================================================================
 * 
 * Register yang digunakan:
 * - USART_SR  : Status Register (TXE, RXNE, TC flags)
 * - USART_DR  : Data Register (baca/tulis data)
 * - USART_BRR : Baud Rate Register
 * - USART_CR1 : Control Register 1 (UE, TE, RE, etc)
 * - USART_CR2 : Control Register 2 (Stop bits)
 * - USART_CR3 : Control Register 3 (Flow control)
 * 
 * ============================================================================ */

/* Kirim 1 karakter via UART (Register Level) */
void UART_SendChar(char c)
{
    /* Tunggu sampai TXE (Transmit Data Register Empty) = 1 */
    while (!(USART1->SR & USART_SR_TXE));
    
    /* Tulis data ke Data Register */
    USART1->DR = c;
}

/* Kirim string via UART */
void UART_SendString(const char *str)
{
    while (*str)
    {
        UART_SendChar(*str++);
    }
}

/* Kirim integer via UART */
void UART_SendInt(int32_t value)
{
    char buffer[12];
    int i = 0;
    uint8_t negative = 0;
    
    if (value < 0)
    {
        negative = 1;
        value = -value;
    }
    
    if (value == 0)
    {
        UART_SendChar('0');
        return;
    }
    
    while (value > 0)
    {
        buffer[i++] = '0' + (value % 10);
        value /= 10;
    }
    
    if (negative) UART_SendChar('-');
    
    while (i > 0)
    {
        UART_SendChar(buffer[--i]);
    }
}

/* Kirim float via UART */
void UART_SendFloat(float value, uint8_t decimals)
{
    if (value < 0)
    {
        UART_SendChar('-');
        value = -value;
    }
    
    int32_t int_part = (int32_t)value;
    UART_SendInt(int_part);
    
    UART_SendChar('.');
    
    float frac = value - int_part;
    for (uint8_t i = 0; i < decimals; i++)
    {
        frac *= 10;
        UART_SendChar('0' + (int)frac % 10);
    }
}

/* Printf redirect ke UART register level */
#ifdef __GNUC__
int __io_putchar(int ch)
{
    UART_SendChar(ch);
    return ch;
}

int _write(int file, char *ptr, int len)
{
    for (int i = 0; i < len; i++)
    {
        UART_SendChar(ptr[i]);
    }
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
    
    /* Inisialisasi Peripheral - SEMUA REGISTER LEVEL */
    MX_GPIO_Init_RegisterLevel();
    MX_USART1_Init_RegisterLevel();
    MX_TIM1_PWM_Init_RegisterLevel();
    MX_TIM2_Interrupt_Init_RegisterLevel();
    
    /* Set speed awal 0% */
    Set_Fan_Speed_RegisterLevel(0);
    
    /* ==================== WELCOME MESSAGE ==================== */
    UART_SendString("\r\n");
    UART_SendString("================================================================\r\n");
    UART_SendString("   AUTOMATIC FAN SPEED CONTROL - TEMPERATURE BASED\r\n");
    UART_SendString("   TUGAS BESAR MIKROPROSESOR DAN ANTARMUKA\r\n");
    UART_SendString("   STM32F411 Black Pill - Register Level Programming\r\n");
    UART_SendString("================================================================\r\n");
    UART_SendString("\r\n");
    
    /* Print Register Configuration */
    Print_All_Register_Status();
    
    /* Test LED */
    UART_SendString("[BOOT] Testing LEDs...\r\n");
    GPIOB->BSRR = GPIO_PIN_0;           /* LED Hijau ON */
    HAL_Delay(200);
    GPIOB->BSRR = GPIO_PIN_0 << 16;     /* LED Hijau OFF */
    GPIOB->BSRR = GPIO_PIN_1;           /* LED Kuning ON */
    HAL_Delay(200);
    GPIOB->BSRR = GPIO_PIN_1 << 16;
    GPIOB->BSRR = GPIO_PIN_2;           /* LED Merah ON */
    HAL_Delay(200);
    GPIOB->BSRR = GPIO_PIN_2 << 16;
    UART_SendString("[BOOT] LED test OK\r\n");
    
    /* Check DS18B20 */
    UART_SendString("[BOOT] Detecting DS18B20 sensor...\r\n");
    sensor_ok = DS18B20_Reset();
    
    if (!sensor_ok)
    {
        UART_SendString("\r\n[ERROR] DS18B20 NOT DETECTED!\r\n");
        UART_SendString("Check: DQ -> PA0, VDD -> 3.3V, GND -> GND, 4.7K pull-up\r\n");
        
        while (1)
        {
            GPIOB->ODR ^= GPIO_PIN_2;
            HAL_Delay(100);
        }
    }
    
    UART_SendString("[BOOT] DS18B20 detected OK!\r\n\r\n");
    
    /* Print Control Info */
    UART_SendString("CONTROLS:\r\n");
    UART_SendString("  BTN1 (PB4): Toggle AUTO/MANUAL mode\r\n");
    UART_SendString("  BTN2 (PB5): Adjust speed (MANUAL mode)\r\n\r\n");
    
    UART_SendString("FAN CONTROL (AUTO mode):\r\n");
    UART_SendString("  Temp <= 25C  -> Fan OFF\r\n");
    UART_SendString("  Temp 25-45C  -> Fan 0-100% (linear)\r\n");
    UART_SendString("  Temp >= 45C  -> Fan FULL\r\n\r\n");
    
    UART_SendString(">>> Starting in AUTO mode...\r\n\r\n");
    UART_SendString("  TIME(s) | TEMP(C) | FAN(%) | MODE   | STATUS\r\n");
    UART_SendString("----------|---------|--------|--------|--------\r\n");
    
    /* Baca suhu pertama kali */
    current_temperature = DS18B20_ReadTemperature();
    
    /* ==================== MAIN LOOP ==================== */
    while (1)
    {
        /* Cek flag dari Timer Interrupt (setiap 1 detik) */
        if (flag_read_temp)
        {
            flag_read_temp = 0;
            
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
 * PRINT ALL REGISTER STATUS - UNTUK LAPORAN
 * ============================================================================ */
void Print_All_Register_Status(void)
{
    UART_SendString("================================================================\r\n");
    UART_SendString("                    REGISTER STATUS\r\n");
    UART_SendString("================================================================\r\n");
    UART_SendString("\r\n");
    
    /* ===== 2.2 UART REGISTERS ===== */
    UART_SendString("[ 2.2 SERIAL COMMUNICATION - USART1 ]\r\n");
    UART_SendString("------------------------------------------------\r\n");
    
    UART_SendString("  USART1->BRR  = 0x");
    /* Print hex value */
    uint32_t brr = USART1->BRR;
    char hex[9];
    for (int i = 7; i >= 0; i--) {
        uint8_t nibble = (brr >> (i * 4)) & 0xF;
        hex[7-i] = nibble < 10 ? '0' + nibble : 'A' + nibble - 10;
    }
    hex[8] = '\0';
    UART_SendString(hex);
    UART_SendString("  (Baud Rate Register)\r\n");
    
    UART_SendString("    - Mantissa: ");
    UART_SendInt((USART1->BRR >> 4) & 0xFFF);
    UART_SendString(", Fraction: ");
    UART_SendInt(USART1->BRR & 0xF);
    UART_SendString("\r\n");
    
    UART_SendString("    - Calculated Baud: ");
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
    UART_SendString("  (Control Register 1)\r\n");
    UART_SendString("    - UE (USART Enable): ");
    UART_SendInt((USART1->CR1 >> 13) & 1);
    UART_SendString(", TE (TX Enable): ");
    UART_SendInt((USART1->CR1 >> 3) & 1);
    UART_SendString(", RE (RX Enable): ");
    UART_SendInt((USART1->CR1 >> 2) & 1);
    UART_SendString("\r\n");
    
    UART_SendString("  USART1->CR2  = 0x");
    uint32_t cr2 = USART1->CR2;
    for (int i = 7; i >= 0; i--) {
        uint8_t nibble = (cr2 >> (i * 4)) & 0xF;
        UART_SendChar(nibble < 10 ? '0' + nibble : 'A' + nibble - 10);
    }
    UART_SendString("  (Control Register 2)\r\n");
    UART_SendString("    - STOP bits: ");
    uint8_t stop = (USART1->CR2 >> 12) & 0x3;
    if (stop == 0) UART_SendString("1 bit");
    else if (stop == 2) UART_SendString("2 bits");
    else UART_SendString("other");
    UART_SendString("\r\n");
    
    UART_SendString("  USART1->SR   = Status Register (runtime)\r\n");
    UART_SendString("  USART1->DR   = Data Register (read/write)\r\n");
    UART_SendString("\r\n");
    
    /* ===== 2.3 TIMER REGISTERS ===== */
    UART_SendString("[ 2.3 TIMER DAN COUNTER ]\r\n");
    UART_SendString("------------------------------------------------\r\n");
    
    UART_SendString("  TIM1 (PWM Generator - 1kHz):\r\n");
    UART_SendString("    TIM1->CR1  = 0x");
    UART_SendInt(TIM1->CR1);
    UART_SendString("  (CEN=");
    UART_SendInt(TIM1->CR1 & 1);
    UART_SendString(")\r\n");
    UART_SendString("    TIM1->PSC  = ");
    UART_SendInt(TIM1->PSC);
    UART_SendString("    (Prescaler)\r\n");
    UART_SendString("    TIM1->ARR  = ");
    UART_SendInt(TIM1->ARR);
    UART_SendString("   (Auto-Reload)\r\n");
    UART_SendString("    TIM1->CCR1 = ");
    UART_SendInt(TIM1->CCR1);
    UART_SendString("     (PWM Duty Cycle)\r\n");
    UART_SendString("    TIM1->CCMR1= 0x");
    UART_SendInt(TIM1->CCMR1);
    UART_SendString(" (PWM Mode)\r\n");
    UART_SendString("    TIM1->CCER = 0x");
    UART_SendInt(TIM1->CCER);
    UART_SendString("  (Output Enable)\r\n");
    UART_SendString("    TIM1->BDTR = 0x");
    UART_SendInt(TIM1->BDTR);
    UART_SendString(" (MOE Enable)\r\n");
    UART_SendString("    PWM Freq = 100MHz / (99+1) / (999+1) = 1kHz\r\n");
    UART_SendString("\r\n");
    
    UART_SendString("  TIM2 (Timer Interrupt - 1Hz):\r\n");
    UART_SendString("    TIM2->CR1  = 0x");
    UART_SendInt(TIM2->CR1);
    UART_SendString("  (CEN=");
    UART_SendInt(TIM2->CR1 & 1);
    UART_SendString(")\r\n");
    UART_SendString("    TIM2->PSC  = ");
    UART_SendInt(TIM2->PSC);
    UART_SendString(" (Prescaler)\r\n");
    UART_SendString("    TIM2->ARR  = ");
    UART_SendInt(TIM2->ARR);
    UART_SendString(" (Auto-Reload)\r\n");
    UART_SendString("    TIM2->DIER = 0x");
    UART_SendInt(TIM2->DIER);
    UART_SendString("  (UIE=");
    UART_SendInt(TIM2->DIER & 1);
    UART_SendString(" Update Int Enable)\r\n");
    UART_SendString("    TIM2->CNT  = Counter (runtime)\r\n");
    UART_SendString("    Int Freq = 50MHz / (9999+1) / (4999+1) = 1Hz\r\n");
    UART_SendString("\r\n");
    
    /* ===== 2.4 INTERRUPT REGISTERS ===== */
    UART_SendString("[ 2.4 INTERRUPT SYSTEM ]\r\n");
    UART_SendString("------------------------------------------------\r\n");
    
    UART_SendString("  EXTI (External Interrupt):\r\n");
    UART_SendString("    EXTI->IMR  = 0x");
    UART_SendInt(EXTI->IMR);
    UART_SendString(" (Interrupt Mask)\r\n");
    UART_SendString("      - Line 4: ");
    UART_SendInt((EXTI->IMR >> 4) & 1);
    UART_SendString(" (PB4 Button)\r\n");
    UART_SendString("      - Line 5: ");
    UART_SendInt((EXTI->IMR >> 5) & 1);
    UART_SendString(" (PB5 Button)\r\n");
    UART_SendString("    EXTI->RTSR = 0x");
    UART_SendInt(EXTI->RTSR);
    UART_SendString(" (Rising Trigger)\r\n");
    UART_SendString("    EXTI->FTSR = 0x");
    UART_SendInt(EXTI->FTSR);
    UART_SendString(" (Falling Trigger)\r\n");
    UART_SendString("    EXTI->PR   = Pending Register (runtime)\r\n");
    UART_SendString("\r\n");
    
    UART_SendString("  NVIC (Interrupt Controller):\r\n");
    UART_SendString("    NVIC->ISER[0] = 0x");
    uint32_t iser = NVIC->ISER[0];
    for (int i = 7; i >= 0; i--) {
        uint8_t nibble = (iser >> (i * 4)) & 0xF;
        UART_SendChar(nibble < 10 ? '0' + nibble : 'A' + nibble - 10);
    }
    UART_SendString("\r\n");
    UART_SendString("      - TIM2_IRQn (28): ");
    UART_SendInt((NVIC->ISER[0] >> 28) & 1);
    UART_SendString("\r\n");
    UART_SendString("      - EXTI4_IRQn (10): ");
    UART_SendInt((NVIC->ISER[0] >> 10) & 1);
    UART_SendString("\r\n");
    UART_SendString("      - EXTI9_5_IRQn (23): ");
    UART_SendInt((NVIC->ISER[0] >> 23) & 1);
    UART_SendString("\r\n\r\n");
    
    /* ===== 2.5 GPIO REGISTERS ===== */
    UART_SendString("[ 2.5 GPIO CONFIGURATION ]\r\n");
    UART_SendString("------------------------------------------------\r\n");
    
    UART_SendString("  GPIOA (DS18B20, PWM, UART):\r\n");
    UART_SendString("    GPIOA->MODER   = 0x");
    uint32_t moder = GPIOA->MODER;
    for (int i = 7; i >= 0; i--) {
        uint8_t nibble = (moder >> (i * 4)) & 0xF;
        UART_SendChar(nibble < 10 ? '0' + nibble : 'A' + nibble - 10);
    }
    UART_SendString("\r\n");
    UART_SendString("      - PA0: ");
    uint8_t pa0_mode = (GPIOA->MODER >> 0) & 0x3;
    if (pa0_mode == 0) UART_SendString("Input");
    else if (pa0_mode == 1) UART_SendString("Output");
    else if (pa0_mode == 2) UART_SendString("AF");
    else UART_SendString("Analog");
    UART_SendString(" (DS18B20)\r\n");
    UART_SendString("      - PA8: ");
    uint8_t pa8_mode = (GPIOA->MODER >> 16) & 0x3;
    if (pa8_mode == 2) UART_SendString("AF");
    UART_SendString(" (TIM1_CH1 PWM)\r\n");
    UART_SendString("      - PA9/PA10: AF (USART1 TX/RX)\r\n");
    
    UART_SendString("    GPIOA->OTYPER  = 0x");
    UART_SendInt(GPIOA->OTYPER);
    UART_SendString("\r\n");
    UART_SendString("      - PA0: ");
    UART_SendString((GPIOA->OTYPER & 1) ? "Open-Drain" : "Push-Pull");
    UART_SendString("\r\n");
    
    UART_SendString("    GPIOA->OSPEEDR = 0x");
    UART_SendInt(GPIOA->OSPEEDR);
    UART_SendString(" (Speed)\r\n");
    
    UART_SendString("    GPIOA->PUPDR   = 0x");
    UART_SendInt(GPIOA->PUPDR);
    UART_SendString(" (Pull-up/down)\r\n");
    
    UART_SendString("    GPIOA->IDR     = Input Data Register (runtime)\r\n");
    UART_SendString("    GPIOA->ODR     = Output Data Register\r\n");
    UART_SendString("    GPIOA->BSRR    = Bit Set/Reset Register\r\n");
    UART_SendString("    GPIOA->AFR[0/1]= Alternate Function\r\n");
    UART_SendString("\r\n");
    
    UART_SendString("  GPIOB (LEDs, Buttons):\r\n");
    UART_SendString("    GPIOB->MODER   = 0x");
    moder = GPIOB->MODER;
    for (int i = 7; i >= 0; i--) {
        uint8_t nibble = (moder >> (i * 4)) & 0xF;
        UART_SendChar(nibble < 10 ? '0' + nibble : 'A' + nibble - 10);
    }
    UART_SendString("\r\n");
    UART_SendString("      - PB0,PB1,PB2: Output (LEDs)\r\n");
    UART_SendString("      - PB4,PB5: Input (Buttons)\r\n");
    
    UART_SendString("    GPIOB->OTYPER  = 0x");
    UART_SendInt(GPIOB->OTYPER);
    UART_SendString(" (Push-Pull for LEDs)\r\n");
    
    UART_SendString("    GPIOB->PUPDR   = 0x");
    UART_SendInt(GPIOB->PUPDR);
    UART_SendString("\r\n");
    UART_SendString("      - PB4,PB5: Pull-up (01)\r\n");
    
    UART_SendString("\r\n");
    UART_SendString("================================================================\r\n");
    UART_SendString("\r\n");
}

/* ============================================================================
 * GPIO INIT - REGISTER LEVEL
 * ============================================================================
 * 
 * STM32F4 GPIO Registers:
 * - GPIOx_MODER   : Mode Register (00=Input, 01=Output, 10=AF, 11=Analog)
 * - GPIOx_OTYPER  : Output Type (0=Push-Pull, 1=Open-Drain)
 * - GPIOx_OSPEEDR : Output Speed
 * - GPIOx_PUPDR   : Pull-up/Pull-down (00=None, 01=Pull-up, 10=Pull-down)
 * - GPIOx_IDR     : Input Data Register
 * - GPIOx_ODR     : Output Data Register
 * - GPIOx_BSRR    : Bit Set/Reset Register
 * - GPIOx_AFR[2]  : Alternate Function Register
 * 
 * ============================================================================ */
static void MX_GPIO_Init_RegisterLevel(void)
{
    /* ========== Enable Clock untuk GPIO ========== */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;    /* Enable GPIOA clock */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;    /* Enable GPIOB clock */
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;   /* Enable SYSCFG untuk EXTI */
    
    /* ========== PA0: DS18B20 (Open-Drain Output) ========== */
    GPIOA->MODER &= ~GPIO_MODER_MODER0;
    GPIOA->MODER |= GPIO_MODER_MODER0_0;        /* Output (01) */
    GPIOA->OTYPER |= GPIO_OTYPER_OT0;           /* Open-Drain */
    GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED0;     /* High speed */
    GPIOA->BSRR = GPIO_PIN_0;                   /* Set HIGH (idle) */
    
    /* ========== PB0, PB1, PB2: LED (Push-Pull Output) ========== */
    /* PB0 - LED Hijau */
    GPIOB->MODER &= ~GPIO_MODER_MODER0;
    GPIOB->MODER |= GPIO_MODER_MODER0_0;        /* Output (01) */
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT0;          /* Push-Pull */
    
    /* PB1 - LED Kuning */
    GPIOB->MODER &= ~GPIO_MODER_MODER1;
    GPIOB->MODER |= GPIO_MODER_MODER1_0;
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT1;
    
    /* PB2 - LED Merah */
    GPIOB->MODER &= ~GPIO_MODER_MODER2;
    GPIOB->MODER |= GPIO_MODER_MODER2_0;
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT2;
    
    /* Matikan semua LED */
    GPIOB->BSRR = (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2) << 16;
    
    /* ========== PB4, PB5: Buttons (Input dengan Pull-up + EXTI) ========== */
    /* PB4 - Button 1 */
    GPIOB->MODER &= ~GPIO_MODER_MODER4;         /* Input (00) */
    GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD4;
    GPIOB->PUPDR |= GPIO_PUPDR_PUPD4_0;         /* Pull-up (01) */
    
    /* PB5 - Button 2 */
    GPIOB->MODER &= ~GPIO_MODER_MODER5;
    GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD5;
    GPIOB->PUPDR |= GPIO_PUPDR_PUPD5_0;
    
    /* ========== EXTI Configuration ========== */
    /* Connect PB4 ke EXTI4, PB5 ke EXTI5 */
    SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR2_EXTI4;
    SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PB;
    
    SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR2_EXTI5;
    SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI5_PB;
    
    /* Enable EXTI line dan set Falling Trigger */
    EXTI->IMR |= EXTI_IMR_IM4 | EXTI_IMR_IM5;
    EXTI->FTSR |= EXTI_FTSR_TR4 | EXTI_FTSR_TR5;
    
    /* Enable NVIC */
    NVIC_SetPriority(EXTI4_IRQn, 2);
    NVIC->ISER[0] |= (1 << EXTI4_IRQn);
    
    NVIC_SetPriority(EXTI9_5_IRQn, 2);
    NVIC->ISER[0] |= (1 << EXTI9_5_IRQn);
}

/* ============================================================================
 * USART1 INIT - REGISTER LEVEL
 * ============================================================================
 * 
 * Baud Rate Calculation:
 * BRR = fCK / (16 * Baud)  untuk Oversampling 16
 * BRR = 100MHz / (16 * 115200) = 54.253
 * Mantissa = 54 = 0x36
 * Fraction = 0.253 * 16 = 4 = 0x4
 * BRR = 0x364
 * 
 * Register:
 * - USART_CR1: UE (enable), TE (TX enable), RE (RX enable)
 * - USART_CR2: Stop bits
 * - USART_BRR: Baud rate
 * 
 * ============================================================================ */
static void MX_USART1_Init_RegisterLevel(void)
{
    /* Enable USART1 clock */
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    
    /* Configure PA9 (TX) dan PA10 (RX) sebagai Alternate Function */
    /* PA9: AF7 (USART1_TX) */
    GPIOA->MODER &= ~GPIO_MODER_MODER9;
    GPIOA->MODER |= GPIO_MODER_MODER9_1;        /* AF mode (10) */
    GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL9;
    GPIOA->AFR[1] |= (7 << GPIO_AFRH_AFSEL9_Pos);  /* AF7 = USART1 */
    GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED9;     /* High speed */
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD9;
    GPIOA->PUPDR |= GPIO_PUPDR_PUPD9_0;         /* Pull-up */
    
    /* PA10: AF7 (USART1_RX) */
    GPIOA->MODER &= ~GPIO_MODER_MODER10;
    GPIOA->MODER |= GPIO_MODER_MODER10_1;
    GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL10;
    GPIOA->AFR[1] |= (7 << GPIO_AFRH_AFSEL10_Pos);
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD10;
    GPIOA->PUPDR |= GPIO_PUPDR_PUPD10_0;
    
    /* Disable USART dulu sebelum konfigurasi */
    USART1->CR1 = 0;
    
    /* Set Baud Rate */
    /* BRR = fCK / Baud = 100MHz / 115200 = 868.05 */
    /* Dengan oversampling 16: Mantissa = 54, Fraction = 4 */
    /* BRR = (Mantissa << 4) | Fraction = (54 << 4) | 4 = 0x364 */
    USART1->BRR = 0x364;
    
    /* CR2: 1 Stop bit (default = 00) */
    USART1->CR2 = 0;
    
    /* CR3: No flow control */
    USART1->CR3 = 0;
    
    /* CR1: Enable USART, TX, RX */
    /* Bit 13: UE (USART Enable) */
    /* Bit 3: TE (Transmitter Enable) */
    /* Bit 2: RE (Receiver Enable) */
    /* Word length 8 bit (M=0), No parity (PCE=0) */
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

/* ============================================================================
 * TIM1 PWM INIT - REGISTER LEVEL
 * ============================================================================ */
static void MX_TIM1_PWM_Init_RegisterLevel(void)
{
    /* Enable TIM1 clock */
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    
    /* Configure PA8 sebagai TIM1_CH1 (AF1) */
    GPIOA->MODER &= ~GPIO_MODER_MODER8;
    GPIOA->MODER |= GPIO_MODER_MODER8_1;        /* AF mode */
    GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL8;
    GPIOA->AFR[1] |= (1 << GPIO_AFRH_AFSEL8_Pos);  /* AF1 = TIM1 */
    
    /* Reset TIM1 */
    TIM1->CR1 = 0;
    TIM1->CR2 = 0;
    
    /* Set Prescaler dan Auto-Reload */
    TIM1->PSC = 99;         /* 100MHz / 100 = 1MHz */
    TIM1->ARR = PWM_PERIOD; /* 1MHz / 1000 = 1kHz */
    TIM1->CCR1 = 0;         /* Initial duty = 0% */
    
    /* PWM Mode 1 pada Channel 1 */
    TIM1->CCMR1 &= ~TIM_CCMR1_OC1M;
    TIM1->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1);  /* PWM Mode 1 (110) */
    TIM1->CCMR1 |= TIM_CCMR1_OC1PE;     /* Preload enable */
    
    /* Enable Channel 1 output */
    TIM1->CCER |= TIM_CCER_CC1E;
    
    /* Advanced Timer: Enable Main Output */
    TIM1->BDTR |= TIM_BDTR_MOE;
    
    /* Enable Counter */
    TIM1->CR1 |= TIM_CR1_CEN;
}

/* ============================================================================
 * TIM2 TIMER INTERRUPT INIT - REGISTER LEVEL
 * ============================================================================ */
static void MX_TIM2_Interrupt_Init_RegisterLevel(void)
{
    /* Enable TIM2 clock */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    
    /* Reset TIM2 */
    TIM2->CR1 = 0;
    TIM2->CR2 = 0;
    
    /* Set Prescaler dan Auto-Reload untuk 1Hz (1 detik) */
    /* APB1 clock = 50MHz (setelah /2 prescaler) */
    TIM2->PSC = 9999;       /* 50MHz / 10000 = 5kHz */
    TIM2->ARR = 4999;       /* 5kHz / 5000 = 1Hz */
    TIM2->CNT = 0;
    
    /* Enable Update Interrupt */
    TIM2->DIER |= TIM_DIER_UIE;
    
    /* Clear pending flag */
    TIM2->SR &= ~TIM_SR_UIF;
    
    /* Enable NVIC */
    NVIC_SetPriority(TIM2_IRQn, 1);
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
    
    /* Set TIM1->CCR1 langsung */
    TIM1->CCR1 = (uint32_t)((percent * PWM_PERIOD) / 100);
}

/* ============================================================================
 * CALCULATE FAN SPEED
 * ============================================================================ */
uint8_t Calculate_Fan_Speed(float temp)
{
    if (temp <= TEMP_FAN_MIN) return FAN_SPEED_MIN;
    if (temp >= TEMP_FAN_MAX) return FAN_SPEED_MAX;
    
    float temp_range = TEMP_FAN_MAX - TEMP_FAN_MIN;
    float speed_range = FAN_SPEED_MAX - FAN_SPEED_MIN;
    float speed = ((temp - TEMP_FAN_MIN) / temp_range) * speed_range;
    
    return (uint8_t)speed;
}

/* ============================================================================
 * PRINT STATUS
 * ============================================================================ */
void Print_Status(float temp, uint8_t speed)
{
    UART_SendString("  ");
    
    /* Time */
    if (seconds_counter < 10) UART_SendString("      ");
    else if (seconds_counter < 100) UART_SendString("     ");
    else if (seconds_counter < 1000) UART_SendString("    ");
    else UART_SendString("   ");
    UART_SendInt(seconds_counter);
    UART_SendString(" |  ");
    
    /* Temperature */
    UART_SendFloat(temp, 2);
    UART_SendString("  |   ");
    
    /* Fan Speed */
    if (speed < 10) UART_SendString(" ");
    if (speed < 100) UART_SendString(" ");
    UART_SendInt(speed);
    UART_SendString("  | ");
    
    /* Mode */
    UART_SendString(auto_mode ? "AUTO  " : "MANUAL");
    UART_SendString(" | ");
    
    /* Status */
    if (temp < TEMP_COLD) UART_SendString("COLD");
    else if (temp < TEMP_HOT) UART_SendString("WARM");
    else UART_SendString("HOT!");
    
    UART_SendString("\r\n");
}

/* ============================================================================
 * UPDATE LEDs - REGISTER LEVEL (BSRR)
 * ============================================================================ */
void Update_LEDs_Temperature(float temp)
{
    /* Matikan semua LED */
    GPIOB->BSRR = (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2) << 16;
    
    /* Nyalakan sesuai suhu */
    if (temp < TEMP_COLD)
        GPIOB->BSRR = GPIO_PIN_0;       /* Hijau */
    else if (temp < TEMP_HOT)
        GPIOB->BSRR = GPIO_PIN_1;       /* Kuning */
    else
        GPIOB->BSRR = GPIO_PIN_2;       /* Merah */
}

/* ============================================================================
 * INTERRUPT HANDLERS
 * ============================================================================ */
void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF)
    {
        TIM2->SR &= ~TIM_SR_UIF;    /* Clear flag */
        flag_read_temp = 1;
    }
}

void EXTI4_IRQHandler(void)
{
    if (EXTI->PR & EXTI_PR_PR4)
    {
        EXTI->PR = EXTI_PR_PR4;     /* Clear pending */
        
        uint32_t current_time = HAL_GetTick();
        if ((current_time - last_button_time) < DEBOUNCE_MS) return;
        last_button_time = current_time;
        
        auto_mode = !auto_mode;
        UART_SendString("\r\n>>> Mode: ");
        UART_SendString(auto_mode ? "AUTO" : "MANUAL");
        UART_SendString("\r\n\r\n");
    }
}

void EXTI9_5_IRQHandler(void)
{
    if (EXTI->PR & EXTI_PR_PR5)
    {
        EXTI->PR = EXTI_PR_PR5;
        
        uint32_t current_time = HAL_GetTick();
        if ((current_time - last_button_time) < DEBOUNCE_MS) return;
        last_button_time = current_time;
        
        if (!auto_mode)
        {
            if (fan_speed_percent < 25) Set_Fan_Speed_RegisterLevel(25);
            else if (fan_speed_percent < 50) Set_Fan_Speed_RegisterLevel(50);
            else if (fan_speed_percent < 75) Set_Fan_Speed_RegisterLevel(75);
            else if (fan_speed_percent < 100) Set_Fan_Speed_RegisterLevel(100);
            else Set_Fan_Speed_RegisterLevel(0);
            
            UART_SendString("\r\n>>> Fan: ");
            UART_SendInt(fan_speed_percent);
            UART_SendString("%\r\n\r\n");
        }
    }
}

/* ============================================================================
 * DS18B20 FUNCTIONS (sama seperti sebelumnya)
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

void DS18B20_SetPinOutput(void)
{
    GPIOA->MODER &= ~GPIO_MODER_MODER0;
    GPIOA->MODER |= GPIO_MODER_MODER0_0;
}

void DS18B20_SetPinInput(void)
{
    GPIOA->MODER &= ~GPIO_MODER_MODER0;
}

uint8_t DS18B20_Reset(void)
{
    uint8_t presence = 0;
    DS18B20_SetPinOutput();
    GPIOA->BSRR = GPIO_PIN_0 << 16;
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
    GPIOA->BSRR = GPIO_PIN_0 << 16;
    if (bit) { delay_us(1); DS18B20_SetPinInput(); delay_us(60); }
    else { delay_us(60); DS18B20_SetPinInput(); delay_us(1); }
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
    for (uint8_t i = 0; i < 8; i++) { DS18B20_WriteBit(byte & 0x01); byte >>= 1; }
}

uint8_t DS18B20_ReadByte(void)
{
    uint8_t byte = 0;
    for (uint8_t i = 0; i < 8; i++) { byte >>= 1; if (DS18B20_ReadBit()) byte |= 0x80; }
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

void Error_Handler(void)
{
    __disable_irq();
    while (1) { GPIOB->ODR ^= GPIO_PIN_2; for (volatile int i = 0; i < 500000; i++); }
}

/* Standard Handlers */
void SysTick_Handler(void)     { HAL_IncTick(); }
void NMI_Handler(void)         { while(1); }
void HardFault_Handler(void)   { while(1); }
void MemManage_Handler(void)   { while(1); }
void BusFault_Handler(void)    { while(1); }
void UsageFault_Handler(void)  { while(1); }
void SVC_Handler(void)         {}
void DebugMon_Handler(void)    {}
void PendSV_Handler(void)      {}