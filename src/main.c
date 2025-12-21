/**
 * ============================================================================
 * FAN SPEED CONTROL + DS18B20 + UART SERIAL MONITOR
 * STM32F411 (PlatformIO + STM32Cube Framework)
 * ============================================================================
 * 
 * FITUR:
 * - Baca suhu dari DS18B20 (1-Wire Protocol)
 * - Kontrol kipas dengan 2 tombol (UP/DOWN)
 * - LED indikator tingkat panas
 * - Serial Monitor via USB-TTL (UART)
 * 
 * PIN MAPPING:
 * - PA0  : DS18B20 Data (1-Wire) + Pull-up 4.7K ke 3.3V
 * - PA8  : PWM Output → 2N2222 Base (via R 1K) → Kipas
 * - PA9  : UART TX → USB-TTL RX (CROSS!)
 * - PA10 : UART RX → USB-TTL TX (CROSS!)
 * - PB4  : Button UP   (Internal Pull-up, Active LOW)
 * - PB5  : Button DOWN (Internal Pull-up, Active LOW)
 * - PB0  : LED Hijau   → Suhu DINGIN (< 30°C)
 * - PB1  : LED Kuning  → Suhu HANGAT (30-40°C)
 * - PB2  : LED Merah   → Suhu PANAS  (> 40°C)
 * 
 * SERIAL MONITOR SETTINGS:
 * - Baud Rate: 115200
 * - Data Bits: 8
 * - Parity: None
 * - Stop Bits: 1
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
#define SPEED_STEP          10
#define DEBOUNCE_MS         200

/* Threshold Suhu (dalam °C) */
#define TEMP_COLD           30.0f       // Di bawah ini = DINGIN (LED Hijau)
#define TEMP_HOT            40.0f       // Di atas ini = PANAS (LED Merah)

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
TIM_HandleTypeDef htim1;
UART_HandleTypeDef huart1;

volatile uint8_t fan_speed_percent = 0;
volatile uint32_t last_button_time = 0;
float current_temperature = 0.0f;
uint8_t sensor_ok = 0;

/* ============================================================================
 * FUNCTION PROTOTYPES
 * ============================================================================ */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
void Set_Fan_Speed(uint8_t percent);
void Update_LEDs_Temperature(float temp);
void Print_Status(float temp, uint8_t speed);
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
    
    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_USART1_UART_Init();
    
    /* Start PWM */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    
    /* Set speed awal 0% */
    Set_Fan_Speed(0);
    
    /* Welcome Message */
    printf("\r\n");
    printf("============================================\r\n");
    printf("   FAN SPEED CONTROL + DS18B20 MONITOR\r\n");
    printf("   STM32F411 Black Pill\r\n");
    printf("============================================\r\n");
    printf("\r\n");
    
    /* Test LED saat startup */
    printf("[BOOT] Testing LEDs...\r\n");
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2, GPIO_PIN_SET);
    HAL_Delay(500);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2, GPIO_PIN_RESET);
    printf("[BOOT] LED test OK\r\n");
    
    /* Check DS18B20 */
    printf("[BOOT] Detecting DS18B20 sensor...\r\n");
    sensor_ok = DS18B20_Reset();
    
    if (!sensor_ok)
    {
        printf("[ERROR] DS18B20 NOT DETECTED!\r\n");
        printf("[ERROR] Check wiring:\r\n");
        printf("        - VDD -> 3.3V\r\n");
        printf("        - GND -> GND\r\n");
        printf("        - DQ  -> PA0 + 4.7K pull-up to 3.3V\r\n");
        printf("\r\n");
        
        /* LED Merah blink cepat */
        while (1)
        {
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
            HAL_Delay(100);
        }
    }
    
    printf("[BOOT] DS18B20 detected OK!\r\n");
    printf("\r\n");
    printf("Controls:\r\n");
    printf("  BTN UP (PB4)   : Increase fan speed +10%%\r\n");
    printf("  BTN DOWN (PB5) : Decrease fan speed -10%%\r\n");
    printf("\r\n");
    printf("LED Indicators:\r\n");
    printf("  GREEN  (PB0) : Temp < 30C (COLD)\r\n");
    printf("  YELLOW (PB1) : Temp 30-40C (WARM)\r\n");
    printf("  RED    (PB2) : Temp > 40C (HOT)\r\n");
    printf("\r\n");
    printf("============================================\r\n");
    printf("Starting monitoring...\r\n");
    printf("\r\n");
    
    /* Print header untuk data */
    printf("  TIME(s)  |  TEMP(C)  |  FAN(%%)  |  STATUS\r\n");
    printf("-----------|-----------|----------|----------\r\n");
    
    /* Main Loop */
    uint32_t last_temp_read = 0;
    uint32_t start_time = HAL_GetTick();
    
    while (1)
    {
        /* Baca suhu setiap 1 detik */
        if (HAL_GetTick() - last_temp_read >= 1000)
        {
            current_temperature = DS18B20_ReadTemperature();
            last_temp_read = HAL_GetTick();
            
            /* Print status ke Serial Monitor */
            Print_Status(current_temperature, fan_speed_percent);
        }
        
        /* Update LED berdasarkan suhu */
        Update_LEDs_Temperature(current_temperature);
        
        HAL_Delay(50);
    }
}

/* ============================================================================
 * PRINT STATUS KE SERIAL MONITOR
 * ============================================================================ */
void Print_Status(float temp, uint8_t speed)
{
    static uint32_t seconds = 0;
    char status[16];
    
    /* Tentukan status berdasarkan suhu */
    if (temp < TEMP_COLD)
    {
        strcpy(status, "COLD");
    }
    else if (temp >= TEMP_COLD && temp < TEMP_HOT)
    {
        strcpy(status, "WARM");
    }
    else
    {
        strcpy(status, "HOT!");
    }
    
    /* Print data dalam format tabel */
    printf("  %7lu  |  %6.2f   |   %3u    |  %s\r\n", 
           seconds, temp, speed, status);
    
    seconds++;
}

/* ============================================================================
 * DWT INIT - Untuk delay microsecond
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
 * DS18B20 - GPIO Mode Switching
 * ============================================================================ */
void DS18B20_SetPinOutput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DS18B20_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DS18B20_PORT, &GPIO_InitStruct);
}

void DS18B20_SetPinInput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DS18B20_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DS18B20_PORT, &GPIO_InitStruct);
}

/* ============================================================================
 * DS18B20 - Reset & Presence Detect
 * ============================================================================ */
uint8_t DS18B20_Reset(void)
{
    uint8_t presence = 0;
    
    DS18B20_SetPinOutput();
    HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_RESET);
    delay_us(480);
    
    DS18B20_SetPinInput();
    delay_us(70);
    
    if (HAL_GPIO_ReadPin(DS18B20_PORT, DS18B20_PIN) == GPIO_PIN_RESET)
    {
        presence = 1;
    }
    
    delay_us(410);
    return presence;
}

/* ============================================================================
 * DS18B20 - Write Bit
 * ============================================================================ */
void DS18B20_WriteBit(uint8_t bit)
{
    DS18B20_SetPinOutput();
    HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_RESET);
    
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

/* ============================================================================
 * DS18B20 - Read Bit
 * ============================================================================ */
uint8_t DS18B20_ReadBit(void)
{
    uint8_t bit = 0;
    
    DS18B20_SetPinOutput();
    HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_RESET);
    delay_us(2);
    
    DS18B20_SetPinInput();
    delay_us(10);
    
    if (HAL_GPIO_ReadPin(DS18B20_PORT, DS18B20_PIN) == GPIO_PIN_SET)
    {
        bit = 1;
    }
    
    delay_us(50);
    return bit;
}

/* ============================================================================
 * DS18B20 - Write Byte
 * ============================================================================ */
void DS18B20_WriteByte(uint8_t byte)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        DS18B20_WriteBit(byte & 0x01);
        byte >>= 1;
    }
}

/* ============================================================================
 * DS18B20 - Read Byte
 * ============================================================================ */
uint8_t DS18B20_ReadByte(void)
{
    uint8_t byte = 0;
    
    for (uint8_t i = 0; i < 8; i++)
    {
        byte >>= 1;
        if (DS18B20_ReadBit())
        {
            byte |= 0x80;
        }
    }
    return byte;
}

/* ============================================================================
 * DS18B20 - Read Temperature (in Celsius)
 * ============================================================================ */
float DS18B20_ReadTemperature(void)
{
    uint8_t temp_lsb, temp_msb;
    int16_t temp_raw;
    float temperature;
    
    /* Step 1: Reset & Skip ROM */
    if (!DS18B20_Reset()) return -999.0f;
    DS18B20_WriteByte(DS18B20_CMD_SKIP_ROM);
    
    /* Step 2: Start Conversion */
    DS18B20_WriteByte(DS18B20_CMD_CONVERT_T);
    
    /* Step 3: Wait for conversion (750ms untuk 12-bit) */
    HAL_Delay(750);
    
    /* Step 4: Reset & Skip ROM lagi */
    if (!DS18B20_Reset()) return -999.0f;
    DS18B20_WriteByte(DS18B20_CMD_SKIP_ROM);
    
    /* Step 5: Read Scratchpad */
    DS18B20_WriteByte(DS18B20_CMD_READ_SCRATCHPAD);
    
    /* Step 6: Read 2 bytes suhu (LSB first) */
    temp_lsb = DS18B20_ReadByte();
    temp_msb = DS18B20_ReadByte();
    
    /* Step 7: Combine & Calculate */
    temp_raw = (temp_msb << 8) | temp_lsb;
    temperature = (float)temp_raw / 16.0f;
    
    return temperature;
}

/* ============================================================================
 * SET FAN SPEED (0-100%)
 * ============================================================================ */
void Set_Fan_Speed(uint8_t percent)
{
    if (percent > 100) percent = 100;
    
    fan_speed_percent = percent;
    uint32_t pwm_value = (uint32_t)((percent * PWM_PERIOD) / 100);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_value);
    
    /* Print perubahan speed */
    printf("\r\n>>> Fan speed changed to: %u%%\r\n", percent);
    printf("  TIME(s)  |  TEMP(C)  |  FAN(%%)  |  STATUS\r\n");
    printf("-----------|-----------|----------|----------\r\n");
}

/* ============================================================================
 * UPDATE LEDs - Berdasarkan SUHU
 * ============================================================================ */
void Update_LEDs_Temperature(float temp)
{
    /* Matikan semua LED dulu */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2, GPIO_PIN_RESET);
    
    if (temp < TEMP_COLD)
    {
        /* DINGIN (< 30°C) → LED HIJAU */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    }
    else if (temp >= TEMP_COLD && temp < TEMP_HOT)
    {
        /* HANGAT (30-40°C) → LED KUNING */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    }
    else
    {
        /* PANAS (> 40°C) → LED MERAH */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
    }
}

/* ============================================================================
 * BUTTON INTERRUPT CALLBACK
 * ============================================================================ */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t current_time = HAL_GetTick();
    
    if ((current_time - last_button_time) < DEBOUNCE_MS) return;
    last_button_time = current_time;
    
    /* Button UP (PB4) */
    if (GPIO_Pin == GPIO_PIN_4)
    {
        if (fan_speed_percent <= (100 - SPEED_STEP))
            Set_Fan_Speed(fan_speed_percent + SPEED_STEP);
        else
            Set_Fan_Speed(100);
    }
    
    /* Button DOWN (PB5) */
    if (GPIO_Pin == GPIO_PIN_5)
    {
        if (fan_speed_percent >= SPEED_STEP)
            Set_Fan_Speed(fan_speed_percent - SPEED_STEP);
        else
            Set_Fan_Speed(0);
    }
}

/* ============================================================================
 * SYSTEM CLOCK CONFIG - 100MHz dari HSI
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
 * TIM1 INIT - PWM 1kHz pada PA8
 * ============================================================================ */
static void MX_TIM1_Init(void)
{
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 99;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = PWM_PERIOD;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_PWM_Init(&htim1);
    
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
    
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig);
}

/* ============================================================================
 * USART1 INIT - 115200 baud pada PA9(TX), PA10(RX)
 * ============================================================================ */
static void MX_USART1_UART_Init(void)
{
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    /* Configure PA9 (TX) dan PA10 (RX) */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* UART Config */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ============================================================================
 * GPIO INIT - LEDs, Buttons, DS18B20
 * ============================================================================ */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /* ===== DS18B20 (PA0) - Open Drain ===== */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
    
    /* ===== LED OUTPUTS (PB0, PB1, PB2) ===== */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2, GPIO_PIN_RESET);
    
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    /* ===== BUTTON INPUTS dengan EXTI (PB4, PB5) ===== */
    GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);
    
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/* ============================================================================
 * ERROR HANDLER
 * ============================================================================ */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
        for (volatile int i = 0; i < 500000; i++);
    }
}

/* ============================================================================
 * INTERRUPT HANDLERS
 * ============================================================================ */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

void EXTI4_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
}

void EXTI9_5_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
}

void NMI_Handler(void) { while(1); }
void HardFault_Handler(void) { while(1); }
void MemManage_Handler(void) { while(1); }
void BusFault_Handler(void) { while(1); }
void UsageFault_Handler(void) { while(1); }
void SVC_Handler(void) {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void) {}