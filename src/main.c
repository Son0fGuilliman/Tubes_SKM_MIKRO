/**
 * ============================================================================
 * FAN SPEED CONTROL - STM32F411 (PlatformIO + STM32Cube Framework)
 * ============================================================================
 * FIXED: Added TIM1 MOE (Main Output Enable) for PWM output
 * ============================================================================
 */

#include "stm32f4xx_hal.h"

/* ============================================================================
 * KONFIGURASI
 * ============================================================================ */
#define PWM_PERIOD      999
#define SPEED_STEP      10
#define DEBOUNCE_MS     200

/* ============================================================================
 * GLOBAL VARIABLES
 * ============================================================================ */
TIM_HandleTypeDef htim1;

volatile uint8_t fan_speed_percent = 0;
volatile uint32_t last_button_time = 0;

/* ============================================================================
 * FUNCTION PROTOTYPES
 * ============================================================================ */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
void Set_Fan_Speed(uint8_t percent);
void Update_LEDs(void);
void Error_Handler(void);

/* ============================================================================
 * MAIN FUNCTION
 * ============================================================================ */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM1_Init();
    
    /* Start PWM pada TIM1 Channel 1 (PA8) */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    
    /* !!!! FIX: Enable Main Output untuk TIM1 (Advanced Timer) !!!! */
    __HAL_TIM_MOE_ENABLE(&htim1);
    
    /* Set speed awal 0% */
    Set_Fan_Speed(0);
    
    /* LED Hijau ON - Sistem Ready */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    
    /* Main Loop */
    while (1)
    {
        Update_LEDs();
        HAL_Delay(50);
    }
}

/* ============================================================================
 * SET FAN SPEED (0-100%)
 * ============================================================================ */
void Set_Fan_Speed(uint8_t percent)
{
    if (percent > 100) percent = 100;
    
    fan_speed_percent = percent;
    
    /* Hitung PWM duty cycle */
    uint32_t pwm_value = (uint32_t)((percent * PWM_PERIOD) / 100);
    
    /* Set PWM */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_value);
}

/* ============================================================================
 * UPDATE LED INDICATORS
 * ============================================================================ */
void Update_LEDs(void)
{
    static uint32_t last_blink = 0;
    static uint8_t led_state = 0;
    
    /* LED Hijau (PB0) - Always ON */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    
    /* LED Kuning (PB1) - Blink sesuai speed */
    if (fan_speed_percent > 0)
    {
        uint32_t blink_interval = 500 - (fan_speed_percent * 4);
        if (blink_interval < 50) blink_interval = 50;
        
        if (HAL_GetTick() - last_blink >= blink_interval)
        {
            led_state = !led_state;
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, led_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
            last_blink = HAL_GetTick();
        }
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    }
    
    /* LED Merah (PB2) - ON saat MAX/MIN */
    if (fan_speed_percent == 100 || fan_speed_percent == 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
    }
}

/* ============================================================================
 * BUTTON INTERRUPT CALLBACK
 * ============================================================================ */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t current_time = HAL_GetTick();
    
    if ((current_time - last_button_time) < DEBOUNCE_MS)
    {
        return;
    }
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
    
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ============================================================================
 * TIM1 INIT - PWM 1kHz pada PA8
 * ============================================================================ */
static void MX_TIM1_Init(void)
{
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    /* Configure PA8 as TIM1_CH1 (AF1) */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;  // Changed to HIGH speed
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* Timer config */
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 99;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = PWM_PERIOD;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim1);
    
    /* PWM Channel config */
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
    
    /* Break and Dead Time config - PENTING untuk TIM1! */
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;  // Auto enable output
    HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig);
}

/* ============================================================================
 * GPIO INIT - LEDs dan Buttons
 * ============================================================================ */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /* LED OUTPUTS (PB0, PB1, PB2) */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2, GPIO_PIN_RESET);
    
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    /* BUTTON INPUTS dengan EXTI (PB4, PB5) */
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