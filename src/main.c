#include "stm32f4xx.h"

// Fungsi Delay Sederhana
void delay_ms(volatile uint32_t count) {
    while (count--) {
        for (volatile int i = 0; i < 1000; i++); 
    }
}

int main(void) {
    // 1. Aktifkan Clock GPIOB (Bit 1 di AHB1ENR)
    RCC->AHB1ENR |= (1 << 1);

    // 2. Reset Mode Pin PB0, PB1, PB2 (Bersihkan bit lama)
    GPIOB->MODER &= ~((3 << 0) | (3 << 2) | (3 << 4));

    // 3. Set Mode menjadi Output (01)
    GPIOB->MODER |=  ((1 << 0) | (1 << 2) | (1 << 4));

    while (1) {
        // Merah ON (PB2)
        GPIOB->ODR |= (1 << 2);
        delay_ms(500);
        GPIOB->ODR &= ~(1 << 2); // OFF

        // Kuning ON (PB1)
        GPIOB->ODR |= (1 << 1);
        delay_ms(500);
        GPIOB->ODR &= ~(1 << 1); // OFF

        // Hijau ON (PB0)
        GPIOB->ODR |= (1 << 0);
        delay_ms(500);
        GPIOB->ODR &= ~(1 << 0); // OFF
        
        delay_ms(500);
    }
}