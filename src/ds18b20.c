#include "ds18b20.h"
#include "timing.h"

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
  delay_ms(750);

  if (!DS18B20_Reset())
    return -999.0f;
  DS18B20_WriteByte(DS18B20_CMD_SKIP_ROM);
  DS18B20_WriteByte(DS18B20_CMD_READ_SCRATCHPAD);

  temp_lsb = DS18B20_ReadByte();
  temp_msb = DS18B20_ReadByte();

  temp_raw = (temp_msb << 8) | temp_lsb;
  return (float)temp_raw / 16.0f;
}
