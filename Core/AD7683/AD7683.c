#include "AD7683.h"

#if defined(MODE_SPI)
uint16_t AD7683_get_value_SPI(SPI_HandleTypeDef *hspi)
{
  uint32_t result = 0;
  uint8_t dummy[3] = {0};
  uint8_t byte_array[3] = {0};

  CS_LOW();
  HAL_SPI_TransmitReceive(hspi, dummy, byte_array, 3, 100);
  CS_HIGH();

  byte_array[0] &= 0b00000011;
  byte_array[1] &= 0b11111111;
  byte_array[2] &= 0b11111100;

  result = (byte_array[0] << 14) | (byte_array[1] << 6) | (byte_array[2] >> 2);
  return (uint16_t)result;
}

#elif defined(MODE_BIT_BANGING)
uint16_t AD7683_get_value_BB(void)
{
  uint16_t spi_data = 0;

  CS_LOW();
  for(int i = 0; i < 22; i++)
  {
    spi_data <<= 1;
    CLK_HIGH();
    for(int i = 0; i < 22; i++) __asm__("nop");
    spi_data |= MISO_READ();
    CLK_LOW();
    for(int i = 0; i < 22; i++) __asm__("nop");
  }
  CS_HIGH();
  return spi_data;
}

#endif

