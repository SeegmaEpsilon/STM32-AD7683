#ifndef AD7683_AD7683_H_
#define AD7683_AD7683_H_

/*
 * You can receive data from the AD7683 ADC in two ways: using hardware SPI or using bit-banging.
 * For the first method, you need to configure SPI to Full-Duplex and the clock signal speed is not more than 2.9 MHz
 * (it is also possible to "receive only", but inside it still contains ReceiveTransmit, so it does not matter).
 * IMPORTANT: CPOL and CPHA must be 1 - so high, 2 edge
 * For the second method, you just need to set up 3 output pins and add their definition to the corresponding macros.
 */

#include "main.h" // if you use HAL, main.h must contains defines from CubeMX

#define CS_LOW()  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET)
#define CS_HIGH() HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET)

// uncomment one of this:
//#define MODE_BIT_BANGING
#define MODE_SPI

// important: clock rate must be lower than 2.9 MHz

#if defined(MODE_SPI)
  uint16_t AD7683_get_value_SPI(SPI_HandleTypeDef *hspi);    // get data via SPI
#elif defined(MODE_BIT_BANGING)
  #define CLK_HIGH()   HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_SET)
  #define CLK_LOW()    HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_RESET)
  #define MISO_READ()  HAL_GPIO_ReadPin(MISO_GPIO_Port, MISO_Pin)
  uint16_t AD7683_get_value_BB(void);                        // get data using bit-banging (BB)
#endif

#endif /* AD7683_AD7683_H_ */
