#ifndef TM1637_H
#define TM1637_H

#include "stm32f4xx.h"
#include <stdint.h>

#define TM1637_DIO_PIN     GPIO_PIN_7   // Example: DIO on PB7
#define TM1637_CLK_PIN     GPIO_PIN_6   // Example: CLK on PB6
#define TM1637_GPIO_PORT   GPIOB        // Adjust as needed

#define TM1637_BRIGHTNESS  0x0F         // Range: 0x88 to 0x8F (brightness levels)

void TM1637_Init(void);
void TM1637_DisplayDigit(uint8_t pos, uint8_t digit);
void TM1637_DisplayNumber(int number);
void TM1637_Clear(void);

#endif
