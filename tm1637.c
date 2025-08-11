#include "TM1637.h"
#include "cmn.h"  // assumes delay_us() or delay_ms() defined here

static const uint8_t digitToSegment[] = {
    0x3F, // 0
    0x06, // 1
    0x5B, // 2
    0x4F, // 3
    0x66, // 4
    0x6D, // 5
    0x7D, // 6
    0x07, // 7
    0x7F, // 8
    0x6F, // 9
    0x00  // blank
};

// ----------- Low-level helper functions -----------

static void TM1637_Start(void) {
    GPIO_SetBits(TM1637_GPIO_PORT, TM1637_CLK_PIN | TM1637_DIO_PIN);
    delay_us(2);
    GPIO_ResetBits(TM1637_GPIO_PORT, TM1637_DIO_PIN);
}

static void TM1637_Stop(void) {
    GPIO_ResetBits(TM1637_GPIO_PORT, TM1637_CLK_PIN);
    delay_us(2);
    GPIO_ResetBits(TM1637_GPIO_PORT, TM1637_DIO_PIN);
    delay_us(2);
    GPIO_SetBits(TM1637_GPIO_PORT, TM1637_CLK_PIN);
    delay_us(2);
    GPIO_SetBits(TM1637_GPIO_PORT, TM1637_DIO_PIN);
}

static void TM1637_WriteByte(uint8_t b) {
    for (int i = 0; i < 8; i++) {
        GPIO_ResetBits(TM1637_GPIO_PORT, TM1637_CLK_PIN);
        delay_us(3);
        if (b & 0x01) {
            GPIO_SetBits(TM1637_GPIO_PORT, TM1637_DIO_PIN);
        } else {
            GPIO_ResetBits(TM1637_GPIO_PORT, TM1637_DIO_PIN);
        }
        b >>= 1;
        delay_us(3);
        GPIO_SetBits(TM1637_GPIO_PORT, TM1637_CLK_PIN);
        delay_us(3);
    }

    // ACK bit
    GPIO_ResetBits(TM1637_GPIO_PORT, TM1637_CLK_PIN);
    GPIO_SetBits(TM1637_GPIO_PORT, TM1637_DIO_PIN); // release
    delay_us(3);
    GPIO_SetBits(TM1637_GPIO_PORT, TM1637_CLK_PIN);
    delay_us(3);
    GPIO_ResetBits(TM1637_GPIO_PORT, TM1637_CLK_PIN);
}

// ----------- Public Interface -----------

void TM1637_Init(void) {
    // Enable GPIO clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    // Set CLK and DIO as outputs
    TM1637_GPIO_PORT->MODER &= ~(0xF << (6 * 2)); // clear mode for pins 6 and 7
    TM1637_GPIO_PORT->MODER |=  (0x5 << (6 * 2)); // set mode to output
}

void TM1637_DisplayDigit(uint8_t pos, uint8_t digit) {
    TM1637_Start();
    TM1637_WriteByte(0x40);  // write command
    TM1637_Stop();

    TM1637_Start();
    TM1637_WriteByte(0xC0 + pos);
    TM1637_WriteByte(digitToSegment[digit]);
    TM1637_Stop();

    TM1637_Start();
    TM1637_WriteByte(TM1637_BRIGHTNESS);  // brightness control
    TM1637_Stop();
}

void TM1637_DisplayNumber(int number) {
    TM1637_Start();
    TM1637_WriteByte(0x40);  // write command
    TM1637_Stop();

    TM1637_Start();
    TM1637_WriteByte(0xC0);  // address command

    for (int i = 3; i >= 0; i--) {
        TM1637_WriteByte(digitToSegment[(number / (int)pow(10, i)) % 10]);
    }

    TM1637_Stop();
    TM1637_Start();
    TM1637_WriteByte(TM1637_BRIGHTNESS);
    TM1637_Stop();
}

void TM1637_Clear(void) {
    TM1637_Start();
    TM1637_WriteByte(0x40);  // set auto-increment mode
    TM1637_Stop();

    TM1637_Start();
    TM1637_WriteByte(0xC0);  // address 0
    for (int i = 0; i < 4; i++) {
        TM1637_WriteByte(0x00);
    }
    TM1637_Stop();

    TM1637_Start();
    TM1637_WriteByte(TM1637_BRIGHTNESS);
    TM1637_Stop();
}
