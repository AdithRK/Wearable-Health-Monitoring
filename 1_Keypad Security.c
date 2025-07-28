#include "stm32f405xx.h"

#include "lcd.h"

#include "spi.h"

#include <stdint.h>

// ==== Delay ====

void delay_ms(uint32_t ms) {

    for (uint32_t i = 0; i < ms * 8000; i++) {

        __asm__("nop");

    }

}

// ==== Keypad ====

char keypad_map[4][4] = {

    {'1','2','3','A'},

    {'4','5','6','B'},

    {'7','8','9','C'},

    {'*','0','#','D'}

};

void keypad_init(void) {

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    for (int i = 2; i <= 5; i++) {

        GPIOB->MODER &= ~(3 << (2 * i));

        GPIOB->MODER |=  (1 << (2 * i)); // Output

        GPIOB->ODR |= (1 << i);

    }

    for (int i = 6; i <= 9; i++) {

        GPIOB->MODER &= ~(3 << (2 * i)); // Input

        GPIOB->PUPDR &= ~(3 << (2 * i));

        GPIOB->PUPDR |=  (1 << (2 * i)); // Pull-up

    }

}

char keypad_getkey(void) {

    for (int row = 0; row < 4; row++) {

        GPIOB->ODR |= (0xF << 2);                 // Set all rows HIGH

        GPIOB->ODR &= ~(1 << (2 + row));          // Set one row LOW

        delay_ms(2);

        for (int col = 0; col < 4; col++) {

            if (!(GPIOB->IDR & (1 << (6 + col)))) {

                while (!(GPIOB->IDR & (1 << (6 + col))));

                return keypad_map[row][col];

            }

        }

    }

    return 0;

}

// ==== EEPROM ====

void eeprom_write_password(char *pass) {

    EepromWriteEnable();

    for (int i = 0; i < 4; i++) {

        EepromWriteByte(0x0000 + i, pass[i]);

    }

}

void eeprom_read_password(char *pass) {

    for (int i = 0; i < 4; i++) {

        pass[i] = EepromReadByte(0x0000 + i);

    }

}

// ==== Main ====

int main(void) {

    char entered[5] = {0}, stored[5] = {0}, newpass[5] = {0};

    uint8_t i = 0;

    LcdInit();

    keypad_init();

    SpiEepromInit();

    eeprom_write_password("1357"); // Default password

    while (1) {

        lprint(0x01, "");

        lprint(0x80, "Welcome user");

        lprint(0xC0, "A:Login B:Change");


        char mode = 0;

        while (!mode) {

            mode = keypad_getkey();

        }

        // ==== Login ====

        if (mode == 'A') {

            i = 0;

            lprint(0x01, "");

            lprint(0x80, "Enter Pswd:");

            while (i < 4) {

                char key = keypad_getkey();

                if (key && key != '*' && key != '#' && key != 'A' && key != 'B') {

                    entered[i++] = key;

                    LcdFxn(1, '*');

                }

            }

            eeprom_read_password(stored);

            if (entered[0] == stored[0] && entered[1] == stored[1] &&

                entered[2] == stored[2] && entered[3] == stored[3]) {

                lprint(0xC0, "Access Granted");

            } else {

                lprint(0xC0, "Access Denied ");

            }

            delay_ms(500);

        }

        // ==== Change Password ====

        else if (mode == 'B') {

            i = 0;

            lprint(0x01, "");

            lprint(0x80, "Old Pswd:");

            while (i < 4) {

                char key = keypad_getkey();

                if (key && key != '*' && key != '#' && key != 'A' && key != 'B') {

                    entered[i++] = key;

                    LcdFxn(1, '*');

                }

            }

            eeprom_read_password(stored);

            if (entered[0] == stored[0] && entered[1] == stored[1] &&

                entered[2] == stored[2] && entered[3] == stored[3]) {

                lprint(0x01, "");

                lprint(0x80, "New Pswd:");

                i = 0;

                while (i < 4) {

                    char key = keypad_getkey();

                    if (key && key != '*' && key != '#' && key != 'A' && key != 'B') {

                        newpass[i++] = key;

                        LcdFxn(1, '*');

                    }

                }

                eeprom_write_password(newpass);

                lprint(0xC0, "Updated");

                delay_ms(100);

                lprint(0xC0, "Successful");

                delay_ms(100);

            } else {

                lprint(0xC0, "Wrong Old Pswd");

                delay_ms(100);

                lprint(0xC0, " Access denied");

                delay_ms(100);


            }

        }

    }

}

