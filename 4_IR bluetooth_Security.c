/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"



/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f405xx.h"
#include "lcd.h"
#include "FreeRTOS.h"
#include "string.h"
#include "task.h"
#include "spi.h"
#include <stdio.h>

#define I2C_DELAY()  for(volatile int i = 0; i < 30; i++) __NOP(); // ~5-10us delay

#define AIT1001_ADDR 0x0A

#define GREEN_LED_GPIO    GPIOA
#define GREEN_LED_PIN     GPIO_PIN_8

#define RED_LED_GPIO      GPIOA
#define RED_LED_PIN       GPIO_PIN_9

volatile float objectTempC = 0.0;

volatile float ambientTempC = 0.0;

uint16_t rawObjectValue = 0;

uint16_t rawAmbientValue = 0;

extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef hi2c2;  // Make sure this is declared

// ========== KEYPAD ==========
char keypad_map[4][4] = {
    {'1','2','3','A'},
    {'4','5','6','B'},
    {'7','8','9','C'},
    {'*','0','#','D'}
};
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


// ========== DELAY ==========
void delay_ms(uint32_t ms) {
    for (volatile uint32_t i = 0; i < ms * 8000; i++) {
        __asm__("nop");
    }
}
void keypad_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    for (int i = 2; i <= 5; i++) {
        GPIOB->MODER &= ~(3 << (2 * i));
        GPIOB->MODER |=  (1 << (2 * i));
        GPIOB->ODR |= (1 << i);
    }
    for (int i = 6; i <= 9; i++) {
        GPIOB->MODER &= ~(3 << (2 * i));
        GPIOB->PUPDR &= ~(3 << (2 * i));
        GPIOB->PUPDR |=  (1 << (2 * i));
    }
}

char keypad_getkey(void) {
    for (int row = 0; row < 4; row++) {
        GPIOB->ODR |= (0xF << 2);
        GPIOB->ODR &= ~(1 << (2 + row));
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

// ========== EEPROM ==========
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

uint16_t ait1001_read_reg(uint8_t reg_addr) {

    uint8_t buf[2] = {0};

    // Send register address
    HAL_I2C_Master_Transmit(&hi2c2, (AIT1001_ADDR << 1), &reg_addr, 1, HAL_MAX_DELAY);

    // Read 2 bytes from device
    HAL_I2C_Master_Receive(&hi2c2, (AIT1001_ADDR << 1) | 0x01, buf, 2, HAL_MAX_DELAY);

    return ((uint16_t)buf[0] << 8) | buf[1];

}

float AIT1001_ConvertRaw(uint16_t rawVal) {

    return ((float)rawVal) * 0.1;

}

void AIT1001_ReadTemps(void) {

	rawObjectValue = ait1001_read_reg(0x01);

	rawAmbientValue = ait1001_read_reg(0x00);

	objectTempC = AIT1001_ConvertRaw(rawObjectValue);

	ambientTempC = AIT1001_ConvertRaw(rawAmbientValue);

}

void vAIT1001_Task(void *pvParameters) {
    char line1[17];
    char line2[17];
    //char bt1_msg[64] = "Hello STM here";
    char bt_msg[64];
    while (1) {

    	AIT1001_ReadTemps();

        // printf("Obj Temp: %.1f C | Amb Temp: %.1f C\n", obj_temp, amb_temp);
        		int obj_int = (int)objectTempC;
                int obj_dec = (int)((objectTempC - obj_int) * 10);

                int amb_int = (int)ambientTempC;
                int amb_dec = (int)((ambientTempC - amb_int) * 10);

                // Format manually
                sprintf(line1, "Obj: %d.%d C     ", obj_int, obj_dec);
                sprintf(line2, "Amb: %d.%d C     ", amb_int, amb_dec);
                // Display on LCD
                if (objectTempC > 33.0f) {

                    lprint(0x80, "High Temp Alert!");
                    lprint(0xC0, line1);
                    HAL_GPIO_WritePin(RED_LED_GPIO, RED_LED_PIN, GPIO_PIN_SET);     // Red ON
                    HAL_GPIO_WritePin(GREEN_LED_GPIO, GREEN_LED_PIN, GPIO_PIN_RESET); // Green OFF
                } else {
                    lprint(0x80, line1);
                    lprint(0xC0, line2);
                    HAL_GPIO_WritePin(GREEN_LED_GPIO, GREEN_LED_PIN, GPIO_PIN_SET);   // Green ON
                    HAL_GPIO_WritePin(RED_LED_GPIO, RED_LED_PIN, GPIO_PIN_RESET);     // Red OFF
                }

                // Format Bluetooth message
                sprintf(bt_msg, "Object: %d.%d C | Ambient: %d.%d C\r\n",
                obj_int, obj_dec, amb_int, amb_dec);

                // Transmit over UART to HC-05
              HAL_UART_Transmit(&huart2, (uint8_t*)bt_msg, strlen(bt_msg), HAL_MAX_DELAY);

                vTaskDelay(pdMS_TO_TICKS(1000));

    }

}
void vAccessControlTask(void *params) {
    char entered[5] = {0}, stored[5] = {0}, newpass[5] = {0};
    uint8_t i = 0;

    eeprom_write_password("1357"); // Default password once

    while (1) {
        lprint(0x01, "");
        lprint(0x80, "Welcome user");
        lprint(0xC0, "A:Login B:Change");

        char mode = 0;
        while (!mode) mode = keypad_getkey();

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
            if (memcmp(entered, stored, 4) == 0) {
                lprint(0xC0, "Access Granted");
                vTaskDelay(pdMS_TO_TICKS(1000));

                // Start the IR Task and stop this task
                xTaskCreate(vAIT1001_Task, "AIT1001", 256, NULL, 1, NULL);
                vTaskDelete(NULL);  // Login task ends
            } else {
                lprint(0xC0, "Access Denied ");
                vTaskDelay(pdMS_TO_TICKS(1500));
            }
        }

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
            if (memcmp(entered, stored, 4) == 0) {
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
                vTaskDelay(pdMS_TO_TICKS(500));
                lprint(0xC0, "Successful");
                vTaskDelay(pdMS_TO_TICKS(500));
            } else {
                lprint(0xC0, "Wrong Old Pswd");
                vTaskDelay(pdMS_TO_TICKS(500));
                lprint(0xC0, "Access Denied");
                vTaskDelay(pdMS_TO_TICKS(500));
            }
        }
    }
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  //i2c_gpio_init();
  LcdInit();
  keypad_init();
  SpiEepromInit();

  xTaskCreate(vAccessControlTask, "Access", 256, NULL, 2, NULL);
         //xTaskCreate(vAIT1001_Task, "AIT1001", 256, NULL, 1, NULL);

         vTaskStartScheduler();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
