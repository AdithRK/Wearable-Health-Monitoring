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
#include "usart.h"
#include "gpio.h"
#include "stm32f405xx.h"
#include "lcd.h"
#include "FreeRTOS.h"
#include "string.h"
#include "task.h"

#include <stdio.h>

#define SDA_PIN 11  // PB11

#define SCL_PIN 10  // PB10

#define I2C_DELAY()  for(volatile int i = 0; i < 30; i++) __NOP(); // ~5-10us delay

#define AIT1001_ADDR 0x0A

volatile float objectTempC = 0.0;

volatile float ambientTempC = 0.0;

uint16_t rawObjectValue = 0;

uint16_t rawAmbientValue = 0;

extern UART_HandleTypeDef huart2;
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Bit-banged I2C delay (~5us)

// Basic ~5µs delay (tuned for ~100kHz)

// ================= I2C GPIO Init =================

// Set SDA as output and drive low

void SDA_LOW(void) {

    GPIOB->MODER &= ~(3 << (2 * SDA_PIN));

    GPIOB->MODER |=  (1 << (2 * SDA_PIN));

    GPIOB->ODR &= ~(1 << SDA_PIN);

}

void SDA_HIGH(void) {

    GPIOB->MODER &= ~(3 << (2 * SDA_PIN));

}

void SCL_LOW(void) {

    GPIOB->MODER &= ~(3 << (2 * SCL_PIN));

    GPIOB->MODER |=  (1 << (2 * SCL_PIN));

    GPIOB->ODR &= ~(1 << SCL_PIN);

}

void SCL_HIGH(void) {

    GPIOB->MODER &= ~(3 << (2 * SCL_PIN));

}

uint8_t READ_SDA(void) {

    return (GPIOB->IDR & (1 << SDA_PIN)) ? 1 : 0;

}

void i2c_gpio_init(void) {

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    GPIOB->MODER &= ~((3 << (2 * SDA_PIN)) | (3 << (2 * SCL_PIN)));

    GPIOB->PUPDR &= ~((3 << (2 * SDA_PIN)) | (3 << (2 * SCL_PIN)));

    GPIOB->OTYPER |= (1 << SDA_PIN) | (1 << SCL_PIN);

    GPIOB->OSPEEDR |= (3 << (2 * SDA_PIN)) | (3 << (2 * SCL_PIN));

    GPIOB->ODR |= (1 << SDA_PIN) | (1 << SCL_PIN);

    SDA_HIGH();

    SCL_HIGH();

}

void i2c_start(void) {

    SDA_HIGH(); SCL_HIGH(); I2C_DELAY();

    SDA_LOW();  I2C_DELAY();

    SCL_LOW();  I2C_DELAY();

}

void i2c_stop(void) {

    SDA_LOW();  I2C_DELAY();

    SCL_HIGH(); I2C_DELAY();

    SDA_HIGH(); I2C_DELAY();

}

uint8_t i2c_write_byte(uint8_t byte) {

    for (int i = 0; i < 8; i++) {

        (byte & 0x80) ? SDA_HIGH() : SDA_LOW();

        I2C_DELAY();

        SCL_HIGH(); I2C_DELAY();

        SCL_LOW();  I2C_DELAY();

        byte <<= 1;

    }

    SDA_HIGH(); I2C_DELAY();

    SCL_HIGH(); I2C_DELAY();

    uint8_t ack = READ_SDA();

    SCL_LOW(); I2C_DELAY();

    return ack;

}

uint8_t i2c_read_byte(uint8_t ack) {

    uint8_t data = 0;

    SDA_HIGH();

    for (int i = 0; i < 8; i++) {

        SCL_HIGH(); I2C_DELAY();

        data <<= 1;

        if (READ_SDA()) data |= 1;

        SCL_LOW(); I2C_DELAY();

    }

    (ack) ? SDA_LOW() : SDA_HIGH();

    I2C_DELAY();

    SCL_HIGH(); I2C_DELAY();

    SCL_LOW(); I2C_DELAY();

    SDA_HIGH();

    return data;

}

uint16_t ait1001_read_reg(uint8_t reg_addr) {

    uint8_t msb, lsb;

    i2c_start();

    i2c_write_byte(AIT1001_ADDR << 1);

    i2c_write_byte(reg_addr);

    i2c_start();

    i2c_write_byte((AIT1001_ADDR << 1) | 0x01);

    msb = i2c_read_byte(1);

    lsb = i2c_read_byte(0);

    i2c_stop();

    return ((uint16_t)msb << 8) | lsb;

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
    char bt1_msg[64] = "Hello STM here";
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
                } else {
                    lprint(0x80, line1);
                    lprint(0xC0, line2);
                }

                // Format Bluetooth message
                sprintf(bt_msg, "Object: %d.%d C | Ambient: %d.%d C\r\n",
                obj_int, obj_dec, amb_int, amb_dec);

                // Transmit over UART to HC-05
//                HAL_UART_Transmit(&huart2, (uint8_t*)bt1_msg, strlen(bt1_msg), HAL_MAX_DELAY);
              HAL_UART_Transmit(&huart2, (uint8_t*)bt_msg, strlen(bt_msg), HAL_MAX_DELAY);

                vTaskDelay(pdMS_TO_TICKS(1000));

    }

}


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// GPIO configuration for bit-bang I2C


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
  /* USER CODE BEGIN 2 */
  i2c_gpio_init();
  LcdInit();
       xTaskCreate(vAIT1001_Task, "AIT1001", 256, NULL, 1, NULL);

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
