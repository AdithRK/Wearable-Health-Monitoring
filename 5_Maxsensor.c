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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f405xx.h"
#include "lcd.h"
#include "FreeRTOS.h"
#include "string.h"
#include "task.h"

#include "max301.h"

#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

uint32_t lastReportTick = 0;
//float heartRate = 0, spo2 = 0;
extern I2C_HandleTypeDef hi2c2;

#define MAX_SAMPLES 500
uint32_t ir_buffer[MAX_SAMPLES];
uint32_t red_buffer[MAX_SAMPLES];
uint16_t sample_index = 0;

volatile float hr = 0.0f;
volatile float spo = 0.0f;
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

void vMaxTask(void *pvParameters);
void vCheckTask(void *pvParameters);
void vSensorTask(void *pvParameters);

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
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  LcdInit();
  HAL_Delay(100);
  lprint(0x80, "starting");
  MAX30102_Init(&hi2c2);
  lprint(0x80, "max ready");

  //starting_show1();
  //starting_show2();

  //xTaskCreate(vMaxTask, "max30100", 256, NULL, 2, NULL);
  xTaskCreate(vSensorTask, "max30100", 256, NULL, 1, NULL);

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

//void vCheckTask(void *pvParameters){
//	uint8_t part_id = 0;
//
//	if (HAL_I2C_Mem_Read(&hi2c2, MAX30102_I2C_ADDR, 0xFF, I2C_MEMADD_SIZE_8BIT, &part_id, 1, HAL_MAX_DELAY) == HAL_OK) {
//	    if (part_id == 0x15) {
//	        lprint(0x80, "MAX301 detected");
//	    } else {
//	        lprint(0x80, "Wrong device");
//	    }
//	} else {
//	    lprint(0x80, "I2C read failed");
//	}
//
//}

void vMaxTask(void *pvParameters) {
    while (1) {
        MAX30102_Update(&hi2c2);  // Read from sensor

        if (HAL_GetTick() - lastReportTick > 1000) {
            char msg1[16], msg2[16];

            uint32_t ir = MAX30102_GetIRRaw();
            uint32_t red = MAX30102_GetRedRaw();

            snprintf(msg1, sizeof(msg1), "IR: %lu", ir);
            snprintf(msg2, sizeof(msg2), "RED: %lu", red);

            LcdFxn(0, 0x01);      // Clear LCD
            lprint(0x80, msg1);   // Print IR
            lprint(0xC0, msg2);   // Print RED

            lastReportTick = HAL_GetTick();
        }
    }
}
/* SpO2 Calculation Task ----------------------------------------------------*/
void vSensorTask(void *pvParameters) {
    while (1) {
        // Step 1: Read current sample
        MAX30102_Update(&hi2c2);
        uint32_t ir = MAX30102_GetIRRaw();
        uint32_t red = MAX30102_GetRedRaw();

        // Step 2: Display live IR/RED values on LCD every 1s
        if (HAL_GetTick() - lastReportTick > 1000) {
            char msg1[16], msg2[16];
            snprintf(msg1, sizeof(msg1), "IR: %lu", ir);
            snprintf(msg2, sizeof(msg2), "RED: %lu", red);

            LcdFxn(0, 0x01);
            lprint(0x80, msg1);
            lprint(0xC0, msg2);

            lastReportTick = HAL_GetTick();
        }

        // Step 3: Buffer samples for HR/SpO2 calculation
        if (sample_index < MAX_SAMPLES) {
            ir_buffer[sample_index] = ir;
            red_buffer[sample_index] = red;
            sample_index++;
        }

        // Step 4: When buffer full, compute and store HR/SpO2
        if (sample_index >= MAX_SAMPLES) {
           hr = calculateHeartRate(ir_buffer, MAX_SAMPLES);
           spo = calculateSpO2(ir_buffer, red_buffer, MAX_SAMPLES);


            sample_index = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // 100 Hz sampling
    }
}



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
