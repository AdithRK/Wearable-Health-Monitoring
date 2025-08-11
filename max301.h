#ifndef MAX30100_H
#define MAX30100_H
 
#include "stm32f4xx_hal.h"
 
#define MAX30102_I2C_ADDR         (0x57 << 1)
#define REG_MODE_CONFIG     0x09
#define REG_SPO2_CONFIG     0x0A
#define REG_LED1_PA         0x0C  // IR
#define REG_LED2_PA         0x0D  // Red
 
 
void MAX30102_Init(I2C_HandleTypeDef *hi2c);
void MAX30102_Update(I2C_HandleTypeDef *hi2c);
uint32_t MAX30102_GetIRRaw(void);
uint32_t MAX30102_GetRedRaw(void);
 
float calculateHeartRate(uint32_t *data, uint16_t len);
float calculateSpO2(uint32_t *ir_data, uint32_t *red_data, uint16_t len);
float get_average(uint32_t *data, uint16_t len);
float get_peak_to_peak(uint32_t *data, uint16_t len);
 
//uint64_t calculateSpO2new(uint32_t *ir_data, uint32_t *red_data, uint16_t len);
//uint64_t calculateHeartRatenew(uint32_t *data, uint16_t len);
 
 
#endif