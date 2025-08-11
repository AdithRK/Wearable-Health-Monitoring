#include "max301.h"
 
#include <stdlib.h>
#include <math.h>
 
static uint16_t ir_raw = 0, red_raw = 0;
static float heartRate = 75.0f;
static float spO2 = 98.0f;
 
 
void MAX30102_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t data;
 
    //fifo reset
    uint8_t fifo_reset = 0;
    HAL_I2C_Mem_Write(hi2c, MAX30102_I2C_ADDR, 0x04, I2C_MEMADD_SIZE_8BIT, &fifo_reset, 1, HAL_MAX_DELAY); // WR_PTR
    HAL_I2C_Mem_Write(hi2c, MAX30102_I2C_ADDR, 0x05, I2C_MEMADD_SIZE_8BIT, &fifo_reset, 1, HAL_MAX_DELAY); // OVF_CNT
    HAL_I2C_Mem_Write(hi2c, MAX30102_I2C_ADDR, 0x06, I2C_MEMADD_SIZE_8BIT, &fifo_reset, 1, HAL_MAX_DELAY); // RD_PTR
 
 
    // Step 1: Reset (optional)
    data = 0x40;  // Reset bit
    HAL_I2C_Mem_Write(hi2c, MAX30102_I2C_ADDR , REG_MODE_CONFIG, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
    HAL_Delay(10);  // Small delay after reset
 
    // Step 2: Set mode to SpO2 (0x03) or Multi-LED (0x07)
    data = 0x03;
    HAL_I2C_Mem_Write(hi2c, MAX30102_I2C_ADDR , REG_MODE_CONFIG, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
 
    // Step 3: Configure SPO2 (100 Hz sample rate, 411us pulse width)
    data = 0x27;
    HAL_I2C_Mem_Write(hi2c, MAX30102_I2C_ADDR , REG_SPO2_CONFIG, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
 
    // Step 4: Set LED currents — typical is 0x1F (6.4 mA)
    data = 0x1F; // IR LED
    HAL_I2C_Mem_Write(hi2c,MAX30102_I2C_ADDR , REG_LED1_PA, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
    data = 0x1F; // RED LED
    HAL_I2C_Mem_Write(hi2c, MAX30102_I2C_ADDR , REG_LED2_PA, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}
 
void MAX30102_Update(I2C_HandleTypeDef *hi2c) {
    uint8_t data[6];
 
    if (HAL_I2C_Mem_Read(hi2c, MAX30102_I2C_ADDR, 0x07, I2C_MEMADD_SIZE_8BIT, data, 6, HAL_MAX_DELAY) == HAL_OK) {
        ir_raw = ((uint32_t)(data[0] & 0x03) << 16) | ((uint32_t)data[1] << 8) | data[2];
        red_raw = ((uint32_t)(data[3] & 0x03) << 16) | ((uint32_t)data[4] << 8) | data[5];
 
        // Simulated values (replace with real algorithm later)
        heartRate = 75.0 + (rand() % 5 - 2); // 73–77 bpm
        spO2 = 97.5 + (rand() % 4 - 2);      // 95.5–99.5%
    }
}
 
uint32_t MAX30102_GetIRRaw(void) {
    return ir_raw;
}
 
uint32_t MAX30102_GetRedRaw(void) {
    return red_raw;
}
 
float calculateHeartRate(uint32_t *data, uint16_t len)
{
    int peaks = 0;
    for (int i = 1; i < len - 1; i++)
    {
        if (data[i] > data[i - 1] && data[i] > data[i + 1] && data[i] > 10000)
            peaks++;
    }
    float bpm = (peaks * 60.0f) / 5.0f; // 5 seconds buffer
    return bpm/10;
}
float calculateSpO2(uint32_t *ir_data, uint32_t *red_data, uint16_t len)
{
    float ir_dc = get_average(ir_data, len);
    float red_dc = get_average(red_data, len);
 
    float ir_ac = get_peak_to_peak(ir_data, len);
    float red_ac = get_peak_to_peak(red_data, len);
 
    if (ir_dc == 0 || red_dc == 0) return 0;
 
    float R = (red_ac / red_dc) / (ir_ac / ir_dc);
    float spo2 = 110.0f - 25.0f * R;
    return spo2;
}
float get_average(uint32_t *data, uint16_t len)
{
    uint64_t sum = 0;
    for (int i = 0; i < len; i++)
        sum += data[i];
    return (float)sum / len;
}
 
float get_peak_to_peak(uint32_t *data, uint16_t len)
{
    uint32_t min = data[0], max = data[0];
    for (int i = 1; i < len; i++)
    {
        if (data[i] < min) min = data[i];
        if (data[i] > max) max = data[i];
    }
    return (float)(max - min);
}