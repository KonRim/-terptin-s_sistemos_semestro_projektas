#include "main.h"

// Coefficients for lux adjustment formula above 1000 lux
#define VEML7700_COEF_A  6.0135e-13f
#define VEML7700_COEF_B -9.3924e-09f
#define VEML7700_COEF_C  8.1488e-05f
#define VEML7700_COEF_D  1.0023

// I2C sensor address and main registers
#define VEML7700_ADDR       (0x10 << 1)  // 0x20 for HAL (8-bit)
#define VEML7700_REG_CONF   0x00
#define VEML7700_REG_ALS    0x04

// Sensor gain options
#define VEML7700_GAIN_1     0x0000  // x1
#define VEML7700_GAIN_2     0x0800  // x2
#define VEML7700_GAIN_1_8   0x1000  // x1/8
#define VEML7700_GAIN_1_4   0x1800  // x1/4

// Sensor integration time options
#define VEML7700_IT_25MS    0x0300
#define VEML7700_IT_100MS    0x0000

// Sensor shutdown and activate bits (because we need to shutdown before changing gain/integration parameters)
#define VEML7700_ALS_SD_ON      0x0000
#define VEML7700_ALS_SD_STANDBY 0x0001

//Max wait time for sensor I2C write commands
#define VEML7700_TIMEOUT 10

// Functions for VEML7700.c
void VEML7700_Set_Gain(I2C_HandleTypeDef *hi2c, uint16_t gain, uint16_t integration_time);

