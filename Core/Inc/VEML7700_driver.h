#include "main.h"



#define VEML7700_ADDR       (0x10 << 1)  // 0x20 for HAL (8-bit)
#define VEML7700_REG_CONF   0x00
#define VEML7700_REG_ALS    0x04
#define VEML7700_REG_WHITE  0x05

// Gain options
#define VEML7700_GAIN_1     0x0000  // x1
#define VEML7700_GAIN_2     0x0800  // x2
#define VEML7700_GAIN_1_8   0x1000  // x1/8
#define VEML7700_GAIN_1_4   0x1800  // x1/4

// Integration time options
#define VEML7700_IT_25MS    0x0300
#define VEML7700_IT_100MS    0x0000


#define VEML7700_ALS_SD_ON      0x0000  // bit 0 = 0, sensor active
#define VEML7700_ALS_SD_STANDBY 0x0001  // bit 0 = 1, sensor standby

