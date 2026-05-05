#include "VEML7700_driver.h"

void VEML7700_Set_Gain(I2C_HandleTypeDef *hi2c, uint16_t gain, uint16_t integration_time) {
    uint8_t conf_buf[2];
    uint16_t conf;

    // Put sensor in standby for changing parameters
    conf = VEML7700_ALS_SD_STANDBY;
    conf_buf[0] = conf & 0xFF;
    conf_buf[1] = (conf >> 8) & 0xFF;
    HAL_I2C_Mem_Write(hi2c, VEML7700_ADDR,
                      VEML7700_REG_CONF, I2C_MEMADD_SIZE_8BIT,
                      conf_buf, 2, VEML7700_TIMEOUT);

    // Write the needed changes
    conf = gain |integration_time | VEML7700_ALS_SD_ON;
    conf_buf[0] = conf & 0xFF;
    conf_buf[1] = (conf >> 8) & 0xFF;
    HAL_I2C_Mem_Write(hi2c, VEML7700_ADDR,
                      VEML7700_REG_CONF, I2C_MEMADD_SIZE_8BIT,
                      conf_buf, 2, VEML7700_TIMEOUT);
}
