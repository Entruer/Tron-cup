#include <stdio.h>

#include "driver/i2c.h"
#include "esp_log.h"


#define I2C_MASTER_PORT   I2C_NUM_0
#define I2C_MASTER_FREQ   400000
#define I2C_MASTER_SCL_IO 21
#define I2C_MASTER_SDA_IO 22

i2c_config_t i2c_config = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = I2C_MASTER_SDA_IO,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_io_num       = I2C_MASTER_SCL_IO,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ,
};

void app_main(void)
{
  while (1)
  {
  }
}
