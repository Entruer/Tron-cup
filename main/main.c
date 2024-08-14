/****************************************************************************************
 * Includes
 ****************************************************************************************/

#include <stdio.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/****************************************************************************************
 * Constants
 ****************************************************************************************/

#define LED_PIN           GPIO_NUM_2

#define I2C_MASTER_PORT   I2C_NUM_0
#define I2C_MASTER_FREQ   100000
#define I2C_MASTER_SDA_IO GPIO_NUM_21
#define I2C_MASTER_SCL_IO GPIO_NUM_22

#define UART_PORT         UART_NUM_2
#define UART_TX_PIN       GPIO_NUM_17
#define UART_RX_PIN       GPIO_NUM_16
#define UART_BUFFER_SIZE  2048

/****************************************************************************************
 * Configuration
 ****************************************************************************************/

i2c_config_t i2c_config = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = I2C_MASTER_SDA_IO,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_io_num       = I2C_MASTER_SCL_IO,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ,
};

uart_config_t uart_config = {
        .baud_rate  = 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
};

/****************************************************************************************
 * Global Variables
 ****************************************************************************************/

float angle[3];
bool  is_angle_updated = false;

/****************************************************************************************
 * Function Definitions
 ****************************************************************************************/

uint8_t read_water_level()
{
  uint8_t data = 0;

  i2c_cmd_handle_t cmd_write = i2c_cmd_link_create();
  i2c_master_start(cmd_write);
  i2c_master_write_byte(cmd_write, 0x80, I2C_MASTER_ACK);
  i2c_master_write(cmd_write, &data, 1, I2C_MASTER_ACK);
  i2c_master_stop(cmd_write);
  i2c_master_cmd_begin(I2C_MASTER_PORT, cmd_write, 100);
  i2c_cmd_link_delete(cmd_write);

  vTaskDelay(pdMS_TO_TICKS(10));

  i2c_cmd_handle_t cmd_read = i2c_cmd_link_create();
  i2c_master_start(cmd_read);
  i2c_master_write_byte(cmd_read, 0x81, I2C_MASTER_ACK);
  i2c_master_read(cmd_read, &data, 1, I2C_MASTER_ACK);
  i2c_master_stop(cmd_read);
  i2c_master_cmd_begin(I2C_MASTER_PORT, cmd_read, 100);
  i2c_cmd_link_delete(cmd_read);

  return data;
}

void decode_imu_data(uint8_t chrTemp[])
{
  // Sum Check
  uint8_t sum = 0;
  for (int i = 0; i < 10; i++)
  {
    sum += chrTemp[i];
  }
  if (sum != chrTemp[10])
  {
    return;
  }

  // Decode
  if (chrTemp[0] == 0x55 && chrTemp[1] == 0x53)
  {
    angle[0] = ((uint16_t) (chrTemp[3] << 8 | chrTemp[2])) / 32768.0 * 180;
    angle[1] = ((uint16_t) (chrTemp[5] << 8 | chrTemp[4])) / 32768.0 * 180;
    angle[2] = ((uint16_t) (chrTemp[7] << 8 | chrTemp[6])) / 32768.0 * 180;
  }
}

void send_water_level(uint8_t data)
{
  ESP_LOGI("Position", "Position stable : %d", data);
}

/****************************************************************************************
 * Tasks
 ****************************************************************************************/

static void uart_task(void *args)
{
  uint8_t *data = (uint8_t *) malloc(UART_BUFFER_SIZE);
  while (1)
  {
    int len = uart_read_bytes(UART_PORT, data, 11, 10);
    if (len > 0)
    {
      is_angle_updated = true;
      decode_imu_data(data);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

static void position_check_task(void *args)
{
  unsigned int position_stable_count    = 0;
  unsigned int water_level_stable_count = 0;
  bool         is_water_level_updated   = false;
  float        angle_init[3]            = {0, 0, 0};
  uint8_t      water_level_init         = 0;
  uint8_t      water_level              = 0;

  // Wait for the first angle data
  while (!is_angle_updated)
  {
    vTaskDelay(pdMS_TO_TICKS(3000));
  }
  angle_init[0] = angle[0];
  angle_init[1] = angle[1];
  angle_init[2] = angle[2];

  while (1)
  {
    if ((angle_init[0] - angle[0] < 10 || angle_init[0] - angle[0] > 350) && (angle_init[1] - angle[1] < 10 || angle_init[1] - angle[1] > 350))
    {
      position_stable_count++;
    }
    else
    {
      position_stable_count    = 0;
      water_level_stable_count = 0;
    }

    if (position_stable_count < 3)
    {
      gpio_set_level(LED_PIN, 0);
      vTaskDelay(pdMS_TO_TICKS(3000));
      continue;
    }

    if (!is_water_level_updated)
    {
      water_level_init       = read_water_level();
      is_water_level_updated = true;
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    water_level = read_water_level();
    if (abs(water_level_init - water_level) < 5)
    {
      water_level_stable_count++;
    }
    else
    {
      water_level_stable_count = 0;
    }

    if (water_level_stable_count < 3)
    {
      gpio_set_level(LED_PIN, 0);
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    send_water_level(water_level);
    position_stable_count = 0;
    water_level_stable_count = 0;
    gpio_set_level(LED_PIN, 1);

    vTaskDelay(pdMS_TO_TICKS(3000));
  }
}

/****************************************************************************************
 * Main
 ****************************************************************************************/

void app_main(void)
{
  ESP_ERROR_CHECK(gpio_reset_pin(LED_PIN));
  ESP_ERROR_CHECK(gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT));

  ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_PORT, I2C_MODE_MASTER, 0, 0, 0));
  ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_PORT, &i2c_config));

  ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_BUFFER_SIZE, 0, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  xTaskCreate(uart_task, "uart_task", 2048, NULL, 10, NULL);
  xTaskCreate(position_check_task, "position_check_task", 2048, NULL, 10, NULL);

  while (1)
  {
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}
