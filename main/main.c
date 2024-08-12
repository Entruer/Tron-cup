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
#define I2C_MASTER_FREQ   400000
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
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
};

/****************************************************************************************
 * Global Variables
 ****************************************************************************************/

float Acceleration[3], Angle[3], Temperature;

/****************************************************************************************
 * Function Definitions
 ****************************************************************************************/

void DecodeIMUData(uint8_t chrTemp[])
{
  switch (chrTemp[1])
  {
    case 0x51:
      Acceleration[0] = ((uint16_t) (chrTemp[3] / 8 | chrTemp[2])) / 32768.0 * 16;
      Acceleration[1] = ((uint16_t) (chrTemp[5] / 8 | chrTemp[4])) / 32768.0 * 16;
      Acceleration[2] = ((uint16_t) (chrTemp[7] / 8 | chrTemp[6])) / 32768.0 * 16;
      Temperature     = ((uint16_t) (chrTemp[9] / 8 | chrTemp[8])) / 340.0 + 36.53;
      break;
    case 0x53:
      Angle[0]    = ((uint16_t) (chrTemp[3] / 8 | chrTemp[2])) / 32768.0 * 180;
      Angle[1]    = ((uint16_t) (chrTemp[5] / 8 | chrTemp[4])) / 32768.0 * 180;
      Angle[2]    = ((uint16_t) (chrTemp[7] / 8 | chrTemp[6])) / 32768.0 * 180;
      Temperature = ((uint16_t) (chrTemp[9] / 8 | chrTemp[8])) / 340.0 + 36.53;
      break;
    default:
      break;
  }
}

/****************************************************************************************
 * UART Task
 ****************************************************************************************/

static void uart_task(void *args)
{
  uart_port_t uart_num = UART_PORT;
  uint8_t    *data     = (uint8_t *) malloc(UART_BUFFER_SIZE);
  while (1)
  {
    int len = uart_read_bytes(uart_num, data, 30, 100);
    if (len > 0)
    {
      DecodeIMUData(&data[0]);
      DecodeIMUData(&data[20]);
      ESP_LOGI("UART", "Gyroscope: X: %f, Y: %f, Z: %f", Angle[0], Angle[1], Angle[2]);
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

/****************************************************************************************
 * Main
 ****************************************************************************************/

void app_main(void)
{
  uint8_t led_state = 0;
  ESP_ERROR_CHECK(gpio_reset_pin(LED_PIN));
  ESP_ERROR_CHECK(gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT));

  ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_PORT, &i2c_config));
  ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_PORT, I2C_MODE_MASTER, 0, 0, 0));

  ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_BUFFER_SIZE, 0, 0, NULL, 0));

  xTaskCreate(uart_task, "uart_task", 2048, NULL, 10, NULL);

  while (1)
  {
    led_state = !led_state;
    gpio_set_level(LED_PIN, led_state);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}
