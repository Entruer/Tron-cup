/****************************************************************************************
 * Includes
 ****************************************************************************************/

#include <stdio.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_log.h"
#include "esp_spp_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

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

float    angle[3];
bool     is_angle_updated = false;
bool     client_connected = false;
uint32_t spp_handle       = 0;

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
  if (client_connected)
  {
    esp_spp_write(spp_handle, 1, &data);
  }
}

/****************************************************************************************
 * Callbacks
 ****************************************************************************************/

void gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
  switch (event)
  {
    case ESP_BT_GAP_AUTH_CMPL_EVT:
      if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS)
      {
        ESP_LOGI("Bluetooth", "authentication success.");
      }
      else
      {
        ESP_LOGE("Bluetooth", "authentication failed, status:%d", param->auth_cmpl.stat);
      }
      break;
    case ESP_BT_GAP_CFM_REQ_EVT:
      ESP_LOGI("Bluetooth", "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %" PRIu32, param->cfm_req.num_val);
      esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
      break;
    default:
      break;
  }
  return;
}

void spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
  switch (event)
  {
    case ESP_SPP_INIT_EVT:
      ESP_LOGI("Bluetooth", "ESP_SPP_INIT_EVT");
      ESP_ERROR_CHECK(esp_spp_start_srv(ESP_SPP_SEC_AUTHENTICATE, ESP_SPP_ROLE_SLAVE, 0, "SPP_SERVER"));
      break;
    case ESP_SPP_START_EVT:
      if (param->start.status == ESP_SPP_SUCCESS)
      {
        ESP_LOGI("Bluetooth", "ESP_SPP_START_EVT handle:%" PRIu32 " sec_id:%d scn:%d", param->start.handle, param->start.sec_id,
                 param->start.scn);
        esp_bt_gap_set_device_name("Pawpaw");
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
      }
      else
      {
        ESP_LOGE("Bluetooth", "ESP_SPP_START_EVT status:%d", param->start.status);
      }
      break;
    case ESP_SPP_DATA_IND_EVT:
      ESP_LOGI("Bluetooth", "ESP_SPP_DATA_IND_EVT len:%d handle:%" PRIu32,
               param->data_ind.len, param->data_ind.handle);
      if (param->data_ind.len < 128)
      {
        ESP_LOG_BUFFER_HEX("", param->data_ind.data, param->data_ind.len);
      }
      break;
    case ESP_SPP_CONG_EVT:
      ESP_LOGI("Bluetooth", "ESP_SPP_CONG_EVT");
      break;
    case ESP_SPP_WRITE_EVT:
      ESP_LOGI("Bluetooth", "ESP_SPP_WRITE_EVT");
      break;
    case ESP_SPP_SRV_OPEN_EVT:
      ESP_LOGI("Bluetooth", "ESP_SPP_SRV_OPEN_EVT status:%d ", param->srv_open.status);
      client_connected = true;
      spp_handle       = param->srv_open.handle;
      break;
    case ESP_SPP_SRV_STOP_EVT:
      ESP_LOGI("Bluetooth", "ESP_SPP_SRV_STOP_EVT");
      client_connected = false;
      break;
    default:
      break;
  }
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
    position_stable_count    = 0;
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

  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
  esp_bt_controller_config_t bt_cfg        = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  esp_bluedroid_config_t     bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
  esp_spp_cfg_t              spp_cfg       = {
                             .mode              = ESP_SPP_MODE_CB,
                             .enable_l2cap_ertm = true,
                             .tx_buffer_size    = 0,
  };
  ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
  ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));
  ESP_ERROR_CHECK(esp_bluedroid_init_with_cfg(&bluedroid_cfg));
  ESP_ERROR_CHECK(esp_bluedroid_enable());
  ESP_ERROR_CHECK(esp_bt_gap_register_callback(gap_callback));
  ESP_ERROR_CHECK(esp_spp_register_callback(spp_callback));
  ESP_ERROR_CHECK(esp_spp_enhanced_init(&spp_cfg));

  // Set parameters of SSP
  esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
  esp_bt_io_cap_t   iocap      = ESP_BT_IO_CAP_IO;
  esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));

  // Set parameters of legacy pairing
  // esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
  // esp_bt_pin_code_t pin_code;
  // esp_bt_gap_set_pin(pin_type, 0, pin_code);

  xTaskCreate(uart_task, "uart_task", 2048, NULL, 10, NULL);
  xTaskCreate(position_check_task, "position_check_task", 2048, NULL, 10, NULL);

  while (1)
  {
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}
