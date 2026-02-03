/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* i2c - Simple Example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected
   over I2C.

   The sensor used in this example is a MPU9250 inertial measurement unit.
*/
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <stdio.h>

static const char* TAG = "example";
#define pdUS_TO_TICKS(time_us)                                               \
    ((TickType_t)(((TickType_t)(time_us) * (TickType_t)configTICK_RATE_HZ) / \
                  (TickType_t)1000000U))

#define I2C_MASTER_SCL_IO \
    CONFIG_I2C_MASTER_SCL /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO \
    CONFIG_I2C_MASTER_SDA        /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0 /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ \
    CONFIG_I2C_MASTER_FREQUENCY     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000
#define SEN66_I2C_ADDR 0x6b /*!< Address of the SEN66 sensor */

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle;

int16_t sensirion_i2c_hal_read(uint8_t address, uint8_t* data, uint8_t count) {
    esp_err_t result =
        i2c_master_receive(dev_handle, data, count, I2C_MASTER_TIMEOUT_MS);
    return (int16_t)result;
}

int16_t sensirion_i2c_hal_write(uint8_t address, const uint8_t* data,
                                uint8_t count) {
    return i2c_master_transmit(dev_handle, data, count, I2C_MASTER_TIMEOUT_MS);
}

/**
 * @brief i2c master initialization
 */
static void i2c_master_init(i2c_master_bus_handle_t* bus_handle,
                            i2c_master_dev_handle_t* dev_handle) {
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SEN66_I2C_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(
        i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
}

/**
 * Initialize all hard- and software components that are needed for the I2C
 * communication.
 */
void sensirion_i2c_hal_init(void) {
    // Initialization is done in app_main for this example
    i2c_master_init(&bus_handle, &dev_handle);
    ESP_LOGI(TAG, "I2C initialized successfully");
}

/**
 * Release all resources initialized by sensirion_i2c_hal_init().
 */
void sensirion_i2c_hal_free(void) {
    // De-initialization is done in app_main for this example
    i2c_master_bus_rm_device(dev_handle);
    i2c_del_master_bus(bus_handle);
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}

void sensirion_i2c_hal_sleep_usec(uint32_t useconds) {
    // For this example, we use vTaskDelay which has a precision of 1 ms
    // For more precise delays, a hardware timer would be needed
    vTaskDelay(pdUS_TO_TICKS(useconds));
}
