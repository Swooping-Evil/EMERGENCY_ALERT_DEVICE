/**
 * This module handles the MPU6050 accelerometer/gyroscope sensor communications
 * and provides raw motion data for further processing.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "esp_err.h"

#include "mpu.h"

#define TAG "MPU"

// MPU6050 registers
#define MPU6050_ADDR          0x68    // I2C address of the MPU6050
#define MPU_REG_POWER         0x6B    // Power management register
#define MPU_REG_GYRO_CONFIG   0x1B    // Gyroscope configuration register
#define MPU_REG_ACCEL_CONFIG  0x1C    // Accelerometer configuration register
#define MPU_REG_INT_ENABLE    0x38    // Interrupt enable register
#define MPU_REG_INT_PIN_CFG   0x37    // Interrupt pin configuration
#define MPU_REG_MOT_DETECT    0x69    // Motion detection control register
#define MPU_REG_MOT_THRESH    0x1F    // Motion detection threshold
#define MPU_REG_ACCEL_XOUT_H  0x3B    // Accelerometer X-axis high byte
#define MPU_REG_GYRO_XOUT_H   0x43    // Gyroscope X-axis high byte

// I2C configuration
#define I2C_MASTER_SCL_IO     22      // GPIO for I2C SCL
#define I2C_MASTER_SDA_IO     21      // GPIO for I2C SDA
#define I2C_MASTER_NUM        I2C_NUM_0
#define I2C_MASTER_FREQ_HZ    400000  // I2C master clock frequency
#define I2C_TIMEOUT_MS        1000

// Global variables to hold the latest sensor readings
static mpu_accel_data_t last_accel_data = {0};
static mpu_gyro_data_t last_gyro_data = {0};

/**
 * Write a byte to an MPU register
 */
static esp_err_t mpu_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, write_buf, sizeof(write_buf), true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

/**
 * Read bytes from MPU registers
 */
static esp_err_t mpu_register_read(uint8_t reg_addr, uint8_t *data_buf, size_t len)
{
    if (len == 0) {
        return ESP_OK;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);

    if (len > 1) {
        i2c_master_read(cmd, data_buf, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data_buf + len - 1, I2C_MASTER_NACK);

    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

/**
 * Initialize I2C communication for MPU
 */
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C parameter configuration failed");
        return ret;
    }

    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver installation failed");
        return ret;
    }

    return ESP_OK;
}

/**
 * Configure MPU for motion detection
 */
static esp_err_t configure_mpu_motion_detection(void)
{
    esp_err_t ret;

    // Configure motion detection threshold
    ret = mpu_register_write_byte(MPU_REG_MOT_THRESH, 10); // Threshold value (adjust based on sensitivity needs)
    if (ret != ESP_OK) return ret;

    // Enable motion detection interrupt
    ret = mpu_register_write_byte(MPU_REG_MOT_DETECT, 0x40); // Enable motion detection
    if (ret != ESP_OK) return ret;

    // Configure interrupt behavior
    ret = mpu_register_write_byte(MPU_REG_INT_PIN_CFG, 0x10); // INT pin as push-pull, active high
    if (ret != ESP_OK) return ret;

    // Enable motion interrupt
    ret = mpu_register_write_byte(MPU_REG_INT_ENABLE, 0x40); // Enable motion interrupt
    if (ret != ESP_OK) return ret;

    return ESP_OK;
}

/**
 * Initialize the MPU sensor
 */
esp_err_t mpu_init(void)
{
    esp_err_t ret;

    // Initialize I2C
    ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C master initialization failed");
        return ret;
    }

    // Wake up the MPU6050 (it starts in sleep mode)
    ret = mpu_register_write_byte(MPU_REG_POWER, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake up MPU");
        return ret;
    }

    // Configure gyroscope range (±250°/s)
    ret = mpu_register_write_byte(MPU_REG_GYRO_CONFIG, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure gyroscope");
        return ret;
    }

    // Configure accelerometer range (±2g)
    ret = mpu_register_write_byte(MPU_REG_ACCEL_CONFIG, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure accelerometer");
        return ret;
    }

    // Configure motion detection
    ret = configure_mpu_motion_detection();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure motion detection");
        return ret;
    }

    ESP_LOGI(TAG, "MPU initialized successfully");
    return ESP_OK;
}

/**
 * Read accelerometer data from MPU
 */
esp_err_t mpu_read_accel(mpu_accel_data_t *accel_data)
{
    uint8_t data[6];
    esp_err_t ret = mpu_register_read(MPU_REG_ACCEL_XOUT_H, data, 6);

    if (ret == ESP_OK) {
        // Combine high and low bytes to form 16-bit values
        accel_data->x = (int16_t)((data[0] << 8) | data[1]);
        accel_data->y = (int16_t)((data[2] << 8) | data[3]);
        accel_data->z = (int16_t)((data[4] << 8) | data[5]);

        // Store the readings globally for later use
        last_accel_data = *accel_data;

        // Convert to g units (±2g range means 16384 LSB/g)
        accel_data->x_g = accel_data->x / 16384.0f;
        accel_data->y_g = accel_data->y / 16384.0f;
        accel_data->z_g = accel_data->z / 16384.0f;
    } else {
        ESP_LOGE(TAG, "Failed to read accelerometer data");
    }

    return ret;
}

/**
 * Read gyroscope data from MPU
 */
esp_err_t mpu_read_gyro(mpu_gyro_data_t *gyro_data)
{
    uint8_t data[6];
    esp_err_t ret = mpu_register_read(MPU_REG_GYRO_XOUT_H, data, 6);

    if (ret == ESP_OK) {
        // Combine high and low bytes to form 16-bit values
        gyro_data->x = (int16_t)((data[0] << 8) | data[1]);
        gyro_data->y = (int16_t)((data[2] << 8) | data[3]);
        gyro_data->z = (int16_t)((data[4] << 8) | data[5]);

        // Store the readings globally for later use
        last_gyro_data = *gyro_data;

        // Convert to degrees per second (±250°/s range means 131 LSB/°/s)
        gyro_data->x_dps = gyro_data->x / 131.0f;
        gyro_data->y_dps = gyro_data->y / 131.0f;
        gyro_data->z_dps = gyro_data->z / 131.0f;
    } else {
        ESP_LOGE(TAG, "Failed to read gyroscope data");
    }

    return ret;
}

/**
 * Get the last known accelerometer data
 */
esp_err_t mpu_get_last_accel(mpu_accel_data_t *accel_data)
{
    if (accel_data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *accel_data = last_accel_data;
    return ESP_OK;
}

/**
 * Get the last known gyroscope data
 */
esp_err_t mpu_get_last_gyro(mpu_gyro_data_t *gyro_data)
{
    if (gyro_data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *gyro_data = last_gyro_data;
    return ESP_OK;
}
