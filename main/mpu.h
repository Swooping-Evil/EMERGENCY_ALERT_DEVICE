/**
 * This header defines the API for interacting with the MPU6050
 * accelerometer and gyroscope sensor.
 */

#ifndef MPU_H
#define MPU_H

#include "esp_err.h"

// Accelerometer data structure
typedef struct {
    int16_t x;        // Raw X-axis accelerometer value
    int16_t y;        // Raw Y-axis accelerometer value
    int16_t z;        // Raw Z-axis accelerometer value
    float x_g;        // X-axis acceleration in g units
    float y_g;        // Y-axis acceleration in g units
    float z_g;        // Z-axis acceleration in g units
} mpu_accel_data_t;

// Gyroscope data structure
typedef struct {
    int16_t x;        // Raw X-axis gyroscope value
    int16_t y;        // Raw Y-axis gyroscope value
    int16_t z;        // Raw Z-axis gyroscope value
    float x_dps;      // X-axis angular velocity in degrees per second
    float y_dps;      // Y-axis angular velocity in degrees per second
    float z_dps;      // Z-axis angular velocity in degrees per second
} mpu_gyro_data_t;

/**
 * Initialize the MPU sensor
 *
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t mpu_init(void);

/**
 * Read accelerometer data from MPU
 *
 * @param accel_data Pointer to store the accelerometer data
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t mpu_read_accel(mpu_accel_data_t *accel_data);

/**
 * Read gyroscope data from MPU
 *
 * @param gyro_data Pointer to store the gyroscope data
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t mpu_read_gyro(mpu_gyro_data_t *gyro_data);

/**
 * Get the last known accelerometer data
 *
 * @param accel_data Pointer to store the accelerometer data
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t mpu_get_last_accel(mpu_accel_data_t *accel_data);

/**
 * Get the last known gyroscope data
 *
 * @param gyro_data Pointer to store the gyroscope data
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t mpu_get_last_gyro(mpu_gyro_data_t *gyro_data);

#endif /* MPU_H */
