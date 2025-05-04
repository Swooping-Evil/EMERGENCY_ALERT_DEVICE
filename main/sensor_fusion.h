/**
 * This header defines the API for sensor fusion operations,
 * including Kalman filtering for motion data.
 */

#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include "esp_err.h"
#include "mpu.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize the sensor fusion module
 *
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t sensor_fusion_init(void);

/**
 * Update sensor fusion with new measurements
 *
 * @param accel_data Pointer to accelerometer data (can be NULL)
 * @param gyro_data Pointer to gyroscope data (can be NULL)
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t sensor_fusion_update(mpu_accel_data_t *accel_data, mpu_gyro_data_t *gyro_data);

/**
 * Get the latest filtered orientation data
 *
 * @param angle_x Pointer to store X angle (pitch) in degrees (can be NULL)
 * @param angle_y Pointer to store Y angle (roll) in degrees (can be NULL)
 * @param angle_z Pointer to store Z angle (yaw) in degrees (can be NULL)
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t sensor_fusion_get_orientation(float *angle_x, float *angle_y, float *angle_z);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_FUSION_H */
