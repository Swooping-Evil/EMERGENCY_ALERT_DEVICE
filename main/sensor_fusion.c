/**
 * This module implements a Kalman filter to improve the accuracy of motion data from multiple sensors.
 */

#include <stdio.h>
#include <math.h>
#include "esp_log.h"

#include "sensor_fusion.h"
#include "mpu.h"

#define TAG "SENSOR_FUSION"

// Kalman filter parameters
#define KALMAN_Q_ANGLE   0.001f  // Process noise covariance for angle
#define KALMAN_Q_BIAS    0.003f  // Process noise covariance for bias
#define KALMAN_R_MEASURE 0.03f   // Measurement noise covariance

// Kalman filter state for each axis
typedef struct {
    float angle;          // Estimated angle
    float bias;           // Estimated gyro bias
    float rate;           // Unbiased rate
    float P[2][2];        // Error covariance matrix
} kalman_state_t;

// Global Kalman filter states for each axis
static kalman_state_t kalman_x = {0};
static kalman_state_t kalman_y = {0};
static kalman_state_t kalman_z = {0};

// Time tracking for integration
static int64_t last_update_time = 0;

/**
 * Initialize Kalman filter state
 */
static void init_kalman_state(kalman_state_t *state)
{
    state->angle = 0.0f;
    state->bias = 0.0f;
    state->P[0][0] = 0.0f;
    state->P[0][1] = 0.0f;
    state->P[1][0] = 0.0f;
    state->P[1][1] = 0.0f;
}

/**
 * Update Kalman filter with new measurements
 */
static float update_kalman(kalman_state_t *state, float angle_measurement, float rate_measurement, float dt)
{
    // Step 1: Project the state ahead
    state->rate = rate_measurement - state->bias;
    state->angle += dt * state->rate;

    // Step 2: Project the error covariance ahead
    state->P[0][0] += dt * (dt * state->P[1][1] - state->P[0][1] - state->P[1][0] + KALMAN_Q_ANGLE);
    state->P[0][1] -= dt * state->P[1][1];
    state->P[1][0] -= dt * state->P[1][1];
    state->P[1][1] += KALMAN_Q_BIAS * dt;

    // Step 3: Compute the Kalman gain
    float S = state->P[0][0] + KALMAN_R_MEASURE;
    float K[2];
    K[0] = state->P[0][0] / S;
    K[1] = state->P[1][0] / S;

    // Step 4: Update estimate with measurement
    float y = angle_measurement - state->angle;
    state->angle += K[0] * y;
    state->bias += K[1] * y;

    // Step 5: Update the error covariance
    float P00_temp = state->P[0][0];
    float P01_temp = state->P[0][1];
    state->P[0][0] -= K[0] * P00_temp;
    state->P[0][1] -= K[0] * P01_temp;
    state->P[1][0] -= K[1] * P00_temp;
    state->P[1][1] -= K[1] * P01_temp;

    return state->angle;
}

/**
 * Calculate angle from accelerometer data
 */
static void calculate_accelerometer_angles(mpu_accel_data_t *accel_data, float *angle_x, float *angle_y)
{
    // Calculate pitch (x-axis rotation)
    *angle_x = atan2f(accel_data->y_g, sqrtf(accel_data->x_g * accel_data->x_g + accel_data->z_g * accel_data->z_g)) * 180.0f / M_PI;

    // Calculate roll (y-axis rotation)
    *angle_y = atan2f(-accel_data->x_g, accel_data->z_g) * 180.0f / M_PI;
}

/**
 * Initialize the sensor fusion module
 */
esp_err_t sensor_fusion_init(void)
{
    ESP_LOGI(TAG, "Initializing sensor fusion module");

    // Initialize Kalman filter states
    init_kalman_state(&kalman_x);
    init_kalman_state(&kalman_y);
    init_kalman_state(&kalman_z);

    // Initialize time tracking
    last_update_time = esp_timer_get_time();

    return ESP_OK;
}

/**
 * Update sensor fusion with new measurements
 */
esp_err_t sensor_fusion_update(mpu_accel_data_t *accel_data, mpu_gyro_data_t *gyro_data)
{
    // Calculate time delta
    int64_t current_time = esp_timer_get_time();
    float dt = (float)(current_time - last_update_time) / 1000000.0f; // Convert to seconds
    last_update_time = current_time;

    // Sanity check on dt
    if (dt <= 0.0f || dt > 1.0f) {
        dt = 0.01f; // Default to 10ms if time delta seems unreasonable
    }

    // If we have both accelerometer and gyroscope data, perform sensor fusion
    if (accel_data != NULL && gyro_data != NULL) {
        // Calculate angles from accelerometer
        float accel_angle_x, accel_angle_y;
        calculate_accelerometer_angles(accel_data, &accel_angle_x, &accel_angle_y);

        // Update Kalman filters with accelerometer angles and gyroscope rates
        float filtered_angle_x = update_kalman(&kalman_x, accel_angle_x, gyro_data->x_dps, dt);
        float filtered_angle_y = update_kalman(&kalman_y, accel_angle_y, gyro_data->y_dps, dt);

        // Update the original data with filtered values (optional)
        // This step depends on how you want to use the filtered data

        ESP_LOGD(TAG, "Filtered angles: X=%.2f°, Y=%.2f°", filtered_angle_x, filtered_angle_y);
    }
    // If we only have accelerometer data, use it directly
    else if (accel_data != NULL) {
        // Calculate angles from accelerometer
        float accel_angle_x, accel_angle_y;
        calculate_accelerometer_angles(accel_data, &accel_angle_x, &accel_angle_y);

        // Update Kalman filters with accelerometer angles only
        // Assume no rotation rate (use 0)
        float filtered_angle_x = update_kalman(&kalman_x, accel_angle_x, 0.0f, dt);
        float filtered_angle_y = update_kalman(&kalman_y, accel_angle_y, 0.0f, dt);

        ESP_LOGD(TAG, "Filtered angles (accel only): X=%.2f°, Y=%.2f°", filtered_angle_x, filtered_angle_y);
    }
    // If we only have gyroscope data, integrate it
    else if (gyro_data != NULL) {
        // Update Kalman filters with gyroscope rates only
        // No measurement update, just prediction
        kalman_x.rate = gyro_data->x_dps - kalman_x.bias;
        kalman_x.angle += dt * kalman_x.rate;

        kalman_y.rate = gyro_data->y_dps - kalman_y.bias;
        kalman_y.angle += dt * kalman_y.rate;

        ESP_LOGD(TAG, "Integrated angles (gyro only): X=%.2f°, Y=%.2f°", kalman_x.angle, kalman_y.angle);
    }
    else {
        // No data provided
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

/**
 * Get the latest filtered orientation data
 */
esp_err_t sensor_fusion_get_orientation(float *angle_x, float *angle_y, float *angle_z)
{
    if (angle_x != NULL) {
        *angle_x = kalman_x.angle;
    }

    if (angle_y != NULL) {
        *angle_y = kalman_y.angle;
    }

    if (angle_z != NULL) {
        *angle_z = kalman_z.angle;
    }

    return ESP_OK;
}
