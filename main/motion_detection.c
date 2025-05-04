/**
 * This module handles the processing of accelerometer and gyroscope
 * data to detect falls and other critical motion events.
 */

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "motion_detection.h"
#include "mpu.h"
#include "sensor_fusion.h"

#define TAG "MOTION_DETECTION"

// Fall detection threshold parameters
#define FALL_ACCEL_THRESHOLD      2.5f    // Acceleration threshold in g
#define FALL_ANGLE_THRESHOLD      60.0f   // Angle threshold in degrees
#define IMPACT_DURATION_MS        50      // Minimum impact duration in milliseconds
#define REST_DURATION_MS          1000    // Post-impact rest detection duration

// Motion detection task parameters
#define MOTION_DETECTION_TASK_STACK_SIZE  4096
#define MOTION_DETECTION_TASK_PRIORITY    5
#define MOTION_SAMPLING_PERIOD_MS         20  // 50 Hz sampling

// Global variables
static TaskHandle_t motion_detection_task_handle = NULL;
static motion_callback_t fall_callback = NULL;
static bool motion_detection_running = false;

// Fall detection state machine states
typedef enum {
    FALL_STATE_MONITORING,
    FALL_STATE_IMPACT_DETECTED,
    FALL_STATE_REST_DETECTED,
    FALL_STATE_FALL_CONFIRMED
} fall_state_t;

// Motion history for pattern recognition
typedef struct {
    float accel_magnitude[10];    // Store last 10 acceleration magnitudes
    int64_t timestamps[10];       // Corresponding timestamps
    int index;                    // Current index in circular buffer
} motion_history_t;

static motion_history_t motion_history = {
    .index = 0
};

/**
 * Calculate the acceleration magnitude from 3-axis accelerometer data
 */
static float calculate_accel_magnitude(mpu_accel_data_t *accel_data)
{
    return sqrtf(accel_data->x_g * accel_data->x_g +
                 accel_data->y_g * accel_data->y_g +
                 accel_data->z_g * accel_data->z_g);
}

/**
 * Calculate the tilt angle from the accelerometer data
 */
static float calculate_tilt_angle(mpu_accel_data_t *accel_data)
{
    // Calculate the angle between the z-axis and the acceleration vector
    float accel_mag = calculate_accel_magnitude(accel_data);
    if (accel_mag > 0) {
        return acosf(accel_data->z_g / accel_mag) * 180.0f / M_PI;
    }
    return 0.0f;
}

/**
 * Add a new acceleration measurement to the motion history
 */
static void update_motion_history(float accel_magnitude)
{
    // Update circular buffer
    motion_history.accel_magnitude[motion_history.index] = accel_magnitude;
    motion_history.timestamps[motion_history.index] = esp_timer_get_time() / 1000; // Convert to ms
    motion_history.index = (motion_history.index + 1) % 10;
}

/**
 * Detect impact by checking for sudden acceleration peak
 */
static bool detect_impact(float accel_magnitude)
{
    // Check if acceleration exceeds the threshold
    return accel_magnitude > FALL_ACCEL_THRESHOLD;
}

/**
 * Detect rest state after impact (person may be lying on the ground)
 */
static bool detect_rest(float accel_magnitude, float tilt_angle)
{
    // Check if person is in a horizontal position (high tilt angle)
    // and not moving significantly (stable acceleration close to 1g)
    return (tilt_angle > FALL_ANGLE_THRESHOLD) &&
           (fabsf(accel_magnitude - 1.0f) < 0.2f);
}

/**
 * Fall detection algorithm
 */
static bool detect_fall(void)
{
    static fall_state_t fall_state = FALL_STATE_MONITORING;
    static int64_t impact_start_time = 0;
    static int64_t rest_start_time = 0;
    int64_t current_time = esp_timer_get_time() / 1000; // Convert to ms

    // Get latest accelerometer data
    mpu_accel_data_t accel_data;
    if (mpu_read_accel(&accel_data) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read accelerometer data");
        return false;
    }

    // Apply sensor fusion for more reliable readings
    sensor_fusion_update(&accel_data, NULL);

    // Calculate magnitude and tilt
    float accel_magnitude = calculate_accel_magnitude(&accel_data);
    float tilt_angle = calculate_tilt_angle(&accel_data);

    // Update motion history
    update_motion_history(accel_magnitude);

    // Fall detection state machine
    switch (fall_state) {
        case FALL_STATE_MONITORING:
            if (detect_impact(accel_magnitude)) {
                ESP_LOGI(TAG, "Impact detected: %f g", accel_magnitude);
                impact_start_time = current_time;
                fall_state = FALL_STATE_IMPACT_DETECTED;
            }
            break;

        case FALL_STATE_IMPACT_DETECTED:
            // Verify impact duration
            if (current_time - impact_start_time > IMPACT_DURATION_MS) {
                if (detect_rest(accel_magnitude, tilt_angle)) {
                    ESP_LOGI(TAG, "Rest after impact detected. Tilt angle: %f degrees", tilt_angle);
                    rest_start_time = current_time;
                    fall_state = FALL_STATE_REST_DETECTED;
                } else if (accel_magnitude < FALL_ACCEL_THRESHOLD * 0.8f) {
                    // If acceleration returns to normal without rest, go back to monitoring
                    fall_state = FALL_STATE_MONITORING;
                }
            }
            break;

        case FALL_STATE_REST_DETECTED:
            // Verify rest duration
            if (current_time - rest_start_time > REST_DURATION_MS) {
                if (detect_rest(accel_magnitude, tilt_angle)) {
                    ESP_LOGI(TAG, "Fall confirmed! Person appears to be lying down.");
                    fall_state = FALL_STATE_FALL_CONFIRMED;
                    return true;
                } else {
                    // If person starts moving again, go back to monitoring
                    fall_state = FALL_STATE_MONITORING;
                }
            }
            break;

        case FALL_STATE_FALL_CONFIRMED:
            // Reset state after confirming fall
            fall_state = FALL_STATE_MONITORING;
            break;
    }

    return false;
}

/**
 * Motion detection task function
 */
static void motion_detection_task(void *pvParameters)
{
    // Initialize motion history
    int64_t current_time = esp_timer_get_time() / 1000;
    for (int i = 0; i < 10; i++) {
        motion_history.accel_magnitude[i] = 1.0f; // Initialize with 1g (earth gravity)
        motion_history.timestamps[i] = current_time;
    }

    ESP_LOGI(TAG, "Motion detection task started");

    while (motion_detection_running) {
        // Run fall detection algorithm
        if (detect_fall() && fall_callback != NULL) {
            // Trigger callback if fall is detected
            fall_callback();
        }

        // Sleep to achieve desired sampling rate
        vTaskDelay(MOTION_SAMPLING_PERIOD_MS / portTICK_PERIOD_MS);
    }

    // Task cleanup
    vTaskDelete(NULL);
}

/**
 * Initialize the motion detection module
 */
esp_err_t motion_detection_init(void)
{
    ESP_LOGI(TAG, "Initializing motion detection module");

    // Initialize motion history
    int64_t current_time = esp_timer_get_time() / 1000;
    for (int i = 0; i < 10; i++) {
        motion_history.accel_magnitude[i] = 1.0f; // Initialize with 1g (earth gravity)
        motion_history.timestamps[i] = current_time;
    }

    return ESP_OK;
}

/**
 * Start motion detection
 */
esp_err_t motion_detection_start(motion_callback_t callback)
{
    if (motion_detection_running) {
        ESP_LOGW(TAG, "Motion detection already running");
        return ESP_ERR_INVALID_STATE;
    }

    // Set callback function
    fall_callback = callback;

    // Set running flag
    motion_detection_running = true;

    // Create motion detection task
    BaseType_t result = xTaskCreate(
        motion_detection_task,
        "motion_detection_task",
        MOTION_DETECTION_TASK_STACK_SIZE,
        NULL,
        MOTION_DETECTION_TASK_PRIORITY,
        &motion_detection_task_handle
    );

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create motion detection task");
        motion_detection_running = false;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Motion detection started");
    return ESP_OK;
}

/**
 * Stop motion detection
 */
esp_err_t motion_detection_stop(void)
{
    if (!motion_detection_running) {
        ESP_LOGW(TAG, "Motion detection not running");
        return ESP_ERR_INVALID_STATE;
    }

    // Clear running flag
    motion_detection_running = false;

    // Wait for task to terminate
    vTaskDelay(100 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "Motion detection stopped");
    return ESP_OK;
}

/**
 * Check if a fall has been detected (for polling approach)
 */
bool motion_detection_is_fall_detected(void)
{
    return detect_fall();
}
