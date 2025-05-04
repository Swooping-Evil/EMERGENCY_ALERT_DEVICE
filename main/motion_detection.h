/**
 * This header defines the API for fall detection and motion analysis.
 */

#ifndef MOTION_DETECTION_H
#define MOTION_DETECTION_H

#include "esp_err.h"

/**
 * Callback function type for fall detection events
 */
typedef void (*motion_callback_t)(void);

/**
 * Initialize the motion detection module
 *
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t motion_detection_init(void);

/**
 * Start the motion detection processing
 *
 * @param callback Function to call when a fall is detected
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t motion_detection_start(motion_callback_t callback);

/**
 * Stop the motion detection processing
 *
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t motion_detection_stop(void);

/**
 * Check if a fall has been detected (polling approach)
 *
 * @return true if fall detected, false otherwise
 */
bool motion_detection_is_fall_detected(void);

#endif /* MOTION_DETECTION_H */
