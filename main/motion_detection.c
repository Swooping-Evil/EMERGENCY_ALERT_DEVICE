#include "motion_detection.h"

MotionDetection::MotionDetection()
    : motionHistory{},
      fallCallback(nullptr),
      motionDetectionRunning(false),
      lastSampleTime(0) {
}

bool MotionDetection::begin() {
    // Initialize Wire library
    Wire.begin();

    // Initialize MPU6050
    mpu.initialize();

    // Test connection
    if (!mpu.testConnection()) {
        return false;
    }

    // Initialize motion history
    unsigned long currentTime = millis();
    for (int i = 0; i < 10; i++) {
        motionHistory.accel_magnitude[i] = 1.0f; // Initialize with 1g (earth gravity)
        motionHistory.timestamps[i] = currentTime;
    }

    return true;
}

float MotionDetection::calculateAccelMagnitude(int16_t ax, int16_t ay, int16_t az) {
    // Convert raw values to g
    float ax_g = ax / 16384.0f;
    float ay_g = ay / 16384.0f;
    float az_g = az / 16384.0f;

    return sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
}

float MotionDetection::calculateTiltAngle(int16_t ax, int16_t ay, int16_t az) {
    // Calculate the acceleration magnitude
    float accelMag = calculateAccelMagnitude(ax, ay, az);

    // Convert raw z-axis to g
    float az_g = az / 16384.0f;

    // Calculate tilt angle
    if (accelMag > 0) {
        return acos(az_g / accelMag) * 180.0f / PI;
    }
    return 0.0f;
}

void MotionDetection::updateMotionHistory(float accelMagnitude) {
    // Update circular buffer
    motionHistory.accel_magnitude[motionHistory.index] = accelMagnitude;
    motionHistory.timestamps[motionHistory.index] = millis();
    motionHistory.index = (motionHistory.index + 1) % 10;
}

bool MotionDetection::detectImpact(float accelMagnitude) {
    // Check if acceleration exceeds the threshold
    return accelMagnitude > FALL_ACCEL_THRESHOLD;
}

bool MotionDetection::detectRest(float accelMagnitude, float tiltAngle) {
    // Check if person is in a horizontal position (high tilt angle)
    // and not moving significantly (stable acceleration close to 1g)
    return (tiltAngle > FALL_ANGLE_THRESHOLD) &&
           (abs(accelMagnitude - 1.0f) < 0.2f);
}

bool MotionDetection::detectFall() {
    static FallState fallState = FALL_STATE_MONITORING;
    static unsigned long impactStartTime = 0;
    static unsigned long restStartTime = 0;
    unsigned long currentTime = millis();

    // Read raw accelerometer data
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);

    // Calculate magnitude and tilt
    float accelMagnitude = calculateAccelMagnitude(ax, ay, az);
    float tiltAngle = calculateTiltAngle(ax, ay, az);

    // Update motion history
    updateMotionHistory(accelMagnitude);

    // Fall detection state machine
    switch (fallState) {
        case FALL_STATE_MONITORING:
            if (detectImpact(accelMagnitude)) {
                impactStartTime = currentTime;
                fallState = FALL_STATE_IMPACT_DETECTED;
            }
            break;

        case FALL_STATE_IMPACT_DETECTED:
            // Verify impact duration
            if (currentTime - impactStartTime > IMPACT_DURATION_MS) {
                if (detectRest(accelMagnitude, tiltAngle)) {
                    restStartTime = currentTime;
                    fallState = FALL_STATE_REST_DETECTED;
                } else if (accelMagnitude < FALL_ACCEL_THRESHOLD * 0.8f) {
                    // If acceleration returns to normal without rest, go back to monitoring
                    fallState = FALL_STATE_MONITORING;
                }
            }
            break;

        case FALL_STATE_REST_DETECTED:
            // Verify rest duration
            if (currentTime - restStartTime > REST_DURATION_MS) {
                if (detectRest(accelMagnitude, tiltAngle)) {
                    fallState = FALL_STATE_FALL_CONFIRMED;
                    return true;
                } else {
                    // If person starts moving again, go back to monitoring
                    fallState = FALL_STATE_MONITORING;
                }
            }
            break;

        case FALL_STATE_FALL_CONFIRMED:
            // Reset state after confirming fall
            fallState = FALL_STATE_MONITORING;
            break;
    }

    return false;
}

bool MotionDetection::start(motion_callback_t callback) {
    if (motionDetectionRunning) {
        return false;
    }

    fallCallback = callback;
    motionDetectionRunning = true;
    return true;
}

void MotionDetection::stop() {
    motionDetectionRunning = false;
    fallCallback = nullptr;
}

void MotionDetection::update() {
    if (!motionDetectionRunning) return;

    // Check sampling rate (50 Hz)
    unsigned long currentTime = millis();
    if (currentTime - lastSampleTime < 20) return;

    lastSampleTime = currentTime;

    // Run fall detection algorithm
    if (detectFall() && fallCallback != nullptr) {
        // Trigger callback if fall is detected
        fallCallback();
    }
}

bool MotionDetection::isFallDetected() {
    return detectFall();
}
