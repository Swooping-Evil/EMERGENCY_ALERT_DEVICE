#ifndef MOTION_DETECTION_H
#define MOTION_DETECTION_H

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>

// Callback function type
typedef void (*motion_callback_t)();

class MotionDetection {
public:
    // Fall detection threshold parameters
    static constexpr float FALL_ACCEL_THRESHOLD      = 2.5f;    // Acceleration threshold in g
    static constexpr float FALL_ANGLE_THRESHOLD      = 60.0f;   // Angle threshold in degrees
    static constexpr unsigned long IMPACT_DURATION_MS  = 50;     // Minimum impact duration in milliseconds
    static constexpr unsigned long REST_DURATION_MS    = 1000;   // Post-impact rest detection duration

private:
    // Fall detection state machine states
    enum FallState {
        FALL_STATE_MONITORING,
        FALL_STATE_IMPACT_DETECTED,
        FALL_STATE_REST_DETECTED,
        FALL_STATE_FALL_CONFIRMED
    };

    // Motion history for pattern recognition
    struct MotionHistory {
        float accel_magnitude[10];    // Store last 10 acceleration magnitudes
        unsigned long timestamps[10]; // Corresponding timestamps
        int index;                    // Current index in circular buffer
    };

    MPU6050 mpu;
    MotionHistory motionHistory;
    motion_callback_t fallCallback;
    bool motionDetectionRunning;
    unsigned long lastSampleTime;

    // Calculate the acceleration magnitude from 3-axis accelerometer data
    float calculateAccelMagnitude(int16_t ax, int16_t ay, int16_t az);

    // Calculate the tilt angle from the accelerometer data
    float calculateTiltAngle(int16_t ax, int16_t ay, int16_t az);

    // Add a new acceleration measurement to the motion history
    void updateMotionHistory(float accelMagnitude);

    // Detect impact by checking for sudden acceleration peak
    bool detectImpact(float accelMagnitude);

    // Detect rest state after impact (person may be lying on the ground)
    bool detectRest(float accelMagnitude, float tiltAngle);

    // Fall detection algorithm
    bool detectFall();

public:
    // Constructor
    MotionDetection();

    // Initialize the motion detection module
    bool begin();

    // Start motion detection with optional callback
    bool start(motion_callback_t callback = nullptr);

    // Stop motion detection
    void stop();

    // Check if a fall has been detected
    bool isFallDetected();

    // Update method to be called in loop()
    void update();
};

#endif // MOTION_DETECTION_H
