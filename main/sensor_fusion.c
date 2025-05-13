#include <Arduino.h>
#include <math.h>
#include "sensor_fusion.h"

class SensorFusion {
private:
    // Kalman filter state for each axis
    struct KalmanState {
        float angle;          // Estimated angle
        float bias;           // Estimated gyro bias
        float rate;           // Unbiased rate
        float P[2][2];        // Error covariance matrix
    };

    // Kalman filter states for each axis
    KalmanState kalmanX = {0};
    KalmanState kalmanY = {0};
    KalmanState kalmanZ = {0};

    // Time tracking for integration
    unsigned long lastUpdateTime = 0;

    // Initialize Kalman filter state
    void initKalmanState(KalmanState &state) {
        state.angle = 0.0f;
        state.bias = 0.0f;
        state.P[0][0] = 0.0f;
        state.P[0][1] = 0.0f;
        state.P[1][0] = 0.0f;
        state.P[1][1] = 0.0f;
    }

    // Update Kalman filter with new measurements
    float updateKalman(KalmanState &state, float angleMeasurement, float rateMeasurement, float dt) {
        // Step 1: Project the state ahead
        state.rate = rateMeasurement - state.bias;
        state.angle += dt * state.rate;

        // Step 2: Project the error covariance ahead
        state.P[0][0] += dt * (dt * state.P[1][1] - state.P[0][1] - state.P[1][0] + KALMAN_Q_ANGLE);
        state.P[0][1] -= dt * state.P[1][1];
        state.P[1][0] -= dt * state.P[1][1];
        state.P[1][1] += KALMAN_Q_BIAS * dt;

        // Step 3: Compute the Kalman gain
        float S = state.P[0][0] + KALMAN_R_MEASURE;
        float K[2];
        K[0] = state.P[0][0] / S;
        K[1] = state.P[1][0] / S;

        // Step 4: Update estimate with measurement
        float y = angleMeasurement - state.angle;
        state.angle += K[0] * y;
        state.bias += K[1] * y;

        // Step 5: Update the error covariance
        float P00_temp = state.P[0][0];
        float P01_temp = state.P[0][1];
        state.P[0][0] -= K[0] * P00_temp;
        state.P[0][1] -= K[0] * P01_temp;
        state.P[1][0] -= K[1] * P00_temp;
        state.P[1][1] -= K[1] * P01_temp;

        return state.angle;
    }

    // Calculate angle from accelerometer data
    void calculateAccelerometerAngles(float ax, float ay, float az, float &angleX, float &angleY) {
        // Calculate pitch (x-axis rotation)
        angleX = atan2f(ay, sqrtf(ax * ax + az * az)) * 180.0f / PI;

        // Calculate roll (y-axis rotation)
        angleY = atan2f(-ax, az) * 180.0f / PI;
    }

public:
    // Initialize the sensor fusion module
    void init() {
        // Initialize Kalman filter states
        initKalmanState(kalmanX);
        initKalmanState(kalmanY);
        initKalmanState(kalmanZ);

        // Initialize time tracking
        lastUpdateTime = micros();
    }

    // Update sensor fusion with new measurements
    bool update(float ax, float ay, float az, float gx, float gy, float gz) {
        // Calculate time delta
        unsigned long currentTime = micros();
        float dt = (float)(currentTime - lastUpdateTime) / 1000000.0f; // Convert to seconds
        lastUpdateTime = currentTime;

        // Sanity check on dt
        if (dt <= 0.0f || dt > 1.0f) {
            dt = 0.01f; // Default to 10ms if time delta seems unreasonable
        }

        // Calculate angles from accelerometer
        float accelAngleX, accelAngleY;
        calculateAccelerometerAngles(ax, ay, az, accelAngleX, accelAngleY);

        // Update Kalman filters with accelerometer angles and gyroscope rates
        float filteredAngleX = updateKalman(kalmanX, accelAngleX, gx, dt);
        float filteredAngleY = updateKalman(kalmanY, accelAngleY, gy, dt);

        return true;
    }

    // Get the latest filtered orientation data
    void getOrientation(float &angleX, float &angleY, float &angleZ) {
        angleX = kalmanX.angle;
        angleY = kalmanY.angle;
        angleZ = kalmanZ.angle;
    }
};

