#include "mpu.h"

// Constructor implementation
MPUSensor::MPUSensor() {
    // Initialize data structures to zero
    last_accel_data = {0};
    last_gyro_data = {0};
}

// I2C communication helper - write register
void MPUSensor::writeRegister(uint8_t reg_addr, uint8_t data) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg_addr);
    Wire.write(data);
    Wire.endTransmission();
}

// I2C communication helper - read registers
bool MPUSensor::readRegisters(uint8_t reg_addr, uint8_t* data_buf, size_t len) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg_addr);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    Wire.requestFrom(MPU6050_ADDR, len);
    for (size_t i = 0; i < len; i++) {
        if (Wire.available()) {
            data_buf[i] = Wire.read();
        } else {
            return false;
        }
    }
    return true;
}

// Configure motion detection
bool MPUSensor::configureMotionDetection() {
    // Configure motion detection threshold
    writeRegister(MPU_REG_MOT_THRESH, 10);

    // Enable motion detection
    writeRegister(MPU_REG_MOT_DETECT, 0x40);

    // Configure interrupt behavior
    writeRegister(MPU_REG_INT_PIN_CFG, 0x10);

    // Enable motion interrupt
    writeRegister(MPU_REG_INT_ENABLE, 0x40);

    return true;
}

// Initialize the MPU sensor
bool MPUSensor::begin() {
    // Initialize I2C
    Wire.begin();

    // Wake up the MPU6050 (it starts in sleep mode)
    writeRegister(MPU_REG_POWER, 0x00);

    // Configure gyroscope range (±250°/s)
    writeRegister(MPU_REG_GYRO_CONFIG, 0x00);

    // Configure accelerometer range (±2g)
    writeRegister(MPU_REG_ACCEL_CONFIG, 0x00);

    // Configure motion detection
    configureMotionDetection();

    return true;
}

// Read accelerometer data
bool MPUSensor::readAccel(mpu_accel_data_t *accel_data) {
    uint8_t data[6];
    if (!readRegisters(MPU_REG_ACCEL_XOUT_H, data, 6)) {
        return false;
    }

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

    return true;
}

// Read gyroscope data
bool MPUSensor::readGyro(mpu_gyro_data_t *gyro_data) {
    uint8_t data[6];
    if (!readRegisters(MPU_REG_GYRO_XOUT_H, data, 6)) {
        return false;
    }

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

    return true;
}

// Get the last known accelerometer data
bool MPUSensor::getLastAccel(mpu_accel_data_t *accel_data) {
    if (accel_data == NULL) {
        return false;
    }

    *accel_data = last_accel_data;
    return true;
}

// Get the last known gyroscope data
bool MPUSensor::getLastGyro(mpu_gyro_data_t *gyro_data) {
    if (gyro_data == NULL) {
        return false;
    }

    *gyro_data = last_gyro_data;
    return true;
}
