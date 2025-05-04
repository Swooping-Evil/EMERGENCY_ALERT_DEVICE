/**
 * This module handles the GPS/GNSS communication and location data extraction.
 */
#include "gps_module.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "driver/uart.h"
#include "esp_log.h"

#define GPS_UART_NUM UART_NUM_2
#define GPS_UART_TX_PIN 17
#define GPS_UART_RX_PIN 16
#define GPS_UART_BAUD_RATE 9600
#define BUF_SIZE 1024

static const char* TAG = "GPS_MODULE";
static GPSData currentGPSData = {0};
static char googleMapsUrl[100] = {0};
static bool gpsInitialized = false;

// Initialize GPS module
void gps_init() {
    if (gpsInitialized) return;

    // Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = GPS_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    // Install UART driver
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(GPS_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_NUM, GPS_UART_TX_PIN, GPS_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    gpsInitialized = true;

    ESP_LOGI(TAG, "GPS module initialized");
}

// Parse NMEA sentence and extract GPS data
static bool parseGPSData(const char* nmea) {
    if (strncmp(nmea, "$GPRMC", 6) != 0 && strncmp(nmea, "$GNRMC", 6) != 0) {
        return false;
    }

    // Sample format: $GPRMC,053508.00,A,1725.64574,N,07835.11697,E,0.041,,121217,,,D*79
    char status;
    char latDir, longDir;
    char latDeg[3], latMin[10], longDeg[4], longMin[10];
    char tempStr[100];
    float latMinutes, longMinutes;

    // Tokenize the string
    strcpy(tempStr, nmea);
    char* token = strtok(tempStr, ",");

    // Skip the message identifier
    token = strtok(NULL, ","); // Timestamp
    if (token != NULL) {
        strncpy(currentGPSData.timestamp, token, 9);
        currentGPSData.timestamp[9] = '\0';
    }

    token = strtok(NULL, ","); // Status
    if (token == NULL || token[0] != 'A') {
        currentGPSData.isValid = false;
        return false;
    }

    token = strtok(NULL, ","); // Latitude
    if (token == NULL) return false;

    // Extract degrees and minutes for latitude
    strncpy(latDeg, token, 2);
    latDeg[2] = '\0';
    strcpy(latMin, token + 2);

    token = strtok(NULL, ","); // Latitude direction
    if (token == NULL) return false;
    latDir = token[0];

    token = strtok(NULL, ","); // Longitude
    if (token == NULL) return false;

    // Extract degrees and minutes for longitude
    strncpy(longDeg, token, 3);
    longDeg[3] = '\0';
    strcpy(longMin, token + 3);

    token = strtok(NULL, ","); // Longitude direction
    if (token == NULL) return false;
    longDir = token[0];

    // Convert to decimal degrees
    latMinutes = atof(latMin) / 60.0;
    currentGPSData.latitude = atof(latDeg) + latMinutes;
    if (latDir == 'S') currentGPSData.latitude *= -1;

    longMinutes = atof(longMin) / 60.0;
    currentGPSData.longitude = atof(longDeg) + longMinutes;
    if (longDir == 'W') currentGPSData.longitude *= -1;

    currentGPSData.isValid = true;

    // Update Google Maps URL
    snprintf(googleMapsUrl, sizeof(googleMapsUrl),
             "https://www.google.com/maps/place/%.6f,%.6f",
             currentGPSData.latitude, currentGPSData.longitude);

    return true;
}

// Update GPS data by reading from UART
bool gps_update() {
    if (!gpsInitialized) {
        ESP_LOGE(TAG, "GPS module not initialized");
        return false;
    }

    uint8_t data[BUF_SIZE];
    char nmeaSentence[BUF_SIZE];
    int nmeaIndex = 0;
    bool foundValidData = false;
    int retry = 0;
    const int maxRetry = 10;

    // Try to get a valid GPS reading, with maximum retries
    while (!foundValidData && retry < maxRetry) {
        int length = uart_read_bytes(GPS_UART_NUM, data, BUF_SIZE - 1, 200 / portTICK_PERIOD_MS);

        if (length > 0) {
            data[length] = 0; // Null-terminate the data

            // Process each byte
            for (int i = 0; i < length; i++) {
                char c = data[i];

                if (c == '$') {
                    // Start of a new NMEA sentence
                    nmeaIndex = 0;
                    nmeaSentence[nmeaIndex++] = c;
                } else if (c == '\r' || c == '\n') {
                    if (nmeaIndex > 0) {
                        // End of NMEA sentence
                        nmeaSentence[nmeaIndex] = 0;
                        if (parseGPSData(nmeaSentence)) {
                            foundValidData = true;
                            break;
                        }
                    }
                } else if (nmeaIndex < BUF_SIZE - 1) {
                    nmeaSentence[nmeaIndex++] = c;
                }
            }
        }

        retry++;
    }

    if (foundValidData) {
        ESP_LOGI(TAG, "GPS data updated: Lat=%.6f, Long=%.6f",
                currentGPSData.latitude, currentGPSData.longitude);
    } else {
        ESP_LOGW(TAG, "Failed to update GPS data after %d attempts", maxRetry);
    }

    return foundValidData;
}

// Get the latest GPS data
bool gps_getData(GPSData* data) {
    if (!currentGPSData.isValid) {
        return false;
    }

    memcpy(data, &currentGPSData, sizeof(GPSData));
    return true;
}

// Check if GPS has a fix
bool gps_isFixed() {
    return currentGPSData.isValid;
}

// Put GPS into low power mode
void gps_sleep() {
    // Send low power mode command to GPS module
    // This depends on the specific GPS module being used
    const char* sleepCmd = "$PMTK161,0*28\r\n"; // Example for MTK GPS modules
    uart_write_bytes(GPS_UART_NUM, sleepCmd, strlen(sleepCmd));
    ESP_LOGI(TAG, "GPS module entering sleep mode");
}

// Wake up GPS from low power mode
void gps_wake() {
    // Send any character to wake up the GPS
    const char* wakeCmd = "\r\n";
    uart_write_bytes(GPS_UART_NUM, wakeCmd, strlen(wakeCmd));
    ESP_LOGI(TAG, "GPS module waking up");
}

// Get Google Maps URL for current location
const char* gps_getGoogleMapsUrl() {
    if (!currentGPSData.isValid) {
        return NULL;
    }
    return googleMapsUrl;
}
