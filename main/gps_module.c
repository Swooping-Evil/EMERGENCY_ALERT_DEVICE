#include "gps_module.h"

GPSModule::GPSModule(int rxPin, int txPin, int baudRate)
    : gpsSerial(new SoftwareSerial(rxPin, txPin)),
      currentGPSData{},
      googleMapsUrl{},
      gpsInitialized(false) {
    gpsSerial->begin(baudRate);
}

bool GPSModule::begin() {
    if (gpsInitialized) return true;

    // Initial setup if needed
    gpsInitialized = true;
    return true;
}

bool GPSModule::parseGPSData(const char* nmea) {
    if (strncmp(nmea, "$GPRMC", 6) != 0 && strncmp(nmea, "$GNRMC", 6) != 0) {
        return false;
    }

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

bool GPSModule::update() {
    if (!gpsInitialized) return false;

    char nmeaSentence[100];
    int nmeaIndex = 0;
    bool foundValidData = false;
    int retry = 0;
    const int maxRetry = 10;

    // Try to get a valid GPS reading, with maximum retries
    while (!foundValidData && retry < maxRetry) {
        while (gpsSerial->available()) {
            char c = gpsSerial->read();

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
            } else if (nmeaIndex < 99) {
                nmeaSentence[nmeaIndex++] = c;
            }
        }

        if (!foundValidData) {
            delay(200);
            retry++;
        }
    }

    return foundValidData;
}

bool GPSModule::getData(GPSData* data) {
    if (!currentGPSData.isValid) {
        return false;
    }

    memcpy(data, &currentGPSData, sizeof(GPSData));
    return true;
}

bool GPSModule::isFixed() {
    return currentGPSData.isValid;
}

void GPSModule::sleep() {
    // Sleep mode command for ublox NEO-6M
    const uint8_t ubxPowerSaveMode[] = {
        0xB5, 0x62,             // UBX header
        0x06, 0x11,             // CFG-RXM message
        0x02, 0x00,             // Payload length (2 bytes)
        0x08, 0x01,             // reserved and power mode (0x01 = power save)
        0x22, 0x92              // Checksum
    };

    for (int i = 0; i < sizeof(ubxPowerSaveMode); i++) {
        gpsSerial->write(ubxPowerSaveMode[i]);
    }
}

void GPSModule::wake() {
    // Wake up command for ublox NEO-6M
    const uint8_t ubxContinuousMode[] = {
        0xB5, 0x62,             // UBX header
        0x06, 0x11,             // CFG-RXM message
        0x02, 0x00,             // Payload length (2 bytes)
        0x08, 0x00,             // reserved and power mode (0x00 = continuous)
        0x21, 0x91              // Checksum
    };

    for (int i = 0; i < sizeof(ubxContinuousMode); i++) {
        gpsSerial->write(ubxContinuousMode[i]);
    }
}

const char* GPSModule::getGoogleMapsUrl() {
    if (!currentGPSData.isValid) {
        return NULL;
    }
    return googleMapsUrl;
}
