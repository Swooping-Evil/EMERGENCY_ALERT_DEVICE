#ifndef GPS_MODULE_H
#define GPS_MODULE_H

#include <Arduino.h>
#include <SoftwareSerial.h>

// GPS Data Structure
struct GPSData {
    char timestamp[10];    // Timestamp of GPS fix
    float latitude;        // Latitude in decimal degrees
    float longitude;       // Longitude in decimal degrees
    bool isValid;          // Whether the GPS data is valid
};

class GPSModule {
private:
    SoftwareSerial* gpsSerial;
    GPSData currentGPSData;
    char googleMapsUrl[100];
    bool gpsInitialized;

    // Parse NMEA sentence and extract GPS data
    bool parseGPSData(const char* nmea);

public:
    // Constructor
    GPSModule(int rxPin, int txPin, int baudRate = 9600);

    // Initialize GPS module
    bool begin();

    // Update GPS data by reading from serial
    bool update();

    // Get the latest GPS data
    bool getData(GPSData* data);

    // Check if GPS has a fix
    bool isFixed();

    // Put GPS into low power mode
    void sleep();

    // Wake up GPS from low power mode
    void wake();

    // Get Google Maps URL for current location
    const char* getGoogleMapsUrl();
};

#endif // GPS_MODULE_H
