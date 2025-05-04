
#ifndef GPS_MODULE_H
#define GPS_MODULE_H

#include <stdbool.h>

// GPS data structure
typedef struct {
    float latitude;
    float longitude;
    bool isValid;
    char timestamp[10];
} GPSData;

// Function prototypes
void gps_init();
bool gps_update();
bool gps_getData(GPSData* data);
bool gps_isFixed();
void gps_sleep();
void gps_wake();
const char* gps_getGoogleMapsUrl();

#endif // GPS_MODULE_H
