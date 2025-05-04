// gsm_module.h
#ifndef GSM_MODULE_H
#define GSM_MODULE_H

#include <stdbool.h>
#include "gps_module.h"

#define MAX_EMERGENCY_CONTACTS 5

typedef enum {
    GSM_STATUS_OFF,
    GSM_STATUS_INITIALIZING,
    GSM_STATUS_READY,
    GSM_STATUS_ERROR
} GSMStatus;

// Function prototypes
void gsm_init();
GSMStatus gsm_getStatus();
bool gsm_sendSMS(const char* phoneNumber, const char* message);
bool gsm_sendEmergencyAlert(const GPSData* gpsData);
void gsm_sleep();
void gsm_wake();
bool gsm_addEmergencyContact(const char* phoneNumber);
bool gsm_removeEmergencyContact(const char* phoneNumber);
void gsm_clearEmergencyContacts();

#endif // GSM_MODULE_H
