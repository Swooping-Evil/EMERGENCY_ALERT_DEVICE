// sos_module.h
#ifndef SOS_MODULE_H
#define SOS_MODULE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    SOS_ALERT_TYPE_NONE,
    SOS_ALERT_TYPE_PANIC_BUTTON,
    SOS_ALERT_TYPE_FALL_DETECTION,
    SOS_ALERT_TYPE_TIMED_ALERT
} SOSAlertType;

typedef struct {
    SOSAlertType type;
    uint32_t timestamp;
    bool gpsValid;
    float latitude;
    float longitude;
} SOSEvent;

// Function prototypes
void sos_init();
void sos_registerPanicButtonPin(int buttonPin);
bool sos_checkPanicButton();
bool sos_isAlertActive();
void sos_triggerAlert(SOSAlertType alertType);
void sos_cancelAlert();
SOSEvent sos_getLastEvent();
bool sos_playAlarm(bool enable);
bool sos_flashLed(bool enable);

#endif // SOS_MODULE_H
