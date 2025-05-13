#ifndef SOS_MODULE_H
#define SOS_MODULE_H

#include <Arduino.h>
#include "gps_module.h"
#include "gsm_module.h"

// Alert types
typedef enum {
  SOS_ALERT_TYPE_NONE,       // No alert
  SOS_ALERT_TYPE_FALL,       // Fall detected
  SOS_ALERT_TYPE_PANIC_BUTTON // Manual panic button
} SOSAlertType;

// SOS event data structure
typedef struct {
  SOSAlertType type;     // Type of alert
  unsigned long timestamp; // Time when alert was triggered (in seconds)
  bool gpsValid;         // Whether GPS data is valid
  float latitude;        // Latitude from GPS, if valid
  float longitude;       // Longitude from GPS, if valid
} SOSEvent;

class SOSModule {
  private:
    // Constants
    static const int PANIC_BUTTON_DEBOUNCE_MS = 50;
    static const int DOUBLE_PRESS_TIMEOUT_MS = 500;
    static const unsigned long ALERT_TIMEOUT_SECONDS = 300; // 5 minutes

    // Pin assignments
    int panicButtonPin;
    int buzzerPin;
    int ledPin;

    // Button state
    volatile bool buttonPressed;
    volatile unsigned long lastButtonPressTime;
    volatile int buttonPressCount;

    // Alert state
    bool alertActive;
    SOSEvent lastEvent;
    bool alarmActive;
    bool ledFlashing;
    unsigned long alertStartTime;

    // Timers and intervals
    unsigned long lastLedToggleTime;
    unsigned long lastAlarmUpdateTime;
    int alarmSequenceState;

    // References to other modules
    GPSModule* gpsModule;
    GSMModule* gsmModule;

    // Helper methods
    void handlePanicButtonPress();
    void updateLedFlashing();
    void updateAlarmSound();
    void alarmSoundPattern();

    // Button interrupt handler - static wrapper and instance method
    static void buttonISRWrapper();
    void buttonISR();

  public:
    SOSModule();

    // Initialization
    void begin(int buttonPin, int buzzer, int led, GPSModule* gps, GSMModule* gsm);

    // Main loop update - call in Arduino loop()
    void update();

    // Alert management
    void triggerAlert(SOSAlertType alertType);
    void cancelAlert();
    bool isAlertActive() const;
    SOSEvent getLastEvent() const;

    // Sound and visual feedback
    bool playAlarm(bool enable);
    bool flashLed(bool enable);
};

#endif // SOS_MODULE_H
