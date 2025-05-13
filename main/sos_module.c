#include "sos_module.h"

// Static pointer for ISR callback
static SOSModule* sosModuleInstance = nullptr;

// Static wrapper for button interrupt handling
void SOSModule::buttonISRWrapper() {
  if (sosModuleInstance) {
    sosModuleInstance->buttonISR();
  }
}

// Constructor
SOSModule::SOSModule()
  : panicButtonPin(-1),
    buzzerPin(-1),
    ledPin(-1),
    buttonPressed(false),
    lastButtonPressTime(0),
    buttonPressCount(0),
    alertActive(false),
    lastEvent({SOS_ALERT_TYPE_NONE, 0, false, 0.0, 0.0}),
    alarmActive(false),
    ledFlashing(false),
    alertStartTime(0),
    lastLedToggleTime(0),
    lastAlarmUpdateTime(0),
    alarmSequenceState(0),
    gpsModule(nullptr),
    gsmModule(nullptr) {
}

// Initialize the SOS module
void SOSModule::begin(int buttonPin, int buzzer, int led, GPSModule* gps, GSMModule* gsm) {
  // Save pin configurations
  panicButtonPin = buttonPin;
  buzzerPin = buzzer;
  ledPin = led;

  // Set up module references
  gpsModule = gps;
  gsmModule = gsm;

  // Configure LED pin
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // Configure buzzer pin
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);

  // Configure panic button with pull-up
  pinMode(panicButtonPin, INPUT_PULLUP);

  // Register this instance for interrupt handling
  sosModuleInstance = this;

  // Attach interrupt for button press (falling edge = button press with pull-up)
  attachInterrupt(digitalPinToInterrupt(panicButtonPin), buttonISRWrapper, FALLING);

  Serial.println("SOS module initialized");
}

// Button interrupt handler
void SOSModule::buttonISR() {
  unsigned long currentTime = millis();

  // Debounce logic
  if ((currentTime - lastButtonPressTime) > PANIC_BUTTON_DEBOUNCE_MS) {
    buttonPressed = true;

    // Count presses for double-press detection
    if ((currentTime - lastButtonPressTime) < DOUBLE_PRESS_TIMEOUT_MS) {
      buttonPressCount++;
    } else {
      buttonPressCount = 1;
    }

    lastButtonPressTime = currentTime;
  }
}

// Main update function (call this in loop())
void SOSModule::update() {
  // Check if panic button was pressed
  if (buttonPressed) {
    buttonPressed = false;
    handlePanicButtonPress();
  }

  // Update LED flashing if active
  if (ledFlashing) {
    updateLedFlashing();
  }

  // Update alarm sound if active
  if (alarmActive) {
    updateAlarmSound();
  }

  // Check for alert timeout
  if (alertActive) {
    unsigned long currentTime = millis();
    if ((currentTime - alertStartTime) / 1000 >= ALERT_TIMEOUT_SECONDS) {
      Serial.println("Alert timeout reached, canceling alert");
      cancelAlert();
    }
  }
}

// Handle panic button press logic
void SOSModule::handlePanicButtonPress() {
  // Get current time
  unsigned long currentTime = millis();

  // Check if we need to wait for potential double press
  if (buttonPressCount == 1 && (currentTime - lastButtonPressTime) < DOUBLE_PRESS_TIMEOUT_MS) {
    // Wait a bit more to see if a second press comes
    return;
  }

  if (buttonPressCount >= 2) {
    // Double press - activate alarm and alert
    Serial.println("Panic button double press detected - triggering loud alarm");
    playAlarm(true);
    flashLed(true);
    triggerAlert(SOS_ALERT_TYPE_PANIC_BUTTON);
    buttonPressCount = 0;
  } else {
    // Single press - silent alert
    Serial.println("Panic button single press detected - triggering silent alert");
    triggerAlert(SOS_ALERT_TYPE_PANIC_BUTTON);
    buttonPressCount = 0;
  }
}

// Trigger an alert
void SOSModule::triggerAlert(SOSAlertType alertType) {
  if (alertActive) {
    Serial.println("Alert already active, retriggering");
  }

  alertActive = true;
  alertStartTime = millis();

  // Get current GPS data
  GPSData gpsData;
  bool gpsValid = false;

  // Try to get GPS data if module is available
  if (gpsModule) {
    gpsValid = gpsModule->getData(&gpsData);
  }

  // Record event data
  lastEvent.type = alertType;
  lastEvent.timestamp = millis() / 1000; // Convert to seconds
  lastEvent.gpsValid = gpsValid;

  if (gpsValid) {
    lastEvent.latitude = gpsData.latitude;
    lastEvent.longitude = gpsData.longitude;
  }

  // Send emergency alert via GSM if available
  if (gsmModule) {
    if (gpsValid) {
      gsmModule->sendEmergencyAlert(&gpsData);
    } else {
      Serial.println("No valid GPS data for emergency alert");

      // Simplified sending without GPS coordinates
      char message[128];
      snprintf(message, sizeof(message),
              "EMERGENCY ALERT!\n"
              "Location not available.\n"
              "Please provide immediate assistance!");

      // In a real implementation, we'd need to send to each contact
      // For now, just show the message
      Serial.println(message);
    }
  } else {
    Serial.println("No GSM module available for sending alert");
  }

  Serial.print("Alert triggered: type=");
  Serial.print(alertType);
  Serial.print(", GPS valid=");
  Serial.println(gpsValid);
}

// Cancel active alert
void SOSModule::cancelAlert() {
  if (!alertActive) {
    return;
  }

  // Turn off alarm and LED
  playAlarm(false);
  flashLed(false);

  alertActive = false;
  Serial.println("Alert canceled");
}

// Get the last recorded SOS event
SOSEvent SOSModule::getLastEvent() const {
  return lastEvent;
}

// Check if alert is currently active
bool SOSModule::isAlertActive() const {
  return alertActive;
}

// Update LED flashing state
void SOSModule::updateLedFlashing() {
  unsigned long currentTime = millis();

  // Flash pattern: on for 100ms, off for 100ms
  if (currentTime - lastLedToggleTime >= 100) {
    static bool ledState = false;
    ledState = !ledState;
    digitalWrite(ledPin, ledState ? HIGH : LOW);
    lastLedToggleTime = currentTime;
  }
}

// Control LED flashing
bool SOSModule::flashLed(bool enable) {
  ledFlashing = enable;

  if (!enable) {
    // Turn off LED when disabling flashing
    digitalWrite(ledPin, LOW);
  }

  Serial.print("LED flashing ");
  Serial.println(enable ? "activated" : "deactivated");
  return true;
}

// Update alarm sound state
void SOSModule::updateAlarmSound() {
  alarmSoundPattern();
}

// SOS Morse code sound pattern
void SOSModule::alarmSoundPattern() {
  // Implementation of the SOS pattern state machine
  unsigned long currentTime = millis();

  if (currentTime - lastAlarmUpdateTime < 100) {
    return; // Not time to update yet
  }

  lastAlarmUpdateTime = currentTime;

  // This is a simplified state machine approach to generate the SOS pattern
  // States 0-5: short beeps (S)
  // States 6-11: pause between letters
  // States 12-23: long beeps (O)
  // States 24-29: pause between letters
  // States 30-35: short beeps (S)
  // States 36-45: long pause before repeating

  // Determine if buzzer should be on or off
  bool buzzerOn = false;

  if (alarmSequenceState < 6) {
    // S: three short beeps
    buzzerOn = (alarmSequenceState % 2 == 0);
  }
  else if (alarmSequenceState < 12) {
    // Pause between letters
    buzzerOn = false;
  }
  else if (alarmSequenceState < 24) {
    // O: three long beeps
    int longBeepState = (alarmSequenceState - 12) / 4;
    int subState = (alarmSequenceState - 12) % 4;
    buzzerOn = (subState < 3); // 3 on, 1 off for each long beep
  }
  else if (alarmSequenceState < 30) {
    // Pause between letters
    buzzerOn = false;
  }
  else if (alarmSequenceState < 36) {
    // S: three short beeps
    buzzerOn = ((alarmSequenceState - 30) % 2 == 0);
  }
  else {
    // Long pause before repeating
    buzzerOn = false;
  }

  // Set buzzer state
  digitalWrite(buzzerPin, buzzerOn ? HIGH : LOW);

  // Move to next state
  alarmSequenceState = (alarmSequenceState + 1) % 46;
}

// Control the alarm sound
bool SOSModule::playAlarm(bool enable) {
  if (enable) {
    if (!alarmActive) {
      alarmActive = true;
      alarmSequenceState = 0;
      Serial.println("Alarm activated");
    }
  } else {
    alarmActive = false;
    digitalWrite(buzzerPin, LOW); // Ensure buzzer is off
    Serial.println("Alarm deactivated");
  }

  return true;
}
