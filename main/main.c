#include <Arduino.h>
#include "gps_module.h"
#include "gsm_module.h"
#include "sos_module.h"
#include "motion_detection.h"
#include "sensor_fusion.h"
#include "mpu.h"
// Pin definitions
const int PANIC_BUTTON_PIN = 2;  // Use pin 2 for interrupt capability
const int BUZZER_PIN = 9;        // PWM pin for buzzer
const int LED_PIN = 13;          // Built-in LED for status indication

// GPS pins (RX, TX)
const int GPS_RX_PIN = 4;
const int GPS_TX_PIN = 5;

// GSM pins (RX, TX)
const int GSM_RX_PIN = 6;
const int GSM_TX_PIN = 7;

// Create module instances
GPSModule gps(GPS_RX_PIN, GPS_TX_PIN);
GSMModule gsm(GSM_RX_PIN, GSM_TX_PIN);
SOSModule sos;

// Forward declarations
void onFallDetected();

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  Serial.println("Emergency Alert System starting...");

  // Initialize modules
  if (!gps.begin()) {
    Serial.println("GPS initialization failed!");
  }

  if (!gsm.begin()) {
    Serial.println("GSM initialization failed!");
  }

  // Add emergency contacts to GSM module
  gsm.addEmergencyContact("1234567890");  // Replace with actual emergency numbers
  gsm.addEmergencyContact("0987654321");  // Replace with actual emergency numbers

  // Initialize SOS module with pins and module references
  sos.begin(PANIC_BUTTON_PIN, BUZZER_PIN, LED_PIN, &gps, &gsm);

  // Perform a brief startup test
  testSystem();

  Serial.println("System initialization complete.");
}

void loop() {
  // Update GPS data
  gps.update();

  // Update SOS module (processes button presses, alarms, etc.)
  sos.update();

  // Handle any fall detection from motion sensor (would be implemented separately)
  handleMotionSensor();

  // Brief delay to prevent tight loop
  delay(10);
}

// Test system components on startup
void testSystem() {
  Serial.println("Running system test...");

  // Test LED
  Serial.println("Testing LED...");
  sos.flashLed(true);
  delay(1000);
  sos.flashLed(false);

  // Test buzzer with brief sound
  Serial.println("Testing buzzer...");
  sos.playAlarm(true);
  delay(500);
  sos.playAlarm(false);

  // Test GPS
  Serial.println("Testing GPS...");
  GPSData gpsData;
  bool gpsOk = gps.getData(&gpsData);

  if (gpsOk) {
    Serial.print("GPS OK - Lat: ");
    Serial.print(gpsData.latitude, 6);
    Serial.print(", Lon: ");
    Serial.println(gpsData.longitude, 6);
  } else {
    Serial.println("GPS not fixed yet. This is normal during startup.");
  }

  // Test GSM (limited test)
  Serial.println("Testing GSM...");
  if (gsm.getStatus() == GSM_STATUS_READY) {
    Serial.println("GSM module ready");
  } else {
    Serial.println("GSM module not ready yet");
  }

  Serial.println("System test complete");
}

// Simulate motion sensor handling
void handleMotionSensor() {
  // This would integrate with your MotionDetection module
  // For testing, we'll simulate a fall detection with a button press on a different pin

  const int MOTION_TEST_PIN = 3;  // Use pin 3 for motion test
  static bool pinInitialized = false;

  if (!pinInitialized) {
    pinMode(MOTION_TEST_PIN, INPUT_PULLUP);
    pinInitialized = true;
  }

  // Check if fall simulation button is pressed
  if (digitalRead(MOTION_TEST_PIN) == LOW) {
    static unsigned long lastPressTime = 0;
    unsigned long currentTime = millis();

    // Simple debounce
    if (currentTime - lastPressTime > 1000) {
      Serial.println("Fall detected! (simulated)");
      onFallDetected();
      lastPressTime = currentTime;
    }
  }
}

// Callback for fall detection
void onFallDetected() {
  // Trigger alert with fall type
  sos.triggerAlert(SOS_ALERT_TYPE_FALL);

  // Also enable LED and sound for fall detection
  sos.flashLed(true);
  sos.playAlarm(true);
}
