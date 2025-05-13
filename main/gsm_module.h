#ifndef GSM_MODULE_H
#define GSM_MODULE_H

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include "gps_module.h"

// Enum for GSM module status
enum GSMStatus {
    GSM_STATUS_OFF,
    GSM_STATUS_INITIALIZING,
    GSM_STATUS_READY,
    GSM_STATUS_ERROR
};

// Maximum number of emergency contacts
#define MAX_EMERGENCY_CONTACTS 5

// EEPROM address definitions
#define EEPROM_INITIALIZED_MARKER 0xAA
#define EEPROM_INITIALIZED_ADDR 0
#define EEPROM_NUM_CONTACTS_ADDR 1
#define EEPROM_CONTACTS_START_ADDR 2
#define CONTACT_SIZE 20  // Max length of a phone number

class GSMModule {
private:
    SoftwareSerial* gsmSerial;
    GSMStatus gsmStatus;
    char emergencyContacts[MAX_EMERGENCY_CONTACTS][CONTACT_SIZE];
    int numEmergencyContacts;
    bool gsmInitialized;

    // Timeout constants
    static const uint32_t AT_TIMEOUT_MS = 3000;
    static const uint32_t AT_RESPONSE_TIMEOUT_MS = 5000;

    // Helper function to send AT command and wait for response
    bool sendATCommand(const String& command, const String& expectedResponse, uint32_t timeout);

    // Save emergency contacts to EEPROM
    void saveEmergencyContacts();

    // Load emergency contacts from EEPROM
    void loadEmergencyContacts();

    // Initialize EEPROM if needed
    void initializeEEPROM();

public:
    // Constructor
    GSMModule(int rxPin, int txPin, int baudRate = 9600);

    // Initialize GSM module
    bool begin();

    // Get current GSM module status
    GSMStatus getStatus() const;

    // Send SMS message
    bool sendSMS(const String& phoneNumber, const String& message);

    // Send emergency alert to all registered contacts
    bool sendEmergencyAlert(const GPSData* gpsData);

    // Put GSM into low power mode
    void sleep();

    // Wake up GSM from low power mode
    void wake();

    // Add emergency contact
    bool addEmergencyContact(const String& phoneNumber);

    // Remove emergency contact
    bool removeEmergencyContact(const String& phoneNumber);

    // Clear all emergency contacts
    void clearEmergencyContacts();

    // Get number of emergency contacts
    int getNumEmergencyContacts() const;

    // Get a specific emergency contact
    String getEmergencyContact(int index) const;
};

#endif // GSM_MODULE_H
