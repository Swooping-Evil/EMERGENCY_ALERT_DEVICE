#include "gsm_module.h"

GSMModule::GSMModule(int rxPin, int txPin, int baudRate)
    : gsmSerial(new SoftwareSerial(rxPin, txPin)),
      gsmStatus(GSM_STATUS_OFF),
      numEmergencyContacts(0),
      gsmInitialized(false) {
    gsmSerial->begin(baudRate);
}

bool GSMModule::begin() {
    if (gsmInitialized) return true;

    gsmStatus = GSM_STATUS_INITIALIZING;

    // Initialize EEPROM on first use
    initializeEEPROM();

    // Wait for module to stabilize
    delay(2000);

    // Load emergency contacts from EEPROM
    loadEmergencyContacts();

    // Initialize GSM module with AT commands
    bool success = true;

    // Test AT communication
    success &= sendATCommand("AT", "OK", AT_TIMEOUT_MS);

    // Enable echo
    success &= sendATCommand("ATE1", "OK", AT_TIMEOUT_MS);

    // Set SMS text mode
    success &= sendATCommand("AT+CMGF=1", "OK", AT_TIMEOUT_MS);

    // Configure SMS notification
    success &= sendATCommand("AT+CNMI=2,2,0,0,0", "OK", AT_TIMEOUT_MS);

    if (success) {
        gsmStatus = GSM_STATUS_READY;
        gsmInitialized = true;
    } else {
        gsmStatus = GSM_STATUS_ERROR;
    }

    return gsmInitialized;
}

bool GSMModule::sendATCommand(const String& command, const String& expectedResponse, uint32_t timeout) {
    // Clear any existing data
    while (gsmSerial->available()) gsmSerial->read();

    // Send the command
    gsmSerial->println(command);

    // Wait for response
    unsigned long startTime = millis();
    String responseBuffer = "";

    while (millis() - startTime < timeout) {
        if (gsmSerial->available()) {
            char c = gsmSerial->read();
            responseBuffer += c;

            // Check if we got the expected response
            if (responseBuffer.indexOf(expectedResponse) != -1) {
                return true;
            }

            // Check for error
            if (responseBuffer.indexOf("ERROR") != -1) {
                return false;
            }
        }
    }

    return false;
}

void GSMModule::initializeEEPROM() {
    // Check if EEPROM has been initialized
    byte initialized = EEPROM.read(EEPROM_INITIALIZED_ADDR);

    if (initialized != EEPROM_INITIALIZED_MARKER) {
        // First time use, initialize EEPROM
        EEPROM.write(EEPROM_INITIALIZED_ADDR, EEPROM_INITIALIZED_MARKER);
        EEPROM.write(EEPROM_NUM_CONTACTS_ADDR, 0);  // No contacts initially

        // Clear contact storage area
        for (int i = 0; i < MAX_EMERGENCY_CONTACTS * CONTACT_SIZE; i++) {
            EEPROM.write(EEPROM_CONTACTS_START_ADDR + i, 0);
        }

        #ifdef ESP32
        EEPROM.commit();
        #endif
    }
}

void GSMModule::loadEmergencyContacts() {
    // Read number of contacts
    numEmergencyContacts = EEPROM.read(EEPROM_NUM_CONTACTS_ADDR);

    // Make sure we don't exceed array bounds
    if (numEmergencyContacts > MAX_EMERGENCY_CONTACTS) {
        numEmergencyContacts = MAX_EMERGENCY_CONTACTS;
    }

    // Read each contact
    for (int i = 0; i < numEmergencyContacts; i++) {
        int contactAddr = EEPROM_CONTACTS_START_ADDR + (i * CONTACT_SIZE);

        // Read the contact character by character
        for (int j = 0; j < CONTACT_SIZE; j++) {
            emergencyContacts[i][j] = EEPROM.read(contactAddr + j);
        }

        // Ensure null termination
        emergencyContacts[i][CONTACT_SIZE - 1] = '\0';
    }
}

void GSMModule::saveEmergencyContacts() {
    // Save number of contacts
    EEPROM.write(EEPROM_NUM_CONTACTS_ADDR, numEmergencyContacts);

    // Save each contact
    for (int i = 0; i < numEmergencyContacts; i++) {
        int contactAddr = EEPROM_CONTACTS_START_ADDR + (i * CONTACT_SIZE);

        // Save the contact character by character
        for (int j = 0; j < CONTACT_SIZE; j++) {
            EEPROM.write(contactAddr + j, emergencyContacts[i][j]);
        }
    }

    // For ESP32, we need to commit the changes
    #ifdef ESP32
    EEPROM.commit();
    #endif
}

GSMStatus GSMModule::getStatus() const {
    return gsmStatus;
}

bool GSMModule::sendSMS(const String& phoneNumber, const String& message) {
    if (gsmStatus != GSM_STATUS_READY) return false;

    // Prepare and send SMS
    gsmSerial->print("AT+CMGS=\"");
    gsmSerial->print(phoneNumber);
    gsmSerial->println("\"");
    delay(100);

    // Send message content
    gsmSerial->print(message);

    // Send Ctrl+Z to finish message
    gsmSerial->write(26);

    // Wait for response
    unsigned long startTime = millis();
    String responseBuffer = "";

    while (millis() - startTime < AT_RESPONSE_TIMEOUT_MS) {
        if (gsmSerial->available()) {
            char c = gsmSerial->read();
            responseBuffer += c;

            // Check for success
            if (responseBuffer.indexOf("+CMGS:") != -1 && responseBuffer.indexOf("OK") != -1) {
                return true;
            }

            // Check for error
            if (responseBuffer.indexOf("ERROR") != -1) {
                return false;
            }
        }
    }

    return false;
}

bool GSMModule::sendEmergencyAlert(const GPSData* gpsData) {
    if (numEmergencyContacts == 0 || !gpsData->isValid) return false;

    bool allSent = true;

    // Prepare emergency message
    String message = "EMERGENCY ALERT!\n"
                     "Location Place\n"
                     "Latitude: " + String(gpsData->latitude, 6) + ",N\n"
                     "Longitude: " + String(gpsData->longitude, 6) + ",E\n"
                     "Please provide immediate assistance!\n"
                     "https://www.google.com/maps/place/" +
                     String(gpsData->latitude, 6) + "," +
                     String(gpsData->longitude, 6);

    // Send to all emergency contacts
    for (int i = 0; i < numEmergencyContacts; i++) {
        bool sent = sendSMS(emergencyContacts[i], message);

        if (!sent) {
            allSent = false;
        }

        // Small delay between messages
        delay(1000);
    }

    return allSent;
}

void GSMModule::sleep() {
    if (gsmStatus == GSM_STATUS_READY) {
        // Send sleep command - this depends on the specific GSM module
        sendATCommand("AT+CSCLK=2", "OK", AT_TIMEOUT_MS);
    }
}

void GSMModule::wake() {
    if (gsmStatus != GSM_STATUS_OFF) {
        // Send any AT command to wake up
        sendATCommand("AT", "OK", AT_TIMEOUT_MS);

        // Disable sleep mode
        sendATCommand("AT+CSCLK=0", "OK", AT_TIMEOUT_MS);
    }
}

bool GSMModule::addEmergencyContact(const String& phoneNumber) {
    if (numEmergencyContacts >= MAX_EMERGENCY_CONTACTS) return false;

    // Validate phone number (basic check)
    if (phoneNumber.length() < 6 || phoneNumber.length() > 15) return false;

    // Check if contact already exists
    for (int i = 0; i < numEmergencyContacts; i++) {
        if (String(emergencyContacts[i]) == phoneNumber) return true;
    }

    // Add new contact
    phoneNumber.toCharArray(emergencyContacts[numEmergencyContacts], sizeof(emergencyContacts[0]));
    numEmergencyContacts++;

    // Save to storage
    saveEmergencyContacts();

    return true;
}

bool GSMModule::removeEmergencyContact(const String& phoneNumber) {
    for (int i = 0; i < numEmergencyContacts; i++) {
        if (String(emergencyContacts[i]) == phoneNumber) {
            // Remove by shifting remaining contacts
            for (int j = i; j < numEmergencyContacts - 1; j++) {
                strcpy(emergencyContacts[j], emergencyContacts[j + 1]);
            }

            numEmergencyContacts--;
            saveEmergencyContacts();
            return true;
        }
    }

    return false;
}

void GSMModule::clearEmergencyContacts() {
    numEmergencyContacts = 0;
    saveEmergencyContacts();
}

int GSMModule::getNumEmergencyContacts() const {
    return numEmergencyContacts;
}

String GSMModule::getEmergencyContact(int index) const {
    if (index >= 0 && index < numEmergencyContacts) {
        return String(emergencyContacts[index]);
    }
    return String("");
}
