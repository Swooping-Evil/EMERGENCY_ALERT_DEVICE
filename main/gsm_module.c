// gsm_module.c
#include "gsm_module.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"

#define GSM_UART_NUM UART_NUM_1
#define GSM_UART_TX_PIN 25
#define GSM_UART_RX_PIN 26
#define GSM_UART_BAUD_RATE 9600
#define GSM_BUF_SIZE 1024
#define AT_TIMEOUT_MS 3000
#define AT_RESPONSE_TIMEOUT_MS 5000
#define NVS_NAMESPACE "gsm_contacts"

static const char* TAG = "GSM_MODULE";
static GSMStatus gsmStatus = GSM_STATUS_OFF;
static char emergencyContacts[MAX_EMERGENCY_CONTACTS][20] = {0};
static int numEmergencyContacts = 0;
static bool gsmInitialized = false;

// Helper function to send AT command and wait for response
static bool sendATCommand(const char* command, const char* expectedResponse, uint32_t timeout) {
    char responseBuffer[GSM_BUF_SIZE];
    int responseLength = 0;

    // Clear any existing data
    uart_flush(GSM_UART_NUM);

    // Send the command
    ESP_LOGI(TAG, "Sending AT command: %s", command);
    uart_write_bytes(GSM_UART_NUM, command, strlen(command));
    uart_write_bytes(GSM_UART_NUM, "\r\n", 2);

    // Wait for response
    uint64_t startTime = esp_timer_get_time() / 1000;
    bool responseFound = false;

    while ((esp_timer_get_time() / 1000 - startTime) < timeout) {
        int length = uart_read_bytes(GSM_UART_NUM, (uint8_t*)&responseBuffer[responseLength],
                                    GSM_BUF_SIZE - responseLength - 1, 100 / portTICK_PERIOD_MS);

        if (length > 0) {
            responseLength += length;
            responseBuffer[responseLength] = 0; // Null-terminate

            // Check if we got the expected response
            if (strstr(responseBuffer, expectedResponse) != NULL) {
                responseFound = true;
                break;
            }

            // Check for error
            if (strstr(responseBuffer, "ERROR") != NULL) {
                ESP_LOGE(TAG, "AT command error: %s", responseBuffer);
                break;
            }
        }
    }

    if (!responseFound) {
        ESP_LOGE(TAG, "AT command timeout: %s", command);
        ESP_LOGE(TAG, "Response: %s", responseBuffer);
    }

    return responseFound;
}

// Load emergency contacts from NVS
static void loadEmergencyContacts() {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS: %s", esp_err_to_name(err));
        return;
    }

    // Read number of contacts
    err = nvs_get_i32(nvs_handle, "num_contacts", &numEmergencyContacts);
    if (err != ESP_OK) {
        numEmergencyContacts = 0;
    }

    // Ensure valid range
    if (numEmergencyContacts > MAX_EMERGENCY_CONTACTS) {
        numEmergencyContacts = MAX_EMERGENCY_CONTACTS;
    }

    // Read each contact
    for (int i = 0; i < numEmergencyContacts; i++) {
        char key[16];
        snprintf(key, sizeof(key), "contact_%d", i);

        size_t length = sizeof(emergencyContacts[i]);
        err = nvs_get_str(nvs_handle, key, emergencyContacts[i], &length);

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Error reading contact %d: %s", i, esp_err_to_name(err));
        }
    }

    nvs_close(nvs_handle);
    ESP_LOGI(TAG, "Loaded %d emergency contacts", numEmergencyContacts);
}

// Save emergency contacts to NVS
static void saveEmergencyContacts() {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS: %s", esp_err_to_name(err));
        return;
    }

    // Store number of contacts
    err = nvs_set_i32(nvs_handle, "num_contacts", numEmergencyContacts);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error saving contact count: %s", esp_err_to_name(err));
    }

    // Store each contact
    for (int i = 0; i < numEmergencyContacts; i++) {
        char key[16];
        snprintf(key, sizeof(key), "contact_%d", i);

        err = nvs_set_str(nvs_handle, key, emergencyContacts[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Error saving contact %d: %s", i, esp_err_to_name(err));
        }
    }

    // Commit changes
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error committing NVS: %s", esp_err_to_name(err));
    }

    nvs_close(nvs_handle);
    ESP_LOGI(TAG, "Saved %d emergency contacts", numEmergencyContacts);
}

// Initialize GSM module
void gsm_init() {
    if (gsmInitialized) return;

    // Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = GSM_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    // Install UART driver
    ESP_ERROR_CHECK(uart_driver_install(GSM_UART_NUM, GSM_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(GSM_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(GSM_UART_NUM, GSM_UART_TX_PIN, GSM_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    gsmStatus = GSM_STATUS_INITIALIZING;

    // Wait for module to stabilize
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    // Load emergency contacts
    loadEmergencyContacts();

    // Initialize GSM module with AT commands
    bool success = true;

    // Test AT communication
    success &= sendATCommand("AT", "OK", AT_TIMEOUT_MS);

    // Enable echo
    success &= sendATCommand("ATE1", "OK", AT_TIMEOUT_MS);

    // Check SIM card
    success &= sendATCommand("AT+CPIN?", "READY", AT_TIMEOUT_MS);

    // Set SMS text mode
    success &= sendATCommand("AT+CMGF=1", "OK", AT_TIMEOUT_MS);

    // Configure SMS notification
    success &= sendATCommand("AT+CNMI=2,2,0,0,0", "OK", AT_TIMEOUT_MS);

    // If all initialization steps were successful
    if (success) {
        gsmStatus = GSM_STATUS_READY;
        gsmInitialized = true;
        ESP_LOGI(TAG, "GSM module initialized successfully");
    } else {
        gsmStatus = GSM_STATUS_ERROR;
        ESP_LOGE(TAG, "GSM module initialization failed");
    }
}

// Get current GSM module status
GSMStatus gsm_getStatus() {
    return gsmStatus;
}

// Send SMS message
bool gsm_sendSMS(const char* phoneNumber, const char* message) {
    if (gsmStatus != GSM_STATUS_READY) {
        ESP_LOGE(TAG, "GSM module not ready, status: %d", gsmStatus);
        return false;
    }

    char command[64];
    snprintf(command, sizeof(command), "AT+CMGS=\"%s\"", phoneNumber);

    // Clear any existing data
    uart_flush(GSM_UART_NUM);

    // Send SMS command
    ESP_LOGI(TAG, "Sending SMS to: %s", phoneNumber);
    uart_write_bytes(GSM_UART_NUM, command, strlen(command));
    uart_write_bytes(GSM_UART_NUM, "\r", 1);

    // Wait for '>' prompt
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // Send message content
    uart_write_bytes(GSM_UART_NUM, message, strlen(message));

    // Send Ctrl+Z to finish message
    uint8_t ctrlZ = 0x1A;
    uart_write_bytes(GSM_UART_NUM, &ctrlZ, 1);

    // Wait for response
    char responseBuffer[GSM_BUF_SIZE] = {0};
    uint64_t startTime = esp_timer_get_time() / 1000;
    bool sent = false;

    while ((esp_timer_get_time() / 1000 - startTime) < AT_RESPONSE_TIMEOUT_MS) {
        int length = uart_read_bytes(GSM_UART_NUM, (uint8_t*)responseBuffer, GSM_BUF_SIZE - 1, 500 / portTICK_PERIOD_MS);

        if (length > 0) {
            responseBuffer[length] = 0;

            // Check for success
            if (strstr(responseBuffer, "+CMGS:") != NULL && strstr(responseBuffer, "OK") != NULL) {
                sent = true;
                break;
            }

            // Check for error
            if (strstr(responseBuffer, "ERROR") != NULL) {
                ESP_LOGE(TAG, "SMS send error: %s", responseBuffer);
                break;
            }
        }
    }

    if (sent) {
        ESP_LOGI(TAG, "SMS sent successfully");
    } else {
        ESP_LOGE(TAG, "Failed to send SMS");
    }

    return sent;
}

// Send emergency alert to all registered contacts
bool gsm_sendEmergencyAlert(const GPSData* gpsData) {
    if (numEmergencyContacts == 0) {
        ESP_LOGE(TAG, "No emergency contacts registered");
        return false;
    }

    if (!gpsData->isValid) {
        ESP_LOGE(TAG, "GPS data not valid for emergency alert");
        return false;
    }

    bool allSent = true;

    // Prepare emergency message
    char message[512];
    snprintf(message, sizeof(message),
             "EMERGENCY ALERT!\n"
             "Location Place\n"
             "Latitude: %.6f,N\n"
             "Longitude: %.6f,E\n"
             "Please provide immediate assistance!\n"
             "https://www.google.com/maps/place/%.6f,%.6f",
             gpsData->latitude, gpsData->longitude,
             gpsData->latitude, gpsData->longitude);

    // Send to all emergency contacts
    for (int i = 0; i < numEmergencyContacts; i++) {
        ESP_LOGI(TAG, "Sending emergency alert to contact %d: %s", i, emergencyContacts[i]);
        bool sent = gsm_sendSMS(emergencyContacts[i], message);

        if (!sent) {
            allSent = false;
            ESP_LOGE(TAG, "Failed to send emergency alert to contact %d", i);
        }

        // Small delay between messages
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    return allSent;
}

// Put GSM into low power mode
void gsm_sleep() {
    if (gsmStatus == GSM_STATUS_READY) {
        // Send sleep command - this depends on the specific GSM module
        sendATCommand("AT+CSCLK=2", "OK", AT_TIMEOUT_MS);
        ESP_LOGI(TAG, "GSM module entering sleep mode");
    }
}

// Wake up GSM from low power mode
void gsm_wake() {
    if (gsmStatus != GSM_STATUS_OFF) {
        // Send any AT command to wake up
        sendATCommand("AT", "OK", AT_TIMEOUT_MS);

        // Disable sleep mode
        sendATCommand("AT+CSCLK=0", "OK", AT_TIMEOUT_MS);
        ESP_LOGI(TAG, "GSM module waking up");
    }
}

// Add emergency contact
bool gsm_addEmergencyContact(const char* phoneNumber) {
    if (numEmergencyContacts >= MAX_EMERGENCY_CONTACTS) {
        ESP_LOGE(TAG, "Maximum number of emergency contacts reached");
        return false;
    }

    // Validate phone number (basic check)
    if (strlen(phoneNumber) < 6 || strlen(phoneNumber) > 15) {
        ESP_LOGE(TAG, "Invalid phone number: %s", phoneNumber);
        return false;
    }

    // Check if contact already exists
    for (int i = 0; i < numEmergencyContacts; i++) {
        if (strcmp(emergencyContacts[i], phoneNumber) == 0) {
            ESP_LOGI(TAG, "Contact already exists: %s", phoneNumber);
            return true;
        }
    }

    // Add new contact
    strncpy(emergencyContacts[numEmergencyContacts], phoneNumber, sizeof(emergencyContacts[0]) - 1);
    numEmergencyContacts++;

    // Save to NVS
    saveEmergencyContacts();

    ESP_LOGI(TAG, "Added emergency contact: %s", phoneNumber);
    return true;
}

// Remove emergency contact
bool gsm_removeEmergencyContact(const char* phoneNumber) {
    bool found = false;

    for (int i = 0; i < numEmergencyContacts; i++) {
        if (strcmp(emergencyContacts[i], phoneNumber) == 0) {
            found = true;

            // Remove by shifting remaining contacts
            for (int j = i; j < numEmergencyContacts - 1; j++) {
                strcpy(emergencyContacts[j], emergencyContacts[j + 1]);
            }

            numEmergencyContacts--;
            break;
        }
    }

    if (found) {
        saveEmergencyContacts();
        ESP_LOGI(TAG, "Removed emergency contact: %s", phoneNumber);
    } else {
        ESP_LOGE(TAG, "Contact not found: %s", phoneNumber);
    }

    return found;
}

// Clear all emergency contacts
void gsm_clearEmergencyContacts() {
    numEmergencyContacts = 0;
    saveEmergencyContacts();
    ESP_LOGI(TAG, "Cleared all emergency contacts");
}
