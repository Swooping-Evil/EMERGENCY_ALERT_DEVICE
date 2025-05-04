// sos_module.c
#include "sos_module.h"
#include "gps_module.h"
#include "gsm_module.h"
#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/ledc.h"

#define PANIC_BUTTON_DEBOUNCE_MS 50
#define DOUBLE_PRESS_TIMEOUT_MS 500
#define ALERT_TIMEOUT_SECONDS 300 // 5 minutes
#define BUZZER_PIN 6
#define LED_PIN 7
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT
#define LEDC_FREQUENCY 4000

static const char* TAG = "SOS_MODULE";
static int panicButtonPin = -1;
static bool buttonPressed = false;
static uint64_t lastButtonPressTime = 0;
static int buttonPressCount = 0;
static bool alertActive = false;
static SOSEvent lastEvent = {SOS_ALERT_TYPE_NONE, 0, false, 0.0, 0.0};
static bool alarmActive = false;
static bool ledFlashing = false;
static esp_timer_handle_t alert_timer = NULL;
static TaskHandle_t flashLedTaskHandle = NULL;
static TaskHandle_t alarmSoundTaskHandle = NULL;

// Forward declarations for internal functions
static void IRAM_ATTR panic_button_isr(void* arg);
static void alert_timeout_callback(void* arg);
static void flash_led_task(void* param);
static void alarm_sound_task(void* param);

// Initialize SOS module
void sos_init() {
    // Setup buzzer PWM
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // sos_module.c (continued)
    // Configure LED pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Configure buzzer PWM channel
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = BUZZER_PIN,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel);

    // Create alert timeout timer
    const esp_timer_create_args_t timer_args = {
        .callback = &alert_timeout_callback,
        .name = "alert_timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &alert_timer));

    ESP_LOGI(TAG, "SOS module initialized");
}

// Register panic button pin and configure interrupt
void sos_registerPanicButtonPin(int buttonPin) {
    panicButtonPin = buttonPin;

    // Configure panic button with pull-up
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << panicButtonPin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE // Trigger on falling edge (button press)
    };
    gpio_config(&io_conf);

    // Install GPIO interrupt service
    gpio_install_isr_service(0);

    // Hook ISR handler for panic button
    gpio_isr_handler_add(panicButtonPin, panic_button_isr, NULL);

    ESP_LOGI(TAG, "Panic button registered on pin %d", panicButtonPin);
}

// Interrupt handler for panic button
static void IRAM_ATTR panic_button_isr(void* arg) {
    uint64_t currentTime = esp_timer_get_time() / 1000; // Convert to ms

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

// Check if panic button was pressed and process
bool sos_checkPanicButton() {
    if (!buttonPressed) {
        return false;
    }

    buttonPressed = false;

    // Get current time
    uint64_t currentTime = esp_timer_get_time() / 1000;

    // Check if we need to wait for potential double press
    if (buttonPressCount == 1 &&
        (currentTime - lastButtonPressTime) < DOUBLE_PRESS_TIMEOUT_MS) {
        // Wait a bit more to see if a second press comes
        return false;
    }

    if (buttonPressCount >= 2) {
        // Double press - activate alarm and alert
        ESP_LOGI(TAG, "Panic button double press detected - triggering loud alarm");
        sos_playAlarm(true);
        sos_flashLed(true);
        sos_triggerAlert(SOS_ALERT_TYPE_PANIC_BUTTON);
        buttonPressCount = 0;
    } else {
        // Single press - silent alert
        ESP_LOGI(TAG, "Panic button single press detected - triggering silent alert");
        sos_triggerAlert(SOS_ALERT_TYPE_PANIC_BUTTON);
        buttonPressCount = 0;
    }

    return true;
}

// Check if alert is currently active
bool sos_isAlertActive() {
    return alertActive;
}

// Trigger an alert
void sos_triggerAlert(SOSAlertType alertType) {
    if (alertActive) {
        ESP_LOGW(TAG, "Alert already active, retriggering");
    }

    alertActive = true;

    // Get current GPS data
    GPSData gpsData;
    bool gpsValid = gps_getData(&gpsData);

    // Record event data
    lastEvent.type = alertType;
    lastEvent.timestamp = (uint32_t)(esp_timer_get_time() / 1000000); // Convert to seconds
    lastEvent.gpsValid = gpsValid;

    if (gpsValid) {
        lastEvent.latitude = gpsData.latitude;
        lastEvent.longitude = gpsData.longitude;
    }

    // Send emergency alert via GSM
    if (gpsValid) {
        gsm_sendEmergencyAlert(&gpsData);
    } else {
        ESP_LOGW(TAG, "No valid GPS data for emergency alert");

        // Send alert without coordinates
        char message[256];
        snprintf(message, sizeof(message),
                "EMERGENCY ALERT!\n"
                "Location not available.\n"
                "Please provide immediate assistance!");

        // Send to all emergency contacts
        // This would need to iterate through contacts like in gsm_sendEmergencyAlert
    }

    // Start alert timeout timer
    esp_timer_start_once(alert_timer, ALERT_TIMEOUT_SECONDS * 1000000);

    ESP_LOGI(TAG, "Alert triggered: type=%d, GPS valid=%d", alertType, gpsValid);
}

// Cancel active alert
void sos_cancelAlert() {
    if (!alertActive) {
        return;
    }

    // Stop timer
    esp_timer_stop(alert_timer);

    // Turn off alarm and LED
    sos_playAlarm(false);
    sos_flashLed(false);

    alertActive = false;
    ESP_LOGI(TAG, "Alert canceled");
}

// Get the last recorded SOS event
SOSEvent sos_getLastEvent() {
    return lastEvent;
}

// Alert timeout callback
static void alert_timeout_callback(void* arg) {
    // Alert has timed out
    ESP_LOGI(TAG, "Alert timeout reached, canceling alert");
    alertActive = false;

    // Turn off alarm and LED
    sos_playAlarm(false);
    sos_flashLed(false);
}

// Task to flash LED
static void flash_led_task(void* param) {
    while (ledFlashing) {
        // Toggle LED
        static bool led_state = false;
        led_state = !led_state;
        gpio_set_level(LED_PIN, led_state);

        // Flash pattern: on for 100ms, off for 100ms
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    // Ensure LED is off when task exits
    gpio_set_level(LED_PIN, 0);
    flashLedTaskHandle = NULL;
    vTaskDelete(NULL);
}

// Task to generate alarm sound
static void alarm_sound_task(void* param) {
    // SOS pattern in Morse code: ... --- ...
    // Short beep = 200ms, long beep = 600ms, pause between = 200ms

    const uint16_t duty = 4095; // 50% duty cycle on 13-bit resolution

    while (alarmActive) {
        // S: three short beeps
        for (int i = 0; i < 3; i++) {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL, duty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL);
            vTaskDelay(200 / portTICK_PERIOD_MS);

            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL, 0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL);
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }

        // Pause between letters
        vTaskDelay(400 / portTICK_PERIOD_MS);

        // O: three long beeps
        for (int i = 0; i < 3; i++) {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL, duty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL);
            vTaskDelay(600 / portTICK_PERIOD_MS);

            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL, 0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL);
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }

        // Pause between letters
        vTaskDelay(400 / portTICK_PERIOD_MS);

        // S: three short beeps
        for (int i = 0; i < 3; i++) {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL, duty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL);
            vTaskDelay(200 / portTICK_PERIOD_MS);

            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL, 0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL);
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }

        // Longer pause before repeating
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    // Ensure buzzer is off when task exits
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL);
    alarmSoundTaskHandle = NULL;
    vTaskDelete(NULL);
}

// Control the alarm sound
bool sos_playAlarm(bool enable) {
    if (enable) {
        if (alarmActive) {
            // Already active
            return true;
        }

        alarmActive = true;

        // Create task to play alarm sound
        BaseType_t result = xTaskCreate(
            alarm_sound_task,
            "alarm_sound_task",
            2048,
            NULL,
            5,
            &alarmSoundTaskHandle
        );

        if (result != pdPASS) {
            ESP_LOGE(TAG, "Failed to create alarm sound task");
            alarmActive = false;
            return false;
        }

        ESP_LOGI(TAG, "Alarm activated");
    } else {
        alarmActive = false;

        // Task will exit on its own once alarmActive is false
        ESP_LOGI(TAG, "Alarm deactivated");
    }

    return true;
}

// Control LED flashing
bool sos_flashLed(bool enable) {
    if (enable) {
        if (ledFlashing) {
            // Already active
            return true;
        }

        ledFlashing = true;

        // Create task to flash LED
        BaseType_t result = xTaskCreate(
            flash_led_task,
            "flash_led_task",
            1024,
            NULL,
            3,
            &flashLedTaskHandle
        );

        if (result != pdPASS) {
            ESP_LOGE(TAG, "Failed to create LED flash task");
            ledFlashing = false;
            return false;
        }

        ESP_LOGI(TAG, "LED flashing activated");
    } else {
        ledFlashing = false;

        // Task will exit on its own once ledFlashing is false
        ESP_LOGI(TAG, "LED flashing deactivated");
    }

    return true;
}
