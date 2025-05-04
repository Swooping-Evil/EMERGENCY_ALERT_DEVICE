/**
 * Emergency Alert Device - Main Program
 *
 * This is the entry point for the wearable emergency alert device designed
 * to assist women in distress. It detects critical events (falls or
 * panic button press) and sends emergency alerts via GSM.
 *
 * Power management is implemented through ESP32's deep sleep functionality,
 * with wake-up triggered by motion detection or button press.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "nvs_flash.h"

// Include module headers
#include "gps_module.h"
#include "gsm_module.h"
#include "sos_module.h"
#include "motion_detection.h"
#include "mpu.h"
#include "sensor_fusion.h"

// GPIO pin definitions
#define PANIC_BUTTON_PIN     GPIO_NUM_14
#define MOTION_INTERRUPT_PIN GPIO_NUM_15
#define STATUS_LED_PIN       GPIO_NUM_2
#define BUZZER_PIN           GPIO_NUM_4

// System states
typedef enum {
    STATE_NORMAL,
    STATE_FALL_DETECTED,
    STATE_SOS_TRIGGERED,
    STATE_SENDING_ALERT,
    STATE_ERROR
} system_state_t;

// Global variables
static system_state_t current_state = STATE_NORMAL;
static bool fall_detected = false;
static bool sos_triggered = false;
static bool motion_interrupt = false;

// Function prototypes
void init_hardware(void);
void process_system_state(void);
void enter_deep_sleep(void);
static void IRAM_ATTR panic_button_isr(void* arg);
static void IRAM_ATTR motion_detect_isr(void* arg);
void send_emergency_alert(void);
void handle_error(const char* error_message);

/**
 * Main application entry point
 */
void app_main(void)
{
    // Initialize NVS (Non-Volatile Storage)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    printf("Emergency Alert Device Initializing...\n");

    // Initialize all hardware components
    init_hardware();

    printf("All systems initialized successfully\n");

    // Main operation loop
    while (1) {
        // Process the current system state
        process_system_state();

        // If in normal state and no events are detected, go to sleep to save power
        if (current_state == STATE_NORMAL && !fall_detected && !sos_triggered) {
            enter_deep_sleep();
        }

        // delay to prevent CPU hogging
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

/**
 * Initialize all hardware components
 */
void init_hardware(void)
{
    // Initialize GPIO for panic button with interrupt
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE; // Interrupt on falling edge
    io_conf.pin_bit_mask = (1ULL << PANIC_BUTTON_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    // Initialize GPIO for motion interrupt
    io_conf.intr_type = GPIO_INTR_POSEDGE; // Interrupt on rising edge
    io_conf.pin_bit_mask = (1ULL << MOTION_INTERRUPT_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpio_config(&io_conf);

    // Initialize output pins
    gpio_set_direction(STATUS_LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(BUZZER_PIN, GPIO_MODE_OUTPUT);

    // Initialize interrupt service and handlers
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PANIC_BUTTON_PIN, panic_button_isr, NULL);
    gpio_isr_handler_add(MOTION_INTERRUPT_PIN, motion_detect_isr, NULL);

    // Initialize MPU (accelerometer/gyroscope)
    if (mpu_init() != ESP_OK) {
        handle_error("Failed to initialize MPU");
    }

    // Initialize motion detection algorithm
    if (motion_detection_init() != ESP_OK) {
        handle_error("Failed to initialize motion detection");
    }

    // Initialize sensor fusion (Kalman filter)
    if (sensor_fusion_init() != ESP_OK) {
        handle_error("Failed to initialize sensor fusion");
    }

    // Initialize GPS module
    if (gps_init() != ESP_OK) {
        handle_error("Failed to initialize GPS module");
    }

    // Initialize GSM module
    if (gsm_init() != ESP_OK) {
        handle_error("Failed to initialize GSM module");
    }

    // Initialize SOS module
    if (sos_init(PANIC_BUTTON_PIN) != ESP_OK) {
        handle_error("Failed to initialize SOS module");
    }

    // Blink LED to indicate successful initialization
    for (int i = 0; i < 3; i++) {
        gpio_set_level(STATUS_LED_PIN, 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        gpio_set_level(STATUS_LED_PIN, 0);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

/**
 * Process the current system state
 */
void process_system_state(void)
{
    switch (current_state) {
        case STATE_NORMAL:
            // Check if a fall was detected
            if (fall_detected) {
                printf("Fall detected! Transitioning to FALL_DETECTED state\n");
                current_state = STATE_FALL_DETECTED;
                // Turn on status LED
                gpio_set_level(STATUS_LED_PIN, 1);
            }

            // Check if SOS button was pressed
            if (sos_triggered) {
                printf("SOS button pressed! Transitioning to SOS_TRIGGERED state\n");
                current_state = STATE_SOS_TRIGGERED;
                // Turn on status LED
                gpio_set_level(STATUS_LED_PIN, 1);
            }
            break;

        case STATE_FALL_DETECTED:
            // Wait for a short period to see if the user cancels the alert
            // (could be implemented with a timer and button press to cancel)
            printf("Fall detected! Sending alert in 5 seconds unless canceled...\n");
            vTaskDelay(5000 / portTICK_PERIOD_MS); // 5 seconds delay

            // If not canceled, proceed to send alert
            current_state = STATE_SENDING_ALERT;
            break;

        case STATE_SOS_TRIGGERED:
            // Immediately transition to sending alert
            printf("SOS triggered! Sending emergency alert immediately\n");

            // Check if it's a double press (to activate loud alarm)
            if (sos_is_double_press()) {
                printf("Double press detected! Activating loud alarm\n");
                gpio_set_level(BUZZER_PIN, 1);
            }

            current_state = STATE_SENDING_ALERT;
            break;

        case STATE_SENDING_ALERT:
            // Send the emergency alert
            send_emergency_alert();

            // Reset the trigger flags
            fall_detected = false;
            sos_triggered = false;

            // Turn off status LED
            gpio_set_level(STATUS_LED_PIN, 0);

            // Return to normal state
            current_state = STATE_NORMAL;
            break;

        case STATE_ERROR:
            // In error state, blink LED rapidly to indicate problem
            gpio_set_level(STATUS_LED_PIN, 1);
            vTaskDelay(200 / portTICK_PERIOD_MS);
            gpio_set_level(STATUS_LED_PIN, 0);
            vTaskDelay(200 / portTICK_PERIOD_MS);

            // Try to recover by returning to normal state
            current_state = STATE_NORMAL;
            break;

        default:
            // Invalid state, reset to normal
            current_state = STATE_NORMAL;
            break;
    }
}

/**
 * Enter deep sleep mode to save power
 */
void enter_deep_sleep(void)
{
    printf("Entering deep sleep mode...\n");

    // Configure wake-up sources
    esp_sleep_enable_ext0_wakeup(PANIC_BUTTON_PIN, 0); // Wake on panic button press (low level)
    esp_sleep_enable_ext1_wakeup(1ULL << MOTION_INTERRUPT_PIN, ESP_EXT1_WAKEUP_ANY_HIGH); // Wake on motion

    // Enter deep sleep
    esp_deep_sleep_start();

    // Execution will continue from app_main() after wake-up
}

/**
 * ISR for panic button
 */
static void IRAM_ATTR panic_button_isr(void* arg)
{
    sos_triggered = true;
}

/**
 * ISR for motion detection
 */
static void IRAM_ATTR motion_detect_isr(void* arg)
{
    motion_interrupt = true;

    // The actual fall detection will be done in the main task
    // This just signals that motion has been detected
}

/**
 * Send emergency alert with GPS location
 */
void send_emergency_alert(void)
{
    // Get current GPS coordinates
    float latitude, longitude;
    if (gps_get_location(&latitude, &longitude) != ESP_OK) {
        handle_error("Failed to get GPS location");
        // Proceed anyway with last known coordinates or default values
    }

    // Prepare and send alert via GSM
    if (gsm_send_emergency_alert(latitude, longitude) != ESP_OK) {
        handle_error("Failed to send emergency alert via GSM");
    } else {
        printf("Emergency alert sent successfully with coordinates: %f, %f\n", latitude, longitude);
    }
}

/**
 * Handle errors
 */
void handle_error(const char* error_message)
{
    printf("ERROR: %s\n", error_message);
    current_state = STATE_ERROR;
}
