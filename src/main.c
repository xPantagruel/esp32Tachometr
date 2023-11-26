#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "ssd1306.h"
#include <string.h>
#include "driver/gpio.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "esp_timer.h"

#define tag "SSD1306"
#define INPUT_PIN 16
#define SECOND_INPUT_PIN 26

#define DEBOUNCE_DELAY_MS 200 // TODO may need to change this value to not miss interrupts (set to less)
#define WHEEL_DIAMETER_CM 120
#define TIMER_INTERVAL_MS 2500 // 2.5 second interval for speed calculation
typedef struct {
    uint8_t pin;
    bool inverted;
    uint16_t history;
    TickType_t last_interrupt_time;
    TickType_t last_button_press_time;
    uint64_t start_time_ms; // To track start time in milliseconds
} debounce_t;

enum DisplayState {
    DISPLAY_SPEED,
    DISPLAY_KM_TRAVELED,
    DISPLAY_AVG_SPEED
};

int state = 0;
float speed_kmph = 0.0; // Track the speed in km/h
float km_traveled = 0.0; // Track the distance traveled

uint64_t start_time_us = 0;
QueueHandle_t interruptQueue1; // Queue for button 1
QueueHandle_t interruptQueue2; // Queue for button 2

enum DisplayState current_display_state = DISPLAY_SPEED; // Initialize to display speed initially

TimerHandle_t speedTimer;

static void IRAM_ATTR gpio_interrupt_handler1(void *args) {
    int pinNumber = (int)args;
    xQueueSendFromISR(interruptQueue1, &pinNumber, NULL);
}
static void IRAM_ATTR gpio_interrupt_handler2(void *args) {
    int pinNumber = (int)args;
    xQueueSendFromISR(interruptQueue2, &pinNumber, NULL);
}
void Wheel_Revolution_Task(void *params) {
    int pinNumber;
    debounce_t *debounce = (debounce_t *)params;
    while (true) {
        if (xQueueReceive(interruptQueue1, &pinNumber, portMAX_DELAY)) {
            float current_time_ms = esp_timer_get_time() / 1000; // Convert to milliseconds

            printf("Time (ms): %f\n", current_time_ms);

            if ((current_time_ms - debounce->last_button_press_time) > DEBOUNCE_DELAY_MS) {
                debounce->last_button_press_time = current_time_ms;
                printf("1. Button Pressed\n");

                if (debounce->last_button_press_time > 0) {
                    float time_hours = (float)(debounce->last_button_press_time - debounce->last_interrupt_time) / (1000.0 * 3600.0);
                    printf("Time (hours): %f\n", time_hours);
                    float distance_km = WHEEL_DIAMETER_CM / 100000.0; // Convert to kilometers

                    // Calculate speed in km/h
                    speed_kmph = distance_km / time_hours;
                    printf("Current Speed: %.2f km/h\n", speed_kmph);

                    km_traveled += distance_km;
                    printf("Km Traveled: %.2f km\n", km_traveled);
                }

                debounce->last_interrupt_time = current_time_ms;
            }
        }
    }
}

void Second_Button_Handle(void *params) {
    int pinNumber;
    debounce_t *debounce = (debounce_t *)params;
    while (true) {
        if (xQueueReceive(interruptQueue2, &pinNumber, portMAX_DELAY)) {
            float current_time_ms = esp_timer_get_time() / 1000; // Convert to milliseconds

            if ((current_time_ms - debounce->last_button_press_time) > DEBOUNCE_DELAY_MS) {
                debounce->last_button_press_time = current_time_ms;
                printf("2. Button Pressed\n");

                // Toggle the display state when button 2 is pressed
                current_display_state = (current_display_state + 1) % 3; // Cycle through display states


                debounce->last_interrupt_time = current_time_ms;
            }
        }
    }
}

void calculateSpeed(TimerHandle_t xTimer) {
    debounce_t *debounce = (debounce_t *)pvTimerGetTimerID(xTimer);

    float current_time_ms = esp_timer_get_time() / 1000; // Convert to milliseconds

    // Check if more than 1 second has passed since the last interrupt
    if ((current_time_ms - debounce->last_interrupt_time) >= TIMER_INTERVAL_MS) {
        float time_diff = current_time_ms - debounce->last_interrupt_time;
        if (time_diff > 0) { // Check for non-zero time difference to avoid division by zero
            float time_hours = time_diff / (1000.0 * 3600.0);
            float distance_km = WHEEL_DIAMETER_CM / 100000.0; // Convert to kilometers
            // Calculate speed in km/h
            speed_kmph = distance_km / time_hours;

            printf("Calculated Speed: %.2f km/h\n", speed_kmph);
        } else {
            printf("No valid time difference to calculate speed.\n");
        }
    }
}

void app_main(void)
{
    // ============================================= Display =============================================

    SSD1306_t dev;

    ESP_LOGI(tag, "INTERFACE is SPI");
    ESP_LOGI(tag, "CONFIG_CS_GPIO=%d", CONFIG_CS_GPIO);
    ESP_LOGI(tag, "CONFIG_DC_GPIO=%d", CONFIG_DC_GPIO);
    ESP_LOGI(tag, "CONFIG_RESET_GPIO=%d", CONFIG_RESET_GPIO);

    spi_master_init(&dev, CONFIG_MOSI_GPIO, CONFIG_SCLK_GPIO, CONFIG_CS_GPIO, CONFIG_DC_GPIO, CONFIG_RESET_GPIO);

    ESP_LOGI(tag, "Panel is 128x64");
    ssd1306_init(&dev, 128, 64);
    ssd1306_clear_screen(&dev, false);

    // ============================================= button 1 =============================================

    // Setup interrupt queue
    interruptQueue1 = xQueueCreate(10, sizeof(int));
    interruptQueue2 = xQueueCreate(10, sizeof(int));

    if (interruptQueue1 == NULL || interruptQueue2 == NULL) {
        return;
    }

    // Initialize debounce structure for button 1
    debounce_t debounce;
    debounce.pin = INPUT_PIN;
    debounce.inverted = false;
    debounce.history = 0;
    debounce.last_interrupt_time = 0;
    debounce.last_button_press_time = 0;
    debounce.start_time_ms = 0;

    esp_rom_gpio_pad_select_gpio(INPUT_PIN);
    gpio_set_direction(INPUT_PIN, GPIO_MODE_INPUT);
    gpio_pullup_en(INPUT_PIN);  // Enable internal pull-up resistor
    gpio_pulldown_dis(INPUT_PIN);  // Disable internal pull-down resistor
    gpio_set_intr_type(INPUT_PIN, GPIO_INTR_ANYEDGE); // Change interrupt type to GPIO_INTR_ANYEDGE for detecting both rising and falling edges

    gpio_install_isr_service(0);
    gpio_isr_handler_add(INPUT_PIN, gpio_interrupt_handler1, (void *)INPUT_PIN);
    xTaskCreate(Wheel_Revolution_Task, "Wheel_Revolution_Task", 2048, (void *)&debounce, 1, NULL);

    // ============================================= button 2 =============================================
    // Initialize debounce structure for button 2
    debounce_t debounce2;
    debounce2.pin = SECOND_INPUT_PIN;
    debounce2.inverted = false;
    debounce2.history = 0;
    debounce2.last_interrupt_time = 0;
    debounce2.last_button_press_time = 0;
    debounce2.start_time_ms = 0;

    esp_rom_gpio_pad_select_gpio(SECOND_INPUT_PIN);
    gpio_set_direction(SECOND_INPUT_PIN, GPIO_MODE_INPUT);
    gpio_pullup_en(SECOND_INPUT_PIN);  // Enable internal pull-up resistor
    gpio_pulldown_dis(SECOND_INPUT_PIN);  // Disable internal pull-down resistor
    gpio_set_intr_type(SECOND_INPUT_PIN, GPIO_INTR_ANYEDGE); // Change interrupt type to GPIO_INTR_ANYEDGE for detecting both rising and falling edges

    gpio_isr_handler_add(SECOND_INPUT_PIN, gpio_interrupt_handler2, (void *)SECOND_INPUT_PIN);
    xTaskCreate(Second_Button_Handle, "Second_Button_Handle", 2048, (void *)&debounce2, 1, NULL);

    // Initialize start time in microseconds
    debounce.start_time_ms = esp_timer_get_time() / 1000;

    // Create the software timer
    speedTimer = xTimerCreate("SpeedTimer", pdMS_TO_TICKS(TIMER_INTERVAL_MS), pdTRUE, (void *)&debounce, calculateSpeed);

    if (speedTimer != NULL) {
        // Start the software timer to calculate speed every second
        xTimerStart(speedTimer, 0);
    } else {
        printf("Failed to create speed timer.\n");
    }

    float total_time_hours = 0.0;
    float average_speed_kmph = 0.0;

    enum DisplayState previous_display_state = current_display_state; // Initialize to display speed initially

    while (1)
    {
        // Check if the display state has changed
        if( previous_display_state != current_display_state ) {
            // Clear the screen
            ssd1306_clear_screen(&dev, false);
            previous_display_state = current_display_state;
        }

        // Display content based on the current state
        switch (current_display_state) {
            case DISPLAY_SPEED:
                char speed_str[20];
                snprintf(speed_str, sizeof(speed_str), "%.2f", speed_kmph);
                ssd1306_display_text_x3(&dev, 3, speed_str, strlen(speed_str), false);

                char speed_str0[20];
                snprintf(speed_str0, sizeof(speed_str0), "KM/hr");
                ssd1306_display_text(&dev, 7, speed_str0, strlen(speed_str0), false);
                break;

            case DISPLAY_KM_TRAVELED:
                char distance_str[20];
                snprintf(distance_str, sizeof(distance_str), "%.3f", km_traveled);
                ssd1306_display_text_x3(&dev, 3, distance_str, strlen(distance_str), false);

                char distance_str0[20];
                snprintf(distance_str0, sizeof(distance_str0), "KM");
                ssd1306_display_text(&dev, 7, distance_str0, strlen(distance_str0), false);

                break;

            case DISPLAY_AVG_SPEED:
                if (km_traveled > 0) {
                    total_time_hours = (float)((float)(esp_timer_get_time()/1000) - debounce.start_time_ms) / (1000.0 * 3600.0);
                    average_speed_kmph = km_traveled / total_time_hours;

                    char average_speed_str[20];
                    snprintf(average_speed_str, sizeof(average_speed_str), "%.2f", average_speed_kmph);
                    ssd1306_display_text_x3(&dev, 3, average_speed_str, strlen(average_speed_str), false);

                    char average_speed_str0[20];
                    snprintf(average_speed_str0, sizeof(average_speed_str0), "AVERAGE KM/hr");
                    ssd1306_display_text(&dev, 7, average_speed_str0, strlen(average_speed_str0), false);
                }
                break;

            default:
                break;
    }
        vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
