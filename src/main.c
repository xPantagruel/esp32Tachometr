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
#define LED_PIN 2
#define DEBOUNCE_DELAY_MS 200
#define WHEEL_DIAMETER_CM 62
#define TIMER_INTERVAL_MS 2500 // 2.5 second interval for speed calculation

int state = 0;
uint64_t start_time_us = 0;
QueueHandle_t interruptQueue;
// Add a new FreeRTOS software timer handle
TimerHandle_t speedTimer;

typedef struct {
    uint8_t pin;
    bool inverted;
    uint16_t history;
    TickType_t last_interrupt_time;
    TickType_t last_button_press_time;
    float speed; // Calculated speed
    float km_traveled; // Track the distance traveled
    uint64_t start_time_ms; // To track start time in milliseconds

} debounce_t;

static void IRAM_ATTR gpio_interrupt_handler(void *args) {
    int pinNumber = (int)args;
    xQueueSendFromISR(interruptQueue, &pinNumber, NULL);
}

void Wheel_Revolution_Task(void *params) {
    int pinNumber;
    debounce_t *debounce = (debounce_t *)params;
    while (true) {
        if (xQueueReceive(interruptQueue, &pinNumber, portMAX_DELAY)) {
            float current_time_ms = esp_timer_get_time() / 1000; // Convert to milliseconds

            printf("Time (ms): %f\n", current_time_ms);

            if ((current_time_ms - debounce->last_button_press_time) > DEBOUNCE_DELAY_MS) {
                debounce->last_button_press_time = current_time_ms;
                printf("Button Pressed\n");

                if (debounce->last_button_press_time > 0) {
                    float time_hours = (float)(debounce->last_button_press_time - debounce->last_interrupt_time) / (1000.0 * 3600.0);
                    printf("Time (hours): %f\n", time_hours);
                    float distance_km = WHEEL_DIAMETER_CM / 100000.0; // Convert to kilometers
                    // Calculate speed in km/h
                    float speed_kmph = distance_km / time_hours;

                    // Update debounce's speed
                    debounce->speed = speed_kmph;
                    printf("Current Speed: %.2f km/h\n", debounce->speed);

                    debounce->km_traveled += distance_km;
                    printf("Km Traveled: %.2f km\n", debounce->km_traveled);
                }

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
            float speed_kmph = distance_km / time_hours;

            // Update debounce's speed
            debounce->speed = speed_kmph;

            printf("Calculated Speed: %.2f km/h\n", debounce->speed);
        } else {
            printf("No valid time difference to calculate speed.\n");
        }
    }
}
void LED_Control_Task(void *params) {
    int pinNumber, count = 0;
    while (true) {
        if (xQueueReceive(interruptQueue, &pinNumber, portMAX_DELAY)) {
            debounce_t *debounce = (debounce_t *)params;

            TickType_t current_time = xTaskGetTickCount();
            if ((current_time - debounce->last_interrupt_time) > pdMS_TO_TICKS(DEBOUNCE_DELAY_MS)) {
                printf("GPIO %d was pressed %d times. The state is %d\n", pinNumber, count++, gpio_get_level(debounce->pin));
                gpio_set_level(LED_PIN, gpio_get_level(debounce->pin));
            }
            debounce->last_interrupt_time = current_time;
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

    // ============================================= button =============================================

    // Setup interrupt queue
    interruptQueue = xQueueCreate(10, sizeof(int));
    if (interruptQueue == NULL) {
        return;
    }

    // Initialize debounce structure
    debounce_t debounce;
    debounce.pin = INPUT_PIN;
    debounce.inverted = false;
    debounce.history = 0;
    debounce.last_interrupt_time = 0;
    debounce.last_button_press_time = 0;
    debounce.speed = 0;
    debounce.km_traveled = 0.0;
    debounce.start_time_ms = 0;

    esp_rom_gpio_pad_select_gpio(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    esp_rom_gpio_pad_select_gpio(INPUT_PIN);
    gpio_set_direction(INPUT_PIN, GPIO_MODE_INPUT);
    gpio_pullup_en(INPUT_PIN);  // Enable internal pull-up resistor
    gpio_pulldown_dis(INPUT_PIN);  // Disable internal pull-down resistor
    gpio_set_intr_type(INPUT_PIN, GPIO_INTR_ANYEDGE); // Change interrupt type to GPIO_INTR_ANYEDGE for detecting both rising and falling edges

    gpio_install_isr_service(0);
    gpio_isr_handler_add(INPUT_PIN, gpio_interrupt_handler, (void *)INPUT_PIN);
    xTaskCreate(Wheel_Revolution_Task, "Wheel_Revolution_Task", 2048, (void *)&debounce, 1, NULL);

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
    while (1)
    {
        // Clear the screen
        ssd1306_clear_screen(&dev, false);

        // Display speed in larger font at the center
        char speed_str[20];
        snprintf(speed_str, sizeof(speed_str), "Speed: %.2f", debounce.speed);
        ssd1306_display_text(&dev, 3, speed_str, strlen(speed_str), true);

        // Display distance below the speed text
        char distance_str[20];
        snprintf(distance_str, sizeof(distance_str), "Km : %.3f", debounce.km_traveled);
        ssd1306_display_text(&dev, 5, distance_str, strlen(distance_str), false);

        // Display average speed below the distance text
        if (debounce.km_traveled > 0) {
            total_time_hours = (float)((float)(esp_timer_get_time()/1000) - debounce.start_time_ms) / (1000.0 * 3600.0);
            average_speed_kmph = debounce.km_traveled / total_time_hours;

            char average_speed_str[20];
            snprintf(average_speed_str, sizeof(average_speed_str), "Avg: %.2f", average_speed_kmph);
            ssd1306_display_text(&dev, 7, average_speed_str, strlen(average_speed_str), false);
            printf("Average Speed: %.4f km/h\n", average_speed_kmph);
        }


        vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
