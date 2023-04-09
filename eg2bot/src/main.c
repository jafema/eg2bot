#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "esp_log.h"
#include "sdkconfig.h"

// Read
// https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/mcpwm.html
// Example
// https://github.com/espressif/esp-idf/blob/master/examples/peripherals/mcpwm/mcpwm_servo_control/main/mcpwm_servo_control_example_main.c



static const char *TAG = "example";

#define BLINK_GPIO 4 //Config GPIO
#define PWM_GPIO 18

static uint8_t s_led_state = 0;

// Prototypes
static void blink_led(void);
static void configure_led(void);
static void configure_PWM(void);

// Functions
static void blink_led(void)
{
    gpio_set_level(BLINK_GPIO, s_led_state);
}

static void configure_led(void)
{
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as push/pull output*/
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

static void configure_PWM(void)
{
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PWM_GPIO);

    mcpwm_config_t pwm_config = {
        .frequency = 1, // frequecy in Hz
        .cmpr_a = 20, // duty cycle of PMWx! = 0
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0
    };

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
}

void app_main() {

    /* configure the peripheral according to the LED type */
    configure_led();
    configure_PWM();
    ESP_LOGI(TAG, "ESP demo PWM");

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 20);

    while(1)
    {
        blink_led();

        /* Toggle led state */
        s_led_state = !s_led_state;
        if (s_led_state)
        {
            ESP_LOGI(TAG, "ESP blink ON, status = %d", s_led_state);
        }
        else
        {
            ESP_LOGI(TAG, "ESP blink OFF, status = %d", s_led_state);
        }

        vTaskDelay(50 * portTICK_PERIOD_MS);

    }

}