#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"
#include "driver/ledc.h"

static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);

#define LOW_LIMIT 95   // 1ms
#define HIGH_LIMIT 506 // 2-ish
adc_cali_handle_t adc2_cali_chan9_handle = NULL;
int knob_position; // 0....2000

void knobMeasure(void *arg)
{

    // zero-initialize the config structure.
    gpio_config_t io_conf = {};
    // disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;

    // bit mask of the pins that you want to set,e.g.GPIO18/19
    // Creates a pin bit mask to the pins at IO2 and IO5
    // 0000000000000000000000000000000100
    io_conf.pin_bit_mask = (1ULL << GPIO_NUM_14) | (1ULL << GPIO_NUM_33);
    // disable pull-down mode
    io_conf.pull_down_en = 0;
    // disable pull-up mode
    io_conf.pull_up_en = 0;

    // configure GPIO with the given settings
    gpio_config(&io_conf);

    // KNOB PINS
    // SET 33 to HIGH
    gpio_set_level(GPIO_NUM_33, 1);
    // SET 14 to GND (LOW)
    gpio_set_level(GPIO_NUM_14, 0);

    // ADC2 INIT
    adc_oneshot_unit_handle_t adc2_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_2,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc2_handle));

    // CHANNEL 9 CONFIG
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, ADC_CHANNEL_9, &config));

    // CALLIBRATION HANDLE

    bool do_calibration1_chan9 = example_adc_calibration_init(ADC_UNIT_2, ADC_CHANNEL_9, ADC_ATTEN_DB_12, &adc2_cali_chan9_handle);

    while (1)
    {

        ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, ADC_CHANNEL_9, &knob_position));

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void motorControl(void *arg)
{

    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_12_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 50, // Set output frequency at 4 kHz
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = GPIO_NUM_21,
        .duty = 300, // Set duty to 0%
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    int duty_value = 0;

    while (1)
    {
        // range 0-4096
        duty_value = knob_position * (HIGH_LIMIT - LOW_LIMIT) / 4096 + LOW_LIMIT;
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty_value));
        // Update duty to apply the new value
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void ledBlink(void *arg)
{
    // zero-initialize the config structure.
    gpio_config_t io_conf = {};
    // disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;

    // bit mask of the pins that you want to set,e.g.GPIO18/19
    // Creates a pin bit mask to the pins at IO2 and IO5
    // 0000000000000000000000000000000100
    io_conf.pin_bit_mask = (1ULL << GPIO_NUM_2);
    // disable pull-down mode
    io_conf.pull_down_en = 0;
    // disable pull-up mode
    io_conf.pull_up_en = 0;

    // configure GPIO with the given settings
    gpio_config(&io_conf);

    int min_duration = 1000 / 30; // 15hz
    int max_duration = 1000 / 4;  // 2hz
    while (1)
    {
        // MEASURE KNOB
        // knob_position = measureKnob();
        int desired_delay = (knob_position * (max_duration - min_duration) / 4095 + min_duration);

        // LED OFF
        // Sets GPIO PIN 2 to HIGH(1)
        gpio_set_level(GPIO_NUM_2, 1);

        // DELAY
        vTaskDelay(desired_delay / portTICK_PERIOD_MS);
        // printf("Delay: %d \n", desired_delay);

        // LED ON
        // Sets GPIO PIN 2 to LOW (0)
        gpio_set_level(GPIO_NUM_2, 0);

        // Delay
        vTaskDelay(desired_delay / portTICK_PERIOD_MS);
        // printf("Delay: %d \n", desired_delay);
    }
}

void ledDim(void *arg)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_12_BIT,
        .timer_num = LEDC_TIMER_1,
        .freq_hz = 100, // Set output frequency at 4 kHz
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER_1,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = GPIO_NUM_15,
        .duty = 300, // Set duty to 0%
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    while (1)
    {
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, knob_position));
        // Update duty to apply the new value
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));

        // Delay
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void startupPrint()
{
    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if (esp_flash_get_size(NULL, &flash_size) != ESP_OK)
    {
        printf("Get flash size failed");
        return;
    }

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());
}

void app_main(void)
{

    startupPrint();

    TaskHandle_t knobtask = NULL;
    xTaskCreate(knobMeasure, "KNOB_TASK", 2048, NULL, 10, &knobtask);

    TaskHandle_t motortask = NULL;
    xTaskCreate(motorControl, "MOTOR_TASK", 2048, NULL, 20, &motortask);

    TaskHandle_t blinktask = NULL;
    xTaskCreate(ledBlink, "BLINK_TASK", 2048, NULL, 5, &blinktask);

    TaskHandle_t dimtask = NULL;
    xTaskCreate(ledDim, "DIM_TASK", 2048, NULL, 5, &dimtask);

    int voltage;

    while (1)
    {

        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc2_cali_chan9_handle, knob_position, &voltage));
        // printf("position = %d %d %1f\n", knob_position, voltage,((double))voltage/1000);

        printf("position = %d %1f\n", knob_position, ((double)voltage / 1000));

        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

// CALIBRATION LOGIC
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

    if (!calibrated)
    {
        ESP_LOGI("ADC", "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
        {
            calibrated = true;
        }
    }

    *out_handle = handle;
    if (ret == ESP_OK)
    {
        ESP_LOGI("ADC", "Calibration Success");
    }
    else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated)
    {
        ESP_LOGW("ADC", "eFuse not burnt, skip software calibration");
    }
    else
    {
        ESP_LOGE("ADC", "Invalid arg or no memory");
    }

    return calibrated;
}