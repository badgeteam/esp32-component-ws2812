//This driver uses the ESP32 RMT peripheral to drive "Neopixel" compatible LEDs
//The usage of the RMT peripheral has been implemented using work by JSchaenzie:
//you can find his work at https://github.com/JSchaenzle/ESP32-NeoPixel-WS2812-RMT

#include <sdkconfig.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <esp_log.h>

#include <driver/gpio.h>
#include <driver/rmt.h>

#include "ws2812.h"

static const char *TAG = "ws2812";

#define WS2812_RMT_CHANNEL RMT_CHANNEL_0

#define WS2812_T0H_NS (350)
#define WS2812_T0L_NS (1000)
#define WS2812_T1H_NS (1000)
#define WS2812_T1L_NS (350)

static uint32_t ws2812_t0h_ticks = 0;
static uint32_t ws2812_t1h_ticks = 0;
static uint32_t ws2812_t0l_ticks = 0;
static uint32_t ws2812_t1l_ticks = 0;

static bool   gActive       = false;
rmt_item32_t* gBuffer       = NULL;
int           gLength       = 0;
gpio_num_t    gPin;

esp_err_t ws2812_init(gpio_num_t aGpioPin, int length) {
    if (gActive) {
        esp_err_t res = ws2812_deinit();
        if (res != ESP_OK) return res;
    }

    gLength = length;
    gBuffer = calloc(gLength * 8, sizeof(rmt_item32_t));
    if (gBuffer == NULL) return ESP_FAIL;
    
    gPin = aGpioPin;
    gpio_config_t io_conf = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1LL << aGpioPin,
        .pull_down_en = 0,
        .pull_up_en   = 0,
    };
    esp_err_t res = gpio_config(&io_conf);
    if (res != ESP_OK) return res;

    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(aGpioPin, WS2812_RMT_CHANNEL);
    config.clk_div = 2;
    res = rmt_config(&config);
    if (res != ESP_OK) return res;
    res = rmt_driver_install(config.channel, 0, 0);
    if (res != ESP_OK) return res;
    gActive = true;
    
    uint32_t counter_clk_hz = 0;
    res = rmt_get_counter_clock(WS2812_RMT_CHANNEL, &counter_clk_hz);
    if (res != ESP_OK) return res;

    float ratio = (float)counter_clk_hz / 1e9;
    ws2812_t0h_ticks = (uint32_t)(ratio * WS2812_T0H_NS);
    ws2812_t0l_ticks = (uint32_t)(ratio * WS2812_T0L_NS);
    ws2812_t1h_ticks = (uint32_t)(ratio * WS2812_T1H_NS);
    ws2812_t1l_ticks = (uint32_t)(ratio * WS2812_T1L_NS);

    ESP_LOGW(TAG, "WS2812 driver init complete");
    return ESP_OK;
}

esp_err_t ws2812_deinit(void) {
    if (!gActive) return ESP_OK;
    free(gBuffer);
    gBuffer = NULL;
    gLength = 0;
    esp_err_t res = rmt_driver_uninstall(WS2812_RMT_CHANNEL);
    if (res != ESP_OK) return res;
    gpio_config_t io_conf = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_INPUT,
        .pin_bit_mask = 1LL << gPin,
        .pull_down_en = 0,
        .pull_up_en   = 0,
    };
    res = gpio_config(&io_conf);
    if (res != ESP_OK) return res;
    gActive = false;
    return ESP_OK;
}

esp_err_t ws2812_prepare_data(const uint8_t *data, int length) {
    const rmt_item32_t bit0 = {{{ ws2812_t0h_ticks, 1, ws2812_t0l_ticks, 0 }}};
    const rmt_item32_t bit1 = {{{ ws2812_t1h_ticks, 1, ws2812_t1l_ticks, 0 }}};
    if (gBuffer == NULL) return ESP_FAIL;  // No buffer
    if (length > gLength) return ESP_FAIL; // Too much data
    for (uint32_t position = 0; position < length; position++) {
        uint32_t mask = 1 << 7;
        for (uint8_t i = 0; i < 8; i++) {
            bool bit = data[position] & mask;
            gBuffer[position*8 + i] = bit ? bit1 : bit0;
            mask >>= 1;
        }
    }
    return ESP_OK;
}

esp_err_t ws2812_send_data(const uint8_t *data, int length) {
    if (!gActive) {
        ESP_LOGE(TAG, "Not active");
        return ESP_FAIL;
    }
    esp_err_t res = ws2812_prepare_data(data, length);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to prepare data");
        return res;
    }
    res = rmt_write_items(WS2812_RMT_CHANNEL, gBuffer, length * 8, false);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write data");
        return res;
    }
    res = rmt_wait_tx_done(WS2812_RMT_CHANNEL, portMAX_DELAY);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wait for TX done");
        return res;
    }
    return res;
}
