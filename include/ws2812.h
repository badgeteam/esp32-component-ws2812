#pragma once

#include <stdint.h>
#include <esp_err.h>
#include <driver/gpio.h>

__BEGIN_DECLS

/**
 * Initialize the WS2812 driver.
 * @return ESP_OK on success; any other value indicates an error
 */
extern esp_err_t ws2812_init(gpio_num_t aGpioPin, int length);

/**
 * Disable the WS2812 driver
 * @return ESP_OK on success; any other value indicates an error
 */
extern esp_err_t ws2812_deinit(void);

/**
 * Send color-data to the WS2812 bus.
 * @param data the data-bytes to send on the bus.
 * @param len the data-length.
 * @return ESP_OK on success; any other value indicates an error
 */
extern esp_err_t ws2812_send_data(const uint8_t *data, int length);

__END_DECLS
