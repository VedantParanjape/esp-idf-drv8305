#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "drv8305.h"

static const char TAG[] = "bldc";

void app_main()
{
    drv8305_t dev = {
        .DRV_EN_GATE_pin = 25,
        .DRV_N_FAULT_pin = 26,
        .DRV_MISO_SDO_pin = 12,
        .DRV_MOSI_SDI_pin = 13,
        .DRV_SCLK_pin = 14,
        .DRV_SCS_pin = 27,
        .spi_host = HSPI_HOST,
        .max_spi_clockspeed = 1000000
    };

    ESP_ERROR_CHECK(drv8305_init(&dev));

    uint16_t val;

    for (uint8_t i = 0x01; i <= 0xC; i++)
    {
        drv8305_read_register(&dev, i, &val);
        ESP_LOGI(TAG, "value %x: %x", i, val);
    }
}