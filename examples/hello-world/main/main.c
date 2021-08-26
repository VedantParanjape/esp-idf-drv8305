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

    drv8305_status_01_reg_t temp;

    // temp->OTW = 1;
    // temp->TEMP_FLAG3 = 1;
    // temp->TEMP_FLAG2 = 1;
    // temp->TEMP_FLAG1 = 1;
    // temp->VCPH_UVFL = 1;
    // temp->VDS_STATUS = 1;
    // temp->PVDD_OVFL = 1;
    // temp->PVDD_UVFL = 1;
    // temp->TEMP_FLAG4 = 1;
    // temp->STATUS_01_RSV1 = 1;
    // temp->FAULT = 1;

    // val = *(uint16_t*)temp;

    // ESP_LOGI(TAG, "%x", val);

    // 00000 0000010000
    //      11    4   0
    //     bit   bit bit

    drv8305_read_status_01_register(&dev, &temp);
    ESP_LOGI(TAG, "%x", *(uint16_t*)&temp);

    ESP_LOGI(TAG, "%d", temp.OTW);
    ESP_LOGI(TAG, "%d", temp.TEMP_FLAG3);
    ESP_LOGI(TAG, "%d", temp.TEMP_FLAG2);
    ESP_LOGI(TAG, "%d", temp.TEMP_FLAG1);
    ESP_LOGI(TAG, "%d", temp.VCPH_UVFL);
    ESP_LOGI(TAG, "%d", temp.VDS_STATUS);
    ESP_LOGI(TAG, "%d", temp.PVDD_OVFL);
    ESP_LOGI(TAG, "%d", temp.PVDD_UVFL);
    ESP_LOGI(TAG, "%d", temp.TEMP_FLAG4);
    ESP_LOGI(TAG, "%d", temp.STAT01_RSV1);
    ESP_LOGI(TAG, "%d", temp.FAULT);

    drv8305_control_05_reg_t temp5_;
    temp5_.IDRIVEP_HS = 0xA;
    temp5_.IDRIVEN_HS = 0xB;
    temp5_.TDRIVEN = 0x2;

    drv8305_write_control_05_register(&dev, temp5_);
    drv8305_control_05_reg_t temp5;
    drv8305_read_control_05_register(&dev, &temp5);
    ESP_LOGI(TAG, "value %x: %x", 0x5, *(uint16_t*)&temp5);

    ESP_LOGI(TAG, "%x", temp5.IDRIVEP_HS);    
    ESP_LOGI(TAG, "%x", temp5.IDRIVEN_HS);    
    ESP_LOGI(TAG, "%x", temp5.TDRIVEN);    
    ESP_LOGI(TAG, "%x", temp5.STAT05_RSV1);
}