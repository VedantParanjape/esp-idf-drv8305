#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

#define enGate 25
#define nFault 26

#define MISO_PIN  12
#define MOSI_PIN  13
#define SCLK_PIN  14
#define CS_PIN    27
#define drv8305_MAX_SPI_FREQ 1000000
#define HOST HSPI_HOST

static const char TAG[] = "bldc";

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define BV(x) (1 << (x))

typedef struct
{
    spi_device_interface_config_t spi_cfg;
    spi_device_handle_t spi_dev;
} drv8305_t;

esp_err_t drv8305_init_desc_spi(drv8305_t *dev, spi_host_device_t host, uint32_t clock_speed_hz, gpio_num_t cs_pin)
{
    memset(&dev->spi_cfg, 0, sizeof(dev->spi_cfg));
    dev->spi_cfg.spics_io_num = cs_pin;
    dev->spi_cfg.clock_speed_hz = clock_speed_hz;
    dev->spi_cfg.mode = 1;
    dev->spi_cfg.queue_size = 1;

    return spi_bus_add_device(host, &dev->spi_cfg, &dev->spi_dev);
}

#define BIT_ORDER 0

#if BIT_ORDER
    #define TX_DEF() uint8_t tx[] = { (out & 0xFF), (out >> 8 & 0xFF)}
#else
    #define TX_DEF() uint8_t tx[] = { (out >> 8 & 0xFF), (out & 0xFF) }
#endif

static esp_err_t write_reg(drv8305_t *dev, uint8_t reg, uint8_t val)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(spi_transaction_t));

    uint16_t out = 0;
    out |= (reg & 0x0F) << 11;
    out |= (val & 0x7FF);

    TX_DEF();
    uint8_t rx[sizeof(tx)];

    t.tx_buffer = tx;
    t.rx_buffer = rx;
    t.length = 16;

    esp_err_t err = spi_device_transmit(dev->spi_dev, &t);
    ESP_LOGI(TAG, "spi_debug: 0x%x 0x%x", (tx[0] << 8 ) | tx[1], (rx[0] << 8 ) | rx[1]);

    return err;
}   

static esp_err_t read_reg(drv8305_t *dev, uint8_t reg, uint16_t *val)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(spi_transaction_t));

    uint16_t out = 0;
    out |= (1 << 15);
    out |= (reg & 0x0F) << 11;
    out |= 0x807F;

    TX_DEF();

    uint8_t rx[sizeof(tx)];

    t.tx_buffer = tx;
    t.rx_buffer = rx;
    t.length = 16;
    
    CHECK(spi_device_transmit(dev->spi_dev, &t));

    *val = (rx[0] << 8 ) | rx[1];

    return ESP_OK;
}

void app_main()
{
    drv8305_t dev;
    
    // Configure SPI bus
    spi_bus_config_t cfg = {
       .mosi_io_num = MOSI_PIN,
       .miso_io_num = MISO_PIN,
       .sclk_io_num = SCLK_PIN,
       .quadwp_io_num = -1,
       .quadhd_io_num = -1,
       .max_transfer_sz = 0,
       .flags = 0
    };

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    gpio_set_direction(nFault, GPIO_MODE_INPUT);
    gpio_set_direction(enGate, GPIO_MODE_OUTPUT);
    gpio_set_direction(CS_PIN, GPIO_MODE_OUTPUT);
    ESP_LOGI(TAG, "enGate Enabled");
    gpio_set_level(enGate, 1);
    gpio_set_level(CS_PIN, 1);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    ESP_ERROR_CHECK(spi_bus_initialize(HOST, &cfg, 1));
    ESP_ERROR_CHECK(drv8305_init_desc_spi(&dev, HOST, drv8305_MAX_SPI_FREQ, CS_PIN));

    vTaskDelay(250 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "DRV8305 INIT");
  
    // init drv8305
    uint16_t val;

    for (uint8_t i = 0x01; i <= 0xC; i++)
    {
        read_reg(&dev, i, &val);
        ESP_LOGI(TAG, "value %x: %x", i, val);
    }
}