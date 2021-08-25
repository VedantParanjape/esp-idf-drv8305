#ifndef DRV8305_H
#define DRV8305_H

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

// struct which holds data for SPI driver
typedef struct drv8305_t_ {
    spi_device_interface_config_t spi_cfg;
    spi_device_handle_t spi_dev;
    gpio_num_t DRV_EN_GATE_pin;
    gpio_num_t DRV_N_FAULT_pin;
    gpio_num_t DRV_MISO_SDO_pin;
    gpio_num_t DRV_MOSI_SDI_pin;
    gpio_num_t DRV_SCLK_pin;
    gpio_num_t DRV_SCS_pin;
    spi_host_device_t spi_host;
    uint32_t max_spi_clockspeed;
} drv8305_t;

// Structs defining the registers on drv8305
typedef struct drv8305_status_01_reg_t_
{
    uint8_t OTW            :1 ; // Bits 0
    uint8_t TEMP_FLAG3     :1 ; // Bits 1
    uint8_t TEMP_FLAG2     :1 ; // Bits 2
    uint8_t TEMP_FLAG1     :1 ; // Bits 3
    uint8_t VCPH_UVFL      :1 ; // Bits 4
    uint8_t VDS_STATUS     :1 ; // Bits 5
    uint8_t PVDD_OVFL      :1 ; // Bits 6
    uint8_t PVDD_UVFL      :1 ; // Bits 7
    uint8_t TEMP_FLAG4     :1 ; // Bits 8
    uint8_t STATUS_01_RSV1 :1 ; // Bits 9
    uint8_t FAULT          :1 ; // Bits 10
} __attribute__((packed, aligned(2))) drv8305_status_01_reg_t;

typedef struct drv8305_status_02_reg_t_
{
    uint8_t SNS_A_OCP      :1 ; // Bits 0,
    uint8_t SNS_B_OCP      :1 ; // Bits 1
    uint8_t SNS_C_OCP      :1 ; // Bits 2
    uint8_t STATUS_02_RSV1 :2 ; // Bits 3:4
    uint8_t FETLC_VDS      :1 ; // Bits 5
    uint8_t FETHC_VDS      :1 ; // Bits 6
    uint8_t FETLB_VDS      :1 ; // Bits 7
    uint8_t FETHB_VDS      :1 ; // Bits 8
    uint8_t FETLA_VDS      :1 ; // Bits 9
    uint8_t FETHA_VDS      :1 ; // Bits 10
} __attribute__((packed, aligned(2))) drv8305_status_02_reg_t;

typedef struct drv8305_status_03_reg_t_
{
    uint8_t VCPH_OVLO_ABS  :1 ; // Bits 0
    uint8_t VCPH_OVLO      :1 ; // Bits 1
    uint8_t VCPH_UVLO2     :1 ; // Bits 2
    uint8_t STATUS_03_RSV1 :1 ; // Bits 3
    uint8_t VCP_LSD_UVLO2  :1 ; // Bits 4
    uint8_t AVDD_UVLO      :1 ; // Bits 5
    uint8_t VREG_UV        :1 ; // Bits 6
    uint8_t STATUS_03_RSV2 :1 ; // Bits 7
    uint8_t OTSD           :1 ; // Bits 8
    uint8_t WD_FAULT       :1 ; // Bits 9
    uint8_t PVDD_UVLO2     :1 ; // Bits 10
} __attribute__((packed, aligned(2))) drv8305_status_03_reg_t;

typedef struct drv8305_status_04_reg_t_
{
    uint8_t STAT04_RSV1    :5 ; // Bits 0:4
    uint8_t FETLC_VGS      :1 ; // Bits 5
    uint8_t FETHC_VGS      :1 ; // Bits 6
    uint8_t FETLB_VGS      :1 ; // Bits 7
    uint8_t FETHB_VGS      :1 ; // Bits 8
    uint8_t FETLA_VGS      :1 ; // Bits 9
    uint8_t FETHA_VGS      :1 ; // Bits 10
} __attribute__((packed)) drv8305_status_04_reg_t;

typedef struct drv8305_status_05_reg_t_
{
    uint8_t IDRIVEP_HS     :4 ; // Bits 0:3
    uint8_t IDRIVEN_HS     :4 ; // Bits 3:7
    uint8_t TDRIVEN        :2 ; // Bits 8:9
    uint8_t RSV2           :1 ; // Bits 10
} __attribute__((packed)) drv8305_status_05_reg_t;

typedef struct drv8305_status_06_reg_t_
{
    uint8_t IDRIVEP_LS     :4 ; // Bits 0:3
    uint8_t IDRIVEN_LS     :4 ; // Bits 3:7
    uint8_t TDRIVEP        :2 ; // Bits 8:9
    uint8_t RSV3           :1 ; // Bits 10
} __attribute__((packed)) drv8305_status_06_reg_t;

typedef struct drv8305_status_08_reg_t_
{
    uint16_t RSV5          :11 ; // Bits 0:10
} __attribute__((packed)) drv8305_status_08_reg_t;

typedef struct drv8305_status_10_reg_t_
{
    uint8_t GAIN_CS1      :2 ; // Bits 0:1
    uint8_t GAIN_CS2      :2 ; // Bits 2:3
    uint8_t GAIN_CS3      :2 ; // Bits 4:5
    uint8_t CS_BLANK      :2 ; // Bits 6:7
    uint8_t DC_CAL_CH1    :1 ; // Bits 8
    uint8_t DC_CAL_CH2    :1 ; // Bits 9
    uint8_t DC_CAL_CH3    :1 ; // Bits 10

} __attribute__((packed)) drv8305_status_10_reg_t;

typedef struct drv8305_status_12_reg_t_
{
    uint8_t VDS_MODE      :3 ; // Bits 0:2
    uint8_t VDS_LEVEL     :5 ; // Bits 3:7
    uint8_t RSV8          :3 ; // Bits 8:10

} __attribute__((packed)) drv8305_status_12_reg_t;

esp_err_t drv8305_init(drv8305_t *dev);
esp_err_t drv8305_write_register(drv8305_t *dev, uint8_t register_address, uint16_t value);
esp_err_t drv8305_read_register(drv8305_t *dev, uint8_t register_address, uint16_t *value);

#endif