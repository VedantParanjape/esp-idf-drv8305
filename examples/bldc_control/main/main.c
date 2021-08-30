#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "driver/spi_master.h"
#include "drv8305.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define PWM_H_A 17
#define PWM_H_B 5
#define PWM_H_C 18

#define PWM_FREQUENCY 20000

#define MISO_PIN 12
#define MOSI_PIN 13
#define SCLK_PIN 14
#define CS_PIN 15
#define DRV8305_EN 2
#define DRV8305_NFAULT 4

static const char TAG[] = "bldc mcpwm";

void
init_gpio ()
{

  // MCPWM
  mcpwm_gpio_init (MCPWM_UNIT_0, MCPWM0A, PWM_H_A);
  mcpwm_gpio_init (MCPWM_UNIT_0, MCPWM1A, PWM_H_B);
  mcpwm_gpio_init (MCPWM_UNIT_0, MCPWM2A, PWM_H_C);

  drv8305_t dev = { .DRV_EN_GATE_pin = DRV8305_EN,
                    .DRV_N_FAULT_pin = DRV8305_NFAULT,
                    .DRV_MISO_SDO_pin = MISO_PIN,
                    .DRV_MOSI_SDI_pin = MOSI_PIN,
                    .DRV_SCLK_pin = SCLK_PIN,
                    .DRV_SCS_pin = CS_PIN,
                    .spi_host = HSPI_HOST,
                    .max_spi_clockspeed = 1000000 };

  // Initialize DRV8305
  ESP_ERROR_CHECK (drv8305_init (&dev));

  drv8305_control_07_reg_t pwm_mode_config;
  drv8305_read_control_07_register (&dev, &pwm_mode_config);
  ESP_LOGI (TAG, "INITIAL VALUE %x", pwm_mode_config.PWM_MODE);
  pwm_mode_config.PWM_MODE = 0b01;
  ESP_ERROR_CHECK (drv8305_write_control_07_register (&dev, pwm_mode_config));

  drv8305_read_control_07_register (&dev, &pwm_mode_config);
  ESP_LOGI (TAG, "FINAL VALUE %x", pwm_mode_config.PWM_MODE);
}

void
mcpwm_config ()
{
  mcpwm_config_t pwm_config;
  pwm_config.frequency = PWM_FREQUENCY; // frequency = 20000Hz
  pwm_config.cmpr_a = 0;                // duty cycle of PWMxA = 50.0%
  pwm_config.cmpr_b = 0;                // duty cycle of PWMxB = 50.0%
  pwm_config.counter_mode
      = MCPWM_UP_DOWN_COUNTER;              // Up-down counter (triangle wave)
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0; // Active HIGH
  mcpwm_init (MCPWM_UNIT_0, MCPWM_TIMER_0,
              &pwm_config); // Configure PWM0A & PWM0B with above settings
  mcpwm_init (MCPWM_UNIT_0, MCPWM_TIMER_1,
              &pwm_config); // Configure PWM0A & PWM0B with above settings
  mcpwm_init (MCPWM_UNIT_0, MCPWM_TIMER_2,
              &pwm_config); // Configure PWM0A & PWM0B with above settings

  mcpwm_stop (MCPWM_UNIT_0, MCPWM_TIMER_0);
  mcpwm_stop (MCPWM_UNIT_0, MCPWM_TIMER_1);
  mcpwm_stop (MCPWM_UNIT_0, MCPWM_TIMER_2);

  mcpwm_start (MCPWM_UNIT_0, MCPWM_TIMER_0);
  mcpwm_start (MCPWM_UNIT_0, MCPWM_TIMER_1);
  mcpwm_start (MCPWM_UNIT_0, MCPWM_TIMER_2);

  mcpwm_sync_enable (MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_SELECT_SYNC0, 0);
  mcpwm_sync_enable (MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_SELECT_SYNC0, 0);
  mcpwm_sync_enable (MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_SELECT_SYNC0, 0);
}

void
app_main ()
{
  init_gpio ();
  mcpwm_config ();

  // PWM values to make it spin
  int pwm[3][3] = { { 0, 40, 80 }, { 80, 40, 0 }, { 40, 80, 0 } };

  // SPIN motor with reasonable speed, adjust delay accordingly
  for (size_t i = 1; i <= 3; i++)
    {
      mcpwm_set_duty (MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, pwm[0][i % 3]);
      mcpwm_set_duty (MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, pwm[1][i % 3]);
      mcpwm_set_duty (MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, pwm[2][i % 3]);
      vTaskDelay (10 / portTICK_PERIOD_MS);
      if (i == 3)
        {
          i = 0;
        }
    }
}