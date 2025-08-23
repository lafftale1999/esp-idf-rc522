#pragma once

#include <driver/i2c.h> // TODO: Migrate to new i2c API
#include <driver/gpio.h>
#include "rc522_driver.h"
#include "driver/i2c_master.h"
#include "i2c_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    gpio_mode_t mode;
    gpio_num_t sda_io_num;
    gpio_num_t scl_io_num;
    bool sda_pullup_en;
    bool scl_pullup_en;
    uint32_t clk_speed;
} i2c_new_config_t;

typedef struct
{
    i2c_new_config_t config;
    i2c_port_t port;
    uint8_t device_address;
    uint32_t rw_timeout_ms;
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
    bool bus_was_created;
    /**
     * GPIO number of the RC522 RST pin.
     * Set to -1 if the RST pin is not connected.
     */
    gpio_num_t rst_io_num;
} rc522_i2c_config_t;

esp_err_t rc522_i2c_create(const rc522_i2c_config_t *config, rc522_driver_handle_t *driver);

#ifdef __cplusplus
}
#endif
