#include <string.h>
#include "rc522_helpers_internal.h"
#include "rc522_types_internal.h"
#include "rc522_driver_internal.h"
#include "driver/rc522_i2c.h"

RC522_LOG_DEFINE_BASE();

static esp_err_t rc522_i2c_install(const rc522_driver_handle_t driver)
{
    RC522_CHECK(driver == NULL);
    RC522_CHECK(driver->config == NULL);

    rc522_i2c_config_t *conf = (rc522_i2c_config_t *)(driver->config);

    i2c_master_bus_handle_t bus = NULL;
    if (conf->bus) {
        bus = conf->bus;
    }
    else {
        if (i2c_master_get_bus_handle(conf->port, &bus) == ESP_OK && bus) {
            conf->bus = bus;
            conf->bus_was_created = false;
        }

        else {
            i2c_master_bus_config_t bus_conf = {
                .i2c_port = conf->port,
                .sda_io_num = conf->config.sda_io_num,
                .scl_io_num = conf->config.scl_io_num,
                .clk_source = I2C_CLK_SRC_DEFAULT,
                .glitch_ignore_cnt = 7
            };

            RC522_RETURN_ON_ERROR(i2c_new_master_bus(&bus_conf, &bus));
            conf->bus = bus;
            conf->bus_was_created = true;
        }
    }

    i2c_device_config_t dev_conf = {
        .dev_addr_length = I2C_DEVICE_ADDRESS_LEN,
        .device_address = conf->device_address,
        .scl_speed_hz = conf->config.clk_speed,
        .scl_wait_us = 0
    };

    for (uint8_t a = 0x08; a <= 0x77; a++) {
        if (i2c_master_probe(conf->bus, a, 50) == ESP_OK) {
            ESP_LOGI("i2c-scan", "Found device @ 0x%02X", a);
        }
    }
    
    RC522_RETURN_ON_ERROR(i2c_master_probe(conf->bus, conf->device_address, 100));
    RC522_RETURN_ON_ERROR(i2c_master_bus_add_device(conf->bus, &dev_conf, &conf->dev));

    if (conf->rst_io_num > GPIO_NUM_NC) {
        RC522_RETURN_ON_ERROR(rc522_driver_init_rst_pin(conf->rst_io_num));
    }

    return ESP_OK;
}

static esp_err_t rc522_i2c_send(const rc522_driver_handle_t driver, uint8_t address, const rc522_bytes_t *bytes)
{
    RC522_CHECK(driver == NULL);
    RC522_CHECK(driver->config == NULL);
    RC522_CHECK_BYTES(bytes);
    
    uint8_t buffer2[64];
    if (bytes->length +1 > sizeof(buffer2)) {
        return ESP_ERR_INVALID_SIZE;
    }

    buffer2[0] = address;
    memcpy(buffer2 + 1, bytes->ptr, bytes->length);

    rc522_i2c_config_t *conf = (rc522_i2c_config_t *)(driver->config);

    RC522_RETURN_ON_ERROR(i2c_master_transmit(conf->dev, buffer2, (bytes->length + 1), conf->rw_timeout_ms));

    return ESP_OK;
}

static esp_err_t rc522_i2c_receive(const rc522_driver_handle_t driver, uint8_t address, rc522_bytes_t *bytes)
{
    RC522_CHECK(driver == NULL);
    RC522_CHECK(driver->config == NULL);
    RC522_CHECK_BYTES(bytes);

    rc522_i2c_config_t *conf = (rc522_i2c_config_t *)(driver->config);
    
    RC522_RETURN_ON_ERROR(i2c_master_transmit_receive(conf->dev, &address, 1, bytes->ptr, bytes->length, conf->rw_timeout_ms));

    return ESP_OK;
}

static esp_err_t rc522_i2c_reset(const rc522_driver_handle_t driver)
{
    RC522_CHECK(driver == NULL);
    RC522_CHECK(driver->config == NULL);

    rc522_i2c_config_t *conf = (rc522_i2c_config_t *)(driver->config);

    if (conf->rst_io_num < 0) {
        return RC522_ERR_RST_PIN_UNUSED;
    }

    RC522_RETURN_ON_ERROR(gpio_set_level(conf->rst_io_num, RC522_DRIVER_HARD_RST_PIN_PWR_DOWN_LEVEL));
    rc522_delay_ms(RC522_DRIVER_HARD_RST_PULSE_DURATION_MS);
    RC522_RETURN_ON_ERROR(gpio_set_level(conf->rst_io_num, !RC522_DRIVER_HARD_RST_PIN_PWR_DOWN_LEVEL));
    rc522_delay_ms(RC522_DRIVER_HARD_RST_PULSE_DURATION_MS);

    return ESP_OK;
}

static esp_err_t rc522_i2c_uninstall(const rc522_driver_handle_t driver)
{
    RC522_CHECK(driver == NULL);
    RC522_CHECK(driver->config == NULL);

    rc522_i2c_config_t *conf = (rc522_i2c_config_t *)(driver->config);

    if (conf->dev) {
        RC522_RETURN_ON_ERROR(i2c_master_bus_rm_device(conf->dev));
        conf->dev = NULL;
    }

    if (conf->bus) {
        RC522_RETURN_ON_ERROR(i2c_del_master_bus(conf->bus) && conf->bus_was_created);
        conf->bus = NULL;
        conf->bus_was_created = false;
    }

    return ESP_OK;
}

esp_err_t rc522_i2c_create(const rc522_i2c_config_t *config, rc522_driver_handle_t *driver)
{
    RC522_CHECK(config == NULL);
    RC522_CHECK(driver == NULL);

    RC522_RETURN_ON_ERROR(rc522_driver_create(config, sizeof(rc522_i2c_config_t), driver));

    (*driver)->install = rc522_i2c_install;
    (*driver)->send = rc522_i2c_send;
    (*driver)->receive = rc522_i2c_receive;
    (*driver)->reset = rc522_i2c_reset;
    (*driver)->uninstall = rc522_i2c_uninstall;

    return ESP_OK;
}
