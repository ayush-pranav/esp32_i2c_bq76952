#ifndef _sw_I2C_H
#define _sw_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>

#define LOW   0x00
#define HIGH  0x01

#define ACK   0x00
#define NAK   0x01

#define CLOCK_STRETCH_TIMEOUT   1000

esp_err_t sw_i2c_init(uint8_t sda, uint8_t scl);
esp_err_t sw_i2c_master_start();
esp_err_t sw_i2c_master_stop();

esp_err_t sw_i2c_master_write_byte(uint8_t buffer);
esp_err_t sw_i2c_master_write(uint8_t *buffer, uint8_t length);

esp_err_t sw_i2c_master_read_byte(uint8_t *buffer, bool ack);
esp_err_t sw_i2c_master_read(uint8_t *buffer, uint16_t length, bool ack);

#ifdef __cplusplus
}
#endif

#endif