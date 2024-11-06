#include <stdio.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sw_i2c.h"  // Assuming your software I2C functions are in sw_i2c.h

#define SDA_PIN 21      // Define SDA pin (adjust as needed)
#define SCL_PIN 22      // Define SCL pin (adjust as needed)
#define BQ76952_ADDRESS 0x08  // I2C address of BQ76952 (verify from datasheet)

// Register address to read (example register)
#define BQ76952_REG_ADDR 0x14  // Example register address

static const char* TAG = "main";

// Function to read a register from the BQ76952
esp_err_t bq76952_read_register(uint8_t reg_addr, uint8_t *data, size_t len) {
    // Start I2C transaction
    sw_i2c_master_start();

    // Send device address with write flag
    if (sw_i2c_master_write_byte((BQ76952_ADDRESS << 1) | LOW) != ACK) {
        ESP_LOGE(TAG, "Failed to address BQ76952 for write");
        sw_i2c_master_stop();
        return ESP_FAIL;
    }

    // Send the register address we want to read
    if (sw_i2c_master_write_byte(reg_addr) != ACK) {
        ESP_LOGE(TAG, "Failed to send register address");
        sw_i2c_master_stop();
        return ESP_FAIL;
    }

    // Restart I2C transaction
    sw_i2c_master_start();

    // Send device address with read flag
    if (sw_i2c_master_write_byte((BQ76952_ADDRESS << 1) | HIGH) != ACK) {
        ESP_LOGE(TAG, "Failed to address BQ76952 for read");
        sw_i2c_master_stop();
        return ESP_FAIL;
    }

    // Read data
    for (size_t i = 0; i < len; i++) {
        data[i] = 0;
        sw_i2c_master_read_byte(&data[i], (i == len - 1) ? NAK : ACK);  // NAK on last byte
    }

    // Stop I2C transaction
    sw_i2c_master_stop();

    return ESP_OK;
}

void app_main() {

    uint8_t reg_address = 0x14;

    uint8_t read_data[2];  // Example: read two bytes from the register
    uint16_t cell_volt[16];
    // Initialize the software I2C
    if (sw_i2c_init(SDA_PIN, SCL_PIN) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C");
        return;
    }
    ESP_LOGI(TAG, "I2C initialized successfully");
    uint8_t i = 0;
    // Read data from BQ76952 register
    while (i < 16)
    {
        esp_err_t ret = bq76952_read_register(reg_address, read_data, sizeof(read_data));
        if (ret == ESP_OK)
        {   
            cell_volt[i] = read_data[0] | read_data[1] << 8;
            
            ESP_LOGI(TAG, "Data read from BQ76952 [%d]: %.4f", i, (float)cell_volt[i]/1000);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to read data from BQ76952");
        }

        i++;
        reg_address += 2;

        // Wait indefinitely
        // while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay to avoid busy looping
    }
}
