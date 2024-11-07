#include <stdio.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sw_i2c.h" 
#include "commands.h"

#define SDA_PIN 21      // Define SDA pin (adjust as needed)
#define SCL_PIN 22      // Define SCL pin (adjust as needed)
#define BQ76952_ADDRESS 0x08  // I2C address of BQ76952 (verify from datasheet)

// Register address to read (example register)
#define BQ76952_REG_ADDR 0x14  // Example register address

static const char* TAG = "I2C_BUS";

#define BQ76952_SUBCMD_REG_L 0x3E  // Register to write lower byte of subcommand
#define BQ76952_SUBCMD_REG_H 0x3F  // Register to write upper byte of subcommand
#define BQ76952_RESPONSE_START 0x40  // Starting address of response buffer
#define BQ76952_RESPONSE_LENGTH 0x61 // Register to read response length
#define BQ76952_CHECKSUM 0x60        // Register to read checksum

uint8_t res_len;

// Helper function to read a single byte from a register
static esp_err_t bq76952_read_byte(uint8_t reg, uint8_t *data) {
    ets_delay_us(8500);
    sw_i2c_master_start();
    if (sw_i2c_master_write_byte((BQ76952_ADDRESS << 1) | LOW) != ACK ||
        sw_i2c_master_write_byte(reg) != ACK) {
        sw_i2c_master_stop();
        return ESP_FAIL;
    }
    sw_i2c_master_start();
    if (sw_i2c_master_write_byte((BQ76952_ADDRESS << 1) | HIGH) != ACK) {
        sw_i2c_master_stop();
        return ESP_FAIL;
    }
    sw_i2c_master_read_byte(data, NAK);
    sw_i2c_master_stop();
    return ESP_OK;
}

// Function to read subcommand response with error checking
esp_err_t bq76952_read_subcommand(uint16_t subcommand, uint8_t *response, size_t *response_len) {
    
    if (response == NULL || response_len == NULL) {
        ESP_LOGE(TAG, "Invalid response buffer or response length pointer");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t temp_data = 0;

    // Step 1: Write lower byte of subcommand to 0x3E
    sw_i2c_master_start();
    if (sw_i2c_master_write_byte((BQ76952_ADDRESS << 1) | LOW) != ACK ||
        sw_i2c_master_write_byte(BQ76952_SUBCMD_REG_L) != ACK ||
        sw_i2c_master_write_byte(subcommand & 0xFF) != ACK) {
        sw_i2c_master_stop();
        return ESP_FAIL;
    }
    sw_i2c_master_stop();

    // Step 2: Write upper byte of subcommand to 0x3F
    sw_i2c_master_start();
    if (sw_i2c_master_write_byte((BQ76952_ADDRESS << 1) | LOW) != ACK ||
        sw_i2c_master_write_byte(BQ76952_SUBCMD_REG_H) != ACK ||
        sw_i2c_master_write_byte((subcommand >> 8) & 0xFF) != ACK) {
        sw_i2c_master_stop();
        return ESP_FAIL;
    }
    sw_i2c_master_stop();

    // Step 3: Poll 0x3E and 0x3F until they match the subcommand values, with timeout
    uint8_t reg3E, reg3F;
    int attempts = 10;  // Timeout after 10 attempts (adjust as needed)
    do {
        if (bq76952_read_byte(BQ76952_SUBCMD_REG_L, &reg3E) != ESP_OK ||
            bq76952_read_byte(BQ76952_SUBCMD_REG_H, &reg3F) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to poll subcommand completion");
            return ESP_FAIL;
        }
        if (reg3E == 0xFF && reg3F == 0xFF) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        if (--attempts <= 0) {
            ESP_LOGE(TAG, "Subcommand polling timed out");
            return ESP_ERR_TIMEOUT;
        }
    } while ((reg3E != (subcommand & 0xFF)) || (reg3F != ((subcommand >> 8) & 0xFF)));

    // Step 4: Read response length from 0x61
    if (bq76952_read_byte(BQ76952_RESPONSE_LENGTH, &temp_data) != ESP_OK) {
        return ESP_FAIL;
    }
    *response_len = temp_data;
    res_len = temp_data;

    // Step 5: Read buffer starting at 0x40 for the expected length
    sw_i2c_master_start();
    if (sw_i2c_master_write_byte((BQ76952_ADDRESS << 1) | LOW) != ACK ||
        sw_i2c_master_write_byte(BQ76952_RESPONSE_START) != ACK) {
        sw_i2c_master_stop();
        return ESP_FAIL;
    }
    sw_i2c_master_start();
    if (sw_i2c_master_write_byte((BQ76952_ADDRESS << 1) | HIGH) != ACK) {
        sw_i2c_master_stop();
        return ESP_FAIL;
    }
    for (size_t i = 0; i < *response_len; i++) {
        if (sw_i2c_master_read_byte(&response[i], (i == *response_len - 1) ? NAK : ACK) != ACK) {
            ESP_LOGE(TAG, "Failed to read response byte %d", i);
            sw_i2c_master_stop();
            return ESP_FAIL;
        }
        //ESP_LOGI(TAG, "Data[%d] 0x%02X", i, response[i]);
    }
    sw_i2c_master_stop();

    /*// Step 6: Read checksum at 0x60 and verify
    uint8_t checksum;
    if (bq76952_read_byte(BQ76952_CHECKSUM, &checksum) != ESP_OK) {
        return ESP_FAIL;
    }
    uint8_t calculated_checksum = 0;
    for (size_t i = 0; i < *response_len; i++) {
        calculated_checksum += response[i];
    }
    if (calculated_checksum != checksum) {
        ESP_LOGE(TAG, "Checksum mismatch: calculated 0x%02X, expected 0x%02X", calculated_checksum, checksum);
        return ESP_FAIL;
    }*/

    return ESP_OK;
}

// Function to write data to a BQ76952 register
esp_err_t bq76952_write_register(uint8_t reg_addr, uint8_t *data, size_t len) {
    // Start I2C transaction
    sw_i2c_master_start();

    // Send device address with write flag
    if (sw_i2c_master_write_byte((BQ76952_ADDRESS << 1) | LOW) != ACK) {
        ESP_LOGE(TAG, "Failed to address BQ76952 for write");
        sw_i2c_master_stop();
        return ESP_FAIL;
    }

    // Send the register address we want to write
    if (sw_i2c_master_write_byte(reg_addr) != ACK) {
        ESP_LOGE(TAG, "Failed to send register address");
        sw_i2c_master_stop();
        return ESP_FAIL;
    }

    // Write data bytes
    for (size_t i = 0; i < len; i++) {
        if (sw_i2c_master_write_byte(data[i]) != ACK) {
            ESP_LOGE(TAG, "Failed to write data byte %d", i);
            sw_i2c_master_stop();
            return ESP_FAIL;
        }
    }

    // Stop I2C transaction
    sw_i2c_master_stop();

    return ESP_OK;
}
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

void app_main() 
{
    esp_err_t ret = 0;
    uint8_t reg_address = 0x14;
    
    // Example subcommand to read (replace with the actual subcommand value)
    uint16_t subcommand = DEVICE_NUM_SUB_CMD;

    //uint8_t read_data[2];  // Example: read two bytes from the register
    uint16_t cell_volt[16];
    
    uint8_t response[2];  // Array to store the response from the subcommand


    // Initialize the software I2C
    if (sw_i2c_init(SDA_PIN, SCL_PIN) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C");
        return;
    }
    ESP_LOGI(TAG, "I2C initialized successfully");


    int8_t i = 0 , j;
    size_t response_len = 0;

    // Read data from BQ76952 register
    while (1)
    {
        ret = bq76952_read_register(reg_address, (uint8_t *)cell_volt, sizeof(cell_volt));
        if (ret == ESP_OK)
        {
            ESP_LOGI(TAG, "Data read from BQ76952 Success");
        }
        else
        {
            ESP_LOGE(TAG, "Failed to read Data from BQ76952d");
        }

        for(i = 0; i < 16; i++ )
        {
            j = i + 1;
            ESP_LOGI(TAG, "Data read from BQ76952 [%02d]: %.3f", j, (float)cell_volt[i] / 1000);
        }
        printf("\n");
        // Read the subcommand response
       
        ret = bq76952_read_subcommand(subcommand, response, &response_len);
        if (ret == ESP_OK)
        {
            for(int8_t i = 0; i < res_len; i++){
            ESP_LOGI(TAG, "Subcommand response[%d]: 0x%02X", i, response[i]);
            }
        }
        else
        {
            ESP_LOGE(TAG, "Failed to read subcommand response");
        }

        vTaskDelay(pdMS_TO_TICKS(5000)); 
        printf("\n");

    }
}
