#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "bme680.h" // Include the library

// I2C Pin Configuration
#define I2C_PORT        I2C_NUM_0
#define I2C_SDA_PIN     GPIO_NUM_21
#define I2C_SCL_PIN     GPIO_NUM_22

static const char *TAG = "app_main";
static i2c_master_bus_handle_t bus_handle;
static bme680_dev_handle_t bme680_dev = NULL;

/**
 * @brief Initialize the I2C master bus
 */
static void i2c_master_init_bus(void)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));
}

/**
 * @brief Main sensor reading task
 */
void sensor_task(void *arg)
{
    bme680_data_t sensor_data;

    // Initialize the BME680
    ESP_LOGI(TAG, "Initializing BME680...");
    esp_err_t err = bme680_init(bus_handle, BME680_I2C_ADDR_DEFAULT, &bme680_dev);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BME680 init failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    // Apply default configuration (T_x2, P_x16, H_x1, 320C/100ms gas)
    err = bme680_configure_default(bme680_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BME680 config failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "BME680 init success. Starting measurement loop.");

    while(1)
    {
        // 1. Trigger a measurement
        err = bme680_trigger_measurement(bme680_dev);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to trigger measurement: %s", esp_err_to_name(err));
        } else {
            // 2. Wait for and get the results
            err = bme680_get_measurement(bme680_dev, &sensor_data);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to get measurement: %s", esp_err_to_name(err));
            } else {
                // 3. Print the results
                printf("\n--- New Measurement ---\n");
                printf("Temperature: %.2f C\n", sensor_data.temperature);
                printf("Pressure:    %.2f hPa\n", sensor_data.pressure);
                printf("Humidity:    %.2f %%rH\n", sensor_data.humidity);
                if (sensor_data.gas_valid) {
                    printf("Gas Resist:  %.0f Ohms\n", sensor_data.gas_resistance);
                } else {
                    printf("Gas Resist:  (Measurement Invalid)\n");
                }
            }
        }

        // Wait 2 seconds for the next measurement
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    // 1. Initialize I2C Bus
    i2c_master_init_bus();

    // 2. Create the sensor task
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
}
