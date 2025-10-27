# dirty-bme680

A quick and 'dirty' BME680 sensor driver for ESP-IDF and PlatformIO.

This library provides a simple, pure ESP-IDF (v5.3+) driver for the Bosch BME680 4-in-1 sensor. It reads temperature, humidity, pressure, and gas resistance using I2C in forced mode.

## ⚠️ Disclaimer: Why 'dirty'?

As the name implies, this library was rapidly developed and is **not heavily tested**.

It was built for a school project and is not guaranteed to be free of bugs or inaccuracies. It gets the job done for hobbyist projects and provides a clear, understandable code structure based on the datasheet.

Do not use this in a Mars rover, a life-support system, or anything else that might explode. For a production-grade, heavily-tested solution, please refer to the official [Bosch BSEC library](https://www.bosch-sensortec.com/software-tools/software/bsec/).

## Features

* **Pure ESP-IDF:** Uses the modern `esp_driver_i2c` (v5.x+) and FreeRTOS tasks. No Arduino-ESP32 components.
* **Complete Compensation:** Implements all floating-point compensation formulas from the datasheet.
* **Forced Mode Operation:** Simple API to trigger a single T/H/P/Gas measurement and read the results.
* **Lightweight:** No complex dependencies.

## Wiring Diagram

The I2C address (0x77 or 0x76) is set by the `SDO` pin.

| BME680 Pin | ESP32 Pin | Notes |
| :--- | :--- | :--- |
| **VCC** | 3.3V | Power |
| **GND** | GND | Ground |
| **SDA** | GPIO 21 | I2C Data (configurable in your code) |
| **SCL** | GPIO 22 | I2C Clock (configurable in your code) |
| **SDO** |  | Sets I2C address to **0x77** (default) |
| | GND | *Alternatively, sets address to 0x76* |
| **CS** |  | Leave it be |

## Installation (PlatformIO)

Add this library as a dependency in your `platformio.ini` file.

If published, you can add it by name:
```ini
lib_deps =
    topiga/dirty-bme680
```

Alternatively, you can place the library in the `lib/` folder of your project, and PlatformIO will find it automatically.

## Quick Start / Usage

Here is a full `main.c` example.

```c
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
```

## Acknowledgements

* This driver was heavily inspired and guided by the amazing ESP-IDF I2C tutorials by [**ltkdt**](https://github.com/ltkdt).
* This driver would be impossible without the official [**Bosch BME680 Datasheet**](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme680-ds001.pdf), which provided all register maps and compensation formulas.

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.
