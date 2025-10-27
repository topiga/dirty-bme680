#ifndef BME680_H
#define BME680_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

// Default I2C address
#define BME680_I2C_ADDR_DEFAULT   0x77  // SDO connected to VDDIO
#define BME680_I2C_ADDR_ALT       0x76  // SDO connected to GND

/**
 * @brief Opaque handle for the BME680 device
 */
typedef struct bme680_dev_s *bme680_dev_handle_t;

/**
 * @brief Structure to hold compensated sensor data
 */
typedef struct {
    double temperature;     // Compensated temperature in degrees Celsius
    double pressure;        // Compensated pressure in hPa (Hectopascals)
    double humidity;        // Compensated humidity in % rH
    double gas_resistance;  // Compensated gas resistance in Ohms
    bool gas_valid;         // True if gas measurement was valid and heater stable
} bme680_data_t;

/**
 * @brief Initialize the BME680 device
 *
 * This function creates a new I2C device handle, soft-resets the sensor,
 * checks the chip ID, and reads all calibration data.
 *
 * @param bus_handle Handle for the I2C master bus (must be pre-initialized)
 * @param i2c_addr The 7-bit I2C address of the sensor (e.g., BME680_I2C_ADDR_DEFAULT)
 * @param dev_handle_out Pointer to store the new device handle
 * @return ESP_OK on success, ESP_FAIL or other error code on failure
 */
esp_err_t bme680_init(i2c_master_bus_handle_t bus_handle, uint8_t i2c_addr, bme680_dev_handle_t *dev_handle_out);

/**
 * @brief Apply a default sensor configuration
 *
 * This sets:
 * - Humidity oversampling: x1
 * - Temperature oversampling: x2
 * - Pressure oversampling: x16
 * - IIR Filter: Off
 * - Gas measurement: Enabled, profile 0
 * - Heater profile 0: 320 degC for 100 ms
 *
 * @param dev_handle Handle for the BME680 device
 * @return ESP_OK on success, ESP_FAIL on I2C write error
 */
esp_err_t bme680_configure_default(bme680_dev_handle_t dev_handle);

/**
 * @brief Trigger a single measurement (Forced Mode)
 *
 * @param dev_handle Handle for the BME680 device
 * @return ESP_OK on success, ESP_FAIL on I2C write error
 */
esp_err_t bme680_trigger_measurement(bme680_dev_handle_t dev_handle);

/**
 * @brief Wait for, read, and compensate a measurement
 *
 * This function will poll the sensor's status register until a new
 * measurement is ready. It then reads the raw data, applies all
 * compensation formulas, and populates the data_out struct.
 *
 * @param dev_handle Handle for the BME680 device
 * @param data_out Pointer to a bme680_data_t struct to store the results
 * @return ESP_OK on success, ESP_FAIL on I2C read error
 */
esp_err_t bme680_get_measurement(bme680_dev_handle_t dev_handle, bme680_data_t *data_out);

/**
 * @brief Delete the BME680 device handle
 *
 * This removes the device from the I2C bus and frees all allocated memory.
 *
 * @param dev_handle Handle for the BME680 device
 * @return ESP_OK on success
 */
esp_err_t bme680_delete(bme680_dev_handle_t dev_handle);

#ifdef __cplusplus
}
#endif

#endif // BME680_H