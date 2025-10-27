#include "bme680.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "bme680";

// --- BME680 Register Definitions ---
#define BME680_CALIB_DATA_1_ADDR    0x89 // 0x89 to 0xA1
#define BME680_CALIB_DATA_2_ADDR    0xE1 // 0xE1 to 0xF0
#define BME680_CALIB_DATA_3_ADDR    0x00 // res_heat_val
#define BME680_CALIB_DATA_4_ADDR    0x02 // res_heat_range
#define BME680_CALIB_DATA_5_ADDR    0x04 // range_sw_err

#define BME680_CHIP_ID_ADDR         0xD0 
#define BME680_CHIP_ID_VAL          0x61 

#define BME680_RESET_ADDR           0xE0 
#define BME680_RESET_CMD            0xB6 
#define BME680_CTRL_GAS_0_ADDR      0x70 
#define BME680_CTRL_GAS_1_ADDR      0x71 
#define BME680_CTRL_HUM_ADDR        0x72 
#define BME680_CTRL_MEAS_ADDR       0x74 
#define BME680_CONFIG_ADDR          0x75 

#define BME680_RES_HEAT_0_ADDR      0x5A 
#define BME680_GAS_WAIT_0_ADDR      0x64 

#define BME680_MEAS_STATUS_0_ADDR   0x1D 
#define BME680_DATA_FIELD_ADDR      0x1F // 0x1F to 0x2B

#define I2C_MASTER_TIMEOUT_MS   1000

/**
 * @brief Private structure to hold calibration data
 */
struct bme680_calib_data {
    uint16_t par_t1; int16_t par_t2; int8_t par_t3;
    uint16_t par_p1; int16_t par_p2; int8_t par_p3; int16_t par_p4; int16_t par_p5;
    int8_t par_p6; int8_t par_p7; int16_t par_p8; int16_t par_p9; uint8_t par_p10;
    uint16_t par_h1; uint16_t par_h2; int8_t par_h3; int8_t par_h4; int8_t par_h5;
    uint8_t par_h6; int8_t par_h7;
    int16_t par_g1; int16_t par_g2; int8_t par_g3;
    uint8_t res_heat_val;
    int8_t res_heat_range;
    int8_t range_sw_err;
};

/**
 * @brief Private device structure (implementation of the opaque handle)
 */
struct bme680_dev_s {
    i2c_master_dev_handle_t i2c_dev;      // I2C device handle
    struct bme680_calib_data calib;     // Sensor calibration data
    int32_t t_fine;                     // t_fine (from temp comp)
};

// Gas resistance lookup tables (from datasheet)
static const double const_array1[16] = {
    1, 1, 1, 1, 1, 0.99, 1, 0.992, 1, 1, 0.998, 0.995, 1, 0.99, 1, 1
};
static const double const_array2[16] = {
    8000000.0, 4000000.0, 2000000.0, 1000000.0, 499500.4995, 248262.1648,
    125000.0, 63004.03226, 31281.28128, 15625.0, 7812.5, 3906.25,
    1953.125, 976.5625, 488.28125, 244.140625
};

/* -------------------------------------------------------------------------- */
/* ------------------------ I2C HELPER FUNCTIONS ---------------------------- */
/* -------------------------------------------------------------------------- */

static esp_err_t read_byte_i2c(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t write_byte_i2c(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/* -------------------------------------------------------------------------- */
/* ---------------------- COMPENSATION FUNCTIONS ---------------------------- */
/* -------------------------------------------------------------------------- */

static uint8_t calc_heater_res(uint16_t target_temp, int16_t amb_temp, struct bme680_calib_data *calib)
{
    double var1, var2, var3, var4, var5;
    var1 = ((double)calib->par_g1 / 16.0) + 49.0;
    var2 = (((double)calib->par_g2 / 32768.0) * 0.0005) + 0.00235;
    var3 = (double)calib->par_g3 / 1024.0;
    var4 = var1 * (1.0 + (var2 * (double)target_temp));
    var5 = var4 + (var3 * (double)amb_temp);
    return (uint8_t)(3.4 * (((var5 * (4.0 / (4.0 + (double)calib->res_heat_range)) * (1.0 / (1.0 + ((double)calib->res_heat_val * 0.002)))) - 25)));
}

static double compensate_temp(bme680_dev_handle_t dev, int32_t temp_adc)
{
    double var1, var2, temp_comp;
    var1 = (((double)temp_adc / 16384.0) - ((double)dev->calib.par_t1 / 1024.0)) * (double)dev->calib.par_t2;
    var2 = ((((double)temp_adc / 131072.0) - ((double)dev->calib.par_t1 / 8192.0)) *
            (((double)temp_adc / 131072.0) - ((double)dev->calib.par_t1 / 8192.0))) *
           ((double)dev->calib.par_t3 * 16.0);
    
    dev->t_fine = (int32_t)(var1 + var2);
    temp_comp = (var1 + var2) / 5120.0;
    return temp_comp;
}

static double compensate_press(bme680_dev_handle_t dev, int32_t press_adc)
{
    double var1, var2, var3, press_comp;
    var1 = ((double)dev->t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * ((double)dev->calib.par_p6 / 131072.0);
    var2 = var2 + (var1 * (double)dev->calib.par_p5 * 2.0);
    var2 = (var2 / 4.0) + ((double)dev->calib.par_p4 * 65536.0);
    var1 = ((((double)dev->calib.par_p3 * var1 * var1) / 16384.0) + ((double)dev->calib.par_p2 * var1)) / 524288.0;
    var1 = (1.0 + (var1 / 32768.0)) * (double)dev->calib.par_p1;
    press_comp = 1048576.0 - (double)press_adc;

    if (var1 == 0) return 0; // Avoid division by zero

    press_comp = ((press_comp - (var2 / 4096.0)) * 6250.0) / var1;
    var1 = ((double)dev->calib.par_p9 * press_comp * press_comp) / 2147483648.0;
    var2 = press_comp * ((double)dev->calib.par_p8 / 32768.0);
    var3 = (press_comp / 256.0) * (press_comp / 256.0) * (press_comp / 256.0) * (dev->calib.par_p10 / 131072.0);
    press_comp = press_comp + (var1 + var2 + var3 + ((double)dev->calib.par_p7 * 128.0)) / 16.0;
    return press_comp;
}

static double compensate_hum(bme680_dev_handle_t dev, int32_t hum_adc, double temp_comp)
{
    double var1, var2, var3, var4, hum_comp;
    var1 = (double)hum_adc - (((double)dev->calib.par_h1 * 16.0) + (((double)dev->calib.par_h3 / 2.0) * temp_comp));
    var2 = var1 * (((double)dev->calib.par_h2 / 262144.0) * (1.0 + (((double)dev->calib.par_h4 / 16384.0) * temp_comp) + (((double)dev->calib.par_h5 / 1048576.0) * temp_comp * temp_comp)));
    var3 = (double)dev->calib.par_h6 / 16384.0;
    var4 = (double)dev->calib.par_h7 / 2097152.0;
    hum_comp = var2 + ((var3 + (var4 * temp_comp)) * var2 * var2);

    if (hum_comp > 100.0) hum_comp = 100.0;
    else if (hum_comp < 0.0) hum_comp = 0.0;
    return hum_comp;
}

static double compensate_gas(bme680_dev_handle_t dev, uint16_t gas_adc, uint8_t gas_range)
{
    double var1 = (1340.0 + (5.0 * (double)dev->calib.range_sw_err)) * (double)const_array1[gas_range];
    double gas_res = var1 * (double)const_array2[gas_range] / ((double)gas_adc - 512.0 + var1);
    return gas_res;
}

/* -------------------------------------------------------------------------- */
/* ----------------------- PUBLIC API FUNCTIONS ----------------------------- */
/* -------------------------------------------------------------------------- */

esp_err_t bme680_init(i2c_master_bus_handle_t bus_handle, uint8_t i2c_addr, bme680_dev_handle_t *dev_handle_out)
{
    esp_err_t err = ESP_OK;

    // Allocate memory for the device handle
    bme680_dev_handle_t dev = (bme680_dev_handle_t)malloc(sizeof(struct bme680_dev_s));
    if (dev == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for device handle");
        return ESP_ERR_NO_MEM;
    }
    memset(dev, 0, sizeof(struct bme680_dev_s));

    // Add device to I2C bus
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = i2c_addr,
        .scl_speed_hz = 100000, // 100KHz
    };
    err = i2c_master_bus_add_device(bus_handle, &dev_config, &dev->i2c_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(err));
        free(dev);
        return err;
    }

    // 1. Soft Reset
    err = write_byte_i2c(dev->i2c_dev, BME680_RESET_ADDR, BME680_RESET_CMD);
    if (err != ESP_OK) goto error_cleanup;
    vTaskDelay(100 / portTICK_PERIOD_MS); // Wait for reset

    // 2. Check Chip ID
    uint8_t chip_id;
    err = read_byte_i2c(dev->i2c_dev, BME680_CHIP_ID_ADDR, &chip_id, 1);
    if (err != ESP_OK) goto error_cleanup;
    if (chip_id != BME680_CHIP_ID_VAL) {
        ESP_LOGE(TAG, "Chip ID read 0x%X, expected 0x%X", chip_id, BME680_CHIP_ID_VAL);
        err = ESP_FAIL;
        goto error_cleanup;
    }
    ESP_LOGI(TAG, "BME680 Chip ID 0x%X OK.", chip_id);

    // 3. Get Calibration Data
    uint8_t data_buf1[25]; // 0x89-0xA1
    uint8_t data_buf2[16]; // 0xE1-0xF0

    err = read_byte_i2c(dev->i2c_dev, BME680_CALIB_DATA_1_ADDR, data_buf1, 25);
    if (err != ESP_OK) goto error_cleanup;
    err = read_byte_i2c(dev->i2c_dev, BME680_CALIB_DATA_2_ADDR, data_buf2, 16);
    if (err != ESP_OK) goto error_cleanup;

    // Parse Temp
    dev->calib.par_t1 = (uint16_t)((data_buf2[0xEA - 0xE1] << 8) | data_buf2[0xE9 - 0xE1]);
    dev->calib.par_t2 = (int16_t)((data_buf1[0x8B - 0x89] << 8) | data_buf1[0x8A - 0x89]);
    dev->calib.par_t3 = (int8_t)(data_buf1[0x8C - 0x89]);
    // Parse Press
    dev->calib.par_p1 = (uint16_t)((data_buf1[0x8F - 0x89] << 8) | data_buf1[0x8E - 0x89]);
    dev->calib.par_p2 = (int16_t)((data_buf1[0x91 - 0x89] << 8) | data_buf1[0x90 - 0x89]);
    dev->calib.par_p3 = (int8_t)(data_buf1[0x92 - 0x89]);
    dev->calib.par_p4 = (int16_t)((data_buf1[0x95 - 0x89] << 8) | data_buf1[0x94 - 0x89]);
    dev->calib.par_p5 = (int16_t)((data_buf1[0x97 - 0x89] << 8) | data_buf1[0x96 - 0x89]);
    dev->calib.par_p6 = (int8_t)(data_buf1[0x99 - 0x89]);
    dev->calib.par_p7 = (int8_t)(data_buf1[0x98 - 0x89]);
    dev->calib.par_p8 = (int16_t)((data_buf1[0x9D - 0x89] << 8) | data_buf1[0x9C - 0x89]);
    dev->calib.par_p9 = (int16_t)((data_buf1[0x9F - 0x89] << 8) | data_buf1[0x9E - 0x89]);
    dev->calib.par_p10 = (uint8_t)(data_buf1[0xA0 - 0x89]);
    // Parse Hum
    dev->calib.par_h1 = (uint16_t)((((uint16_t)data_buf2[0xE2 - 0xE1]) & 0x0F) << 8) | (uint16_t)data_buf2[0xE3 - 0xE1];
    dev->calib.par_h2 = (uint16_t)((((uint16_t)data_buf2[0xE2 - 0xE1]) & 0xF0) << 4) | (uint16_t)data_buf2[0xE1 - 0xE1];
    dev->calib.par_h3 = (int8_t)(data_buf2[0xE4 - 0xE1]);
    dev->calib.par_h4 = (int8_t)(data_buf2[0xE5 - 0xE1]);
    dev->calib.par_h5 = (int8_t)(data_buf2[0xE6 - 0xE1]);
    dev->calib.par_h6 = (uint8_t)(data_buf2[0xE7 - 0xE1]);
    dev->calib.par_h7 = (int8_t)(data_buf2[0xE8 - 0xE1]);
    // Parse Gas
    dev->calib.par_g1 = (int8_t)(data_buf2[0xED - 0xE1]);
    dev->calib.par_g2 = (int16_t)((data_buf2[0xEC - 0xE1] << 8) | data_buf2[0xEB - 0xE1]);
    dev->calib.par_g3 = (int8_t)(data_buf2[0xEE - 0xE1]);
    // Other Gas Calib
    err = read_byte_i2c(dev->i2c_dev, BME680_CALIB_DATA_3_ADDR, &dev->calib.res_heat_val, 1);
    if (err != ESP_OK) goto error_cleanup;
    err = read_byte_i2c(dev->i2c_dev, BME680_CALIB_DATA_4_ADDR, &data_buf1[0], 1);
    if (err != ESP_OK) goto error_cleanup;
    dev->calib.res_heat_range = (data_buf1[0] & 0x30) >> 4;
    err = read_byte_i2c(dev->i2c_dev, BME680_CALIB_DATA_5_ADDR, &data_buf1[0], 1);
    if (err != ESP_OK) goto error_cleanup;
    dev->calib.range_sw_err = (int8_t)((data_buf1[0] & 0xF0) >> 4);

    *dev_handle_out = dev;
    return ESP_OK;

error_cleanup:
    ESP_LOGE(TAG, "Error during init, cleaning up. Last error: %s", esp_err_to_name(err));
    i2c_master_bus_rm_device(dev->i2c_dev);
    free(dev);
    return err;
}

esp_err_t bme680_configure_default(bme680_dev_handle_t dev)
{
    esp_err_t err;

    // Set humidity oversampling (x1) -> 0b001
    err = write_byte_i2c(dev->i2c_dev, BME680_CTRL_HUM_ADDR, 0b001);
    if (err != ESP_OK) return err;

    // Set T(x2)/P(x16) oversampling -> 0b010 101 00
    uint8_t ctrl_meas_val = (0b010 << 5) | (0b101 << 2);
    err = write_byte_i2c(dev->i2c_dev, BME680_CTRL_MEAS_ADDR, ctrl_meas_val);
    if (err != ESP_OK) return err;

    // Set IIR filter (off) -> 0b000
    err = write_byte_i2c(dev->i2c_dev, BME680_CONFIG_ADDR, 0b000 << 2);
    if (err != ESP_OK) return err;

    // Set heater 0 duration (100ms) -> 0x59
    err = write_byte_i2c(dev->i2c_dev, BME680_GAS_WAIT_0_ADDR, 0x59);
    if (err != ESP_OK) return err;

    // Set heater 0 temperature (320C, @ 25C ambient)
    uint8_t res_heat_0 = calc_heater_res(320, 25, &dev->calib);
    err = write_byte_i2c(dev->i2c_dev, BME680_RES_HEAT_0_ADDR, res_heat_0);
    if (err != ESP_OK) return err;

    // Enable gas (profile 0) -> 0b0001 0000
    err = write_byte_i2c(dev->i2c_dev, BME680_CTRL_GAS_1_ADDR, (1 << 4) | (0 << 0));
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "BME680 Default configuration applied.");
    return ESP_OK;
}

esp_err_t bme680_trigger_measurement(bme680_dev_handle_t dev)
{
    uint8_t ctrl_meas;
    esp_err_t err;

    // Read current value
    err = read_byte_i2c(dev->i2c_dev, BME680_CTRL_MEAS_ADDR, &ctrl_meas, 1);
    if (err != ESP_OK) return err;

    // Set mode<1:0> to 0b01 (forced mode)
    return write_byte_i2c(dev->i2c_dev, BME680_CTRL_MEAS_ADDR, (ctrl_meas & 0xFC) | 0b01);
}

esp_err_t bme680_get_measurement(bme680_dev_handle_t dev, bme680_data_t *data_out)
{
    esp_err_t err;

    // Poll status register 0x1D for new_data_0<7>
    uint8_t status = 0;
    do {
        vTaskDelay(5 / portTICK_PERIOD_MS); // Poll
        err = read_byte_i2c(dev->i2c_dev, BME680_MEAS_STATUS_0_ADDR, &status, 1);
        if (err != ESP_OK) return err;
    } while ((status & 0x80) == 0);

    // Read data fields 0x1F to 0x2B (13 bytes)
    uint8_t raw_data[13];
    err = read_byte_i2c(dev->i2c_dev, BME680_DATA_FIELD_ADDR, raw_data, 13);
    if (err != ESP_OK) return err;

    // Parse data
    uint8_t gas_lsb = raw_data[0x2B - 0x1F];
    data_out->gas_valid = (gas_lsb & 0x20) > 0; // gas_valid_r<5>
    bool heat_stab = (gas_lsb & 0x10) > 0; // heat_stab_r<4>
    data_out->gas_valid = data_out->gas_valid && heat_stab;

    int32_t press_adc = (int32_t)((raw_data[0x1F - 0x1F] << 12) | (raw_data[0x20 - 0x1F] << 4) | (raw_data[0x21 - 0x1F] >> 4));
    int32_t temp_adc  = (int32_t)((raw_data[0x22 - 0x1F] << 12) | (raw_data[0x23 - 0x1F] << 4) | (raw_data[0x24 - 0x1F] >> 4));
    int32_t hum_adc   = (int32_t)((raw_data[0x25 - 0x1F] << 8) | raw_data[0x26 - 0x1F]);
    uint16_t gas_adc  = (uint16_t)((raw_data[0x2A - 0x1F] << 2) | (gas_lsb >> 6));
    uint8_t gas_range = gas_lsb & 0x0F;

    // Compensate data
    double temp_c = compensate_temp(dev, temp_adc);
    data_out->temperature = temp_c;
    data_out->pressure = compensate_press(dev, press_adc) / 100.0; // Convert to hPa
    data_out->humidity = compensate_hum(dev, hum_adc, temp_c);

    if (data_out->gas_valid) {
        data_out->gas_resistance = compensate_gas(dev, gas_adc, gas_range);
    } else {
        data_out->gas_resistance = 0; // Invalid reading
    }

    return ESP_OK;
}

esp_err_t bme680_delete(bme680_dev_handle_t dev)
{
    if (dev == NULL) return ESP_OK;
    esp_err_t err = i2c_master_bus_rm_device(dev->i2c_dev);
    free(dev);
    return err;
}