/*
 * INA219 ESP-IDF Driver (i2c_master) - revised fork
 *
 * Original work: Copyright (C) 2025 TNY Robotics, MIT License
 *   https://github.com/tny-robotics/ina219-esp-idf
 * Modifications: see CHANGES.md. Distributed under the same MIT License.
 *
 * SPDX-License-Identifier: MIT
 */

#include "ina219.h"
#include <math.h>
#include <string.h>
#include <esp_log.h>

static const char *TAG = "ina219";

/* Registers */
#define INA219_REG_CONFIG       0x00
#define INA219_REG_SHUNT_VOLT   0x01
#define INA219_REG_BUS_VOLT     0x02
#define INA219_REG_POWER        0x03
#define INA219_REG_CURRENT      0x04
#define INA219_REG_CALIBRATION  0x05

#define INA219_CFG_RESET        0x8000
#define INA219_CFG_MODE_MASK    0x0007
#define INA219_BUS_OVF          0x0001  /* math overflow flag */
#define INA219_BUS_CNVR         0x0002  /* conversion ready flag */

#define INA219_BUS_VOLT_LSB     0.004f  /* 4 mV per bit */
#define INA219_SHUNT_VOLT_LSB   0.01f   /* 10 uV per bit = 0.01 mV */
#define INA219_CAL_CONSTANT     0.04096 /* from the datasheet */

/* i2c_master_transmit*() take the timeout in MILLISECONDS (not ticks). */
#define INA219_I2C_TIMEOUT_MS   100

/* Full-scale shunt voltage for each PGA setting, in mV. */
static const float s_gain_range_mV[4] = { 40.0f, 80.0f, 160.0f, 320.0f };

static esp_err_t ina219_write_reg(ina219_handle_t handle, uint8_t reg_addr, uint16_t val)
{
    uint8_t buf[3] = { reg_addr, (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) }; /* MSB first */
    return i2c_master_transmit(handle.dev_handle, buf, sizeof(buf), INA219_I2C_TIMEOUT_MS);
}

static esp_err_t ina219_read_reg(ina219_handle_t handle, uint8_t reg_addr, uint16_t *val)
{
    uint8_t buf[2];
    esp_err_t err = i2c_master_transmit_receive(handle.dev_handle, &reg_addr, 1, buf, sizeof(buf),
                                                INA219_I2C_TIMEOUT_MS);
    if (err == ESP_OK) {
        *val = (uint16_t)((buf[0] << 8) | buf[1]);
    }
    return err;
}

static bool adc_setting_valid(ina219_adc_res_avg_t v)
{
    return (v <= INA219_ADC_12BIT_1SAMP) || (v >= INA219_ADC_12BIT_2SAMP && v <= INA219_ADC_12BIT_128SAMP);
}

esp_err_t ina219_create(i2c_master_bus_handle_t i2c_bus_handle, ina219_info_t info, ina219_handle_t *out_handle)
{
    if (i2c_bus_handle == NULL || out_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (info.address < 0x40 || info.address > 0x4F) {
        ESP_LOGE(TAG, "invalid address 0x%02X (INA219 uses 0x40..0x4F)", info.address);
        return ESP_ERR_INVALID_ARG;
    }

    memset(out_handle, 0, sizeof(*out_handle)); /* lsb = 0 => "not configured" is reliable */

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = info.address,
        .scl_speed_hz = info.clock_speed ? info.clock_speed : INA219_DEFAULT_CLOCK,
    };
    return i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &out_handle->dev_handle);
}

esp_err_t ina219_pick_gain(float shunt_ohms, float max_current_A, ina219_gain_t *out_gain)
{
    if (out_gain == NULL || !(shunt_ohms > 0.0f) || !(max_current_A > 0.0f)) {
        return ESP_ERR_INVALID_ARG;
    }
    const float v_mV = max_current_A * shunt_ohms * 1000.0f;
    for (int g = 0; g < 4; g++) {
        if (v_mV <= s_gain_range_mV[g] * 1.01f) { /* 1% tolerance for float rounding */
            *out_gain = (ina219_gain_t)g;
            return ESP_OK;
        }
    }
    return ESP_ERR_INVALID_ARG;
}

esp_err_t ina219_config(ina219_handle_t *handle, ina219_config_t config)
{
    if (handle == NULL || handle->dev_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!(config.shunt_resistor_ohms > 0.0f) || !(config.max_expected_current_A > 0.0f)) {
        ESP_LOGE(TAG, "shunt resistor and max current must be > 0");
        return ESP_ERR_INVALID_ARG;
    }
    if ((unsigned)config.bus_voltage_range > INA219_BUS_VOLTAGE_RANGE_32V ||
        (unsigned)config.gain > INA219_GAIN_8_320MV ||
        !adc_setting_valid(config.badc_res) || !adc_setting_valid(config.sadc_res) ||
        (unsigned)config.mode > INA219_MODE_SHUNT_BUS_CONT) {
        ESP_LOGE(TAG, "invalid configuration enum value");
        return ESP_ERR_INVALID_ARG;
    }

    /* The shunt voltage at max current must fit in the PGA range, or readings clip. */
    const float vshunt_max_mV = config.max_expected_current_A * config.shunt_resistor_ohms * 1000.0f;
    if (vshunt_max_mV > s_gain_range_mV[config.gain] * 1.01f) {
        ESP_LOGE(TAG, "%.1f mV at max current exceeds the %.0f mV PGA range: increase gain or lower max current",
                 vshunt_max_mV, s_gain_range_mV[config.gain]);
        return ESP_ERR_INVALID_ARG;
    }

    /* Cal = trunc(0.04096 / (Current_LSB * R_shunt)), Current_LSB = Imax / 2^15 (datasheet). */
    const double ideal_lsb = (double)config.max_expected_current_A / 32768.0;
    const double cal_d = floor(INA219_CAL_CONSTANT / (ideal_lsb * (double)config.shunt_resistor_ohms));
    if (!(cal_d >= 2.0) || cal_d > 65535.0) {
        ESP_LOGE(TAG, "calibration value %.0f out of range (check shunt and max current)", cal_d);
        return ESP_ERR_INVALID_ARG;
    }
    /* Bit 0 of the calibration register is void (always reads 0): use an even value. */
    const uint16_t cal_val = (uint16_t)cal_d & 0xFFFE;

    /* Use the LSB that corresponds to the value really written, not the ideal one. */
    const double real_lsb = INA219_CAL_CONSTANT / ((double)cal_val * (double)config.shunt_resistor_ohms);

    uint16_t cfg_val = (uint16_t)(((uint16_t)config.bus_voltage_range << 13) |
                                  ((uint16_t)config.gain << 11) |
                                  ((uint16_t)config.badc_res << 7) |
                                  ((uint16_t)config.sadc_res << 3) |
                                  (uint16_t)config.mode);

    esp_err_t err = ina219_write_reg(*handle, INA219_REG_CALIBRATION, cal_val);
    if (err != ESP_OK) return err;
    err = ina219_write_reg(*handle, INA219_REG_CONFIG, cfg_val);
    if (err != ESP_OK) return err;

    handle->current_lsb = (float)real_lsb;
    handle->power_lsb = (float)(20.0 * real_lsb); /* Power_LSB = 20 x Current_LSB */
    return ESP_OK;
}

esp_err_t ina219_reset(ina219_handle_t handle)
{
    return ina219_write_reg(handle, INA219_REG_CONFIG, INA219_CFG_RESET);
}

esp_err_t ina219_sleep(ina219_handle_t handle)
{
    uint16_t cfg;
    esp_err_t err = ina219_read_reg(handle, INA219_REG_CONFIG, &cfg);
    if (err != ESP_OK) return err;
    cfg = (uint16_t)((cfg & ~INA219_CFG_MODE_MASK) | INA219_MODE_POWER_DOWN);
    return ina219_write_reg(handle, INA219_REG_CONFIG, cfg);
}

esp_err_t ina219_wake_up(ina219_handle_t handle, ina219_mode_t mode)
{
    if ((unsigned)mode > INA219_MODE_SHUNT_BUS_CONT) return ESP_ERR_INVALID_ARG;
    uint16_t cfg;
    esp_err_t err = ina219_read_reg(handle, INA219_REG_CONFIG, &cfg);
    if (err != ESP_OK) return err;
    cfg = (uint16_t)((cfg & ~INA219_CFG_MODE_MASK) | (uint16_t)mode);
    return ina219_write_reg(handle, INA219_REG_CONFIG, cfg);
}

esp_err_t ina219_get_bus_voltage_V(ina219_handle_t handle, float *voltage_v)
{
    if (voltage_v == NULL) return ESP_ERR_INVALID_ARG;
    uint16_t raw;
    esp_err_t err = ina219_read_reg(handle, INA219_REG_BUS_VOLT, &raw);
    if (err == ESP_OK) {
        *voltage_v = (float)(raw >> 3) * INA219_BUS_VOLT_LSB; /* BD is bits 15..3 */
    }
    return err;
}

esp_err_t ina219_get_shunt_voltage_mV(ina219_handle_t handle, float *voltage_mv)
{
    if (voltage_mv == NULL) return ESP_ERR_INVALID_ARG;
    uint16_t raw;
    esp_err_t err = ina219_read_reg(handle, INA219_REG_SHUNT_VOLT, &raw);
    if (err == ESP_OK) {
        *voltage_mv = (float)(int16_t)raw * INA219_SHUNT_VOLT_LSB;
    }
    return err;
}

esp_err_t ina219_get_current_mA(ina219_handle_t handle, float *current_ma)
{
    if (current_ma == NULL) return ESP_ERR_INVALID_ARG;
    if (handle.current_lsb <= 0.0f) return ESP_ERR_INVALID_STATE; /* ina219_config() missing */
    uint16_t raw;
    esp_err_t err = ina219_read_reg(handle, INA219_REG_CURRENT, &raw);
    if (err == ESP_OK) {
        *current_ma = (float)(int16_t)raw * handle.current_lsb * 1000.0f;
    }
    return err;
}

esp_err_t ina219_get_power_mW(ina219_handle_t handle, float *power_mw)
{
    if (power_mw == NULL) return ESP_ERR_INVALID_ARG;
    if (handle.power_lsb <= 0.0f) return ESP_ERR_INVALID_STATE;
    uint16_t raw;
    esp_err_t err = ina219_read_reg(handle, INA219_REG_POWER, &raw);
    if (err == ESP_OK) {
        *power_mw = (float)raw * handle.power_lsb * 1000.0f;
    }
    return err;
}

esp_err_t ina219_get_status(ina219_handle_t handle, bool *conversion_ready, bool *overflow)
{
    uint16_t raw;
    esp_err_t err = ina219_read_reg(handle, INA219_REG_BUS_VOLT, &raw);
    if (err != ESP_OK) return err;
    if (conversion_ready) *conversion_ready = (raw & INA219_BUS_CNVR) != 0;
    if (overflow) *overflow = (raw & INA219_BUS_OVF) != 0;
    return ESP_OK;
}

esp_err_t ina219_read_all(ina219_handle_t handle, ina219_measurement_t *out)
{
    if (out == NULL) return ESP_ERR_INVALID_ARG;
    if (handle.current_lsb <= 0.0f) return ESP_ERR_INVALID_STATE;

    uint16_t bus, shunt, cur, pwr;
    esp_err_t err;
    /* Power is read last: reading it clears the CNVR flag. */
    if ((err = ina219_read_reg(handle, INA219_REG_BUS_VOLT, &bus)) != ESP_OK) return err;
    if ((err = ina219_read_reg(handle, INA219_REG_SHUNT_VOLT, &shunt)) != ESP_OK) return err;
    if ((err = ina219_read_reg(handle, INA219_REG_CURRENT, &cur)) != ESP_OK) return err;
    if ((err = ina219_read_reg(handle, INA219_REG_POWER, &pwr)) != ESP_OK) return err;

    out->bus_voltage_V = (float)(bus >> 3) * INA219_BUS_VOLT_LSB;
    out->shunt_voltage_mV = (float)(int16_t)shunt * INA219_SHUNT_VOLT_LSB;
    out->current_mA = (float)(int16_t)cur * handle.current_lsb * 1000.0f;
    out->power_mW = (float)pwr * handle.power_lsb * 1000.0f;
    out->overflow = (bus & INA219_BUS_OVF) != 0;
    return ESP_OK;
}

esp_err_t ina219_delete(ina219_handle_t handle)
{
    if (handle.dev_handle == NULL) return ESP_ERR_INVALID_ARG;
    return i2c_master_bus_rm_device(handle.dev_handle);
}
