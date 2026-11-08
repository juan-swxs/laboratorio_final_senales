/*
 * INA219 ESP-IDF Driver (i2c_master) - revised fork
 *
 * Original work: Copyright (C) 2025 TNY Robotics, MIT License
 *   https://github.com/tny-robotics/ina219-esp-idf
 * Modifications: see CHANGES.md. Distributed under the same MIT License.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <esp_err.h>
#include <driver/i2c_master.h>

#ifdef __cplusplus
extern "C" {
#endif

#define INA219_DEFAULT_ADDR   0x40     /**< A0 and A1 to GND */
#define INA219_DEFAULT_CLOCK  100000   /**< Hz. The INA219 supports up to 400 kHz fast mode (and HS mode) */

/** @brief Bus voltage range options */
typedef enum {
    INA219_BUS_VOLTAGE_RANGE_16V = 0x00, /**< 0-16V Range */
    INA219_BUS_VOLTAGE_RANGE_32V = 0x01  /**< 0-32V Range */
} ina219_bus_voltage_range_t;

/** @brief PGA gain and shunt voltage range */
typedef enum {
    INA219_GAIN_1_40MV  = 0x00, /**< Gain 1, +/- 40mV Range */
    INA219_GAIN_2_80MV  = 0x01, /**< Gain 2, +/- 80mV Range */
    INA219_GAIN_4_160MV = 0x02, /**< Gain 4, +/- 160mV Range */
    INA219_GAIN_8_320MV = 0x03  /**< Gain 8, +/- 320mV Range */
} ina219_gain_t;

/** @brief ADC resolution and hardware averaging settings */
typedef enum {
    INA219_ADC_9BIT_1SAMP   = 0x00, /**< 9-bit, 1 sample (84us) */
    INA219_ADC_10BIT_1SAMP  = 0x01, /**< 10-bit, 1 sample (148us) */
    INA219_ADC_11BIT_1SAMP  = 0x02, /**< 11-bit, 1 sample (276us) */
    INA219_ADC_12BIT_1SAMP  = 0x03, /**< 12-bit, 1 sample (532us) */
    INA219_ADC_12BIT_2SAMP  = 0x09, /**< 12-bit, 2 samples averaged (1.06ms) */
    INA219_ADC_12BIT_4SAMP  = 0x0A, /**< 12-bit, 4 samples averaged (2.13ms) */
    INA219_ADC_12BIT_8SAMP  = 0x0B, /**< 12-bit, 8 samples averaged (4.26ms) */
    INA219_ADC_12BIT_16SAMP = 0x0C, /**< 12-bit, 16 samples averaged (8.51ms) */
    INA219_ADC_12BIT_32SAMP = 0x0D, /**< 12-bit, 32 samples averaged (17.02ms) */
    INA219_ADC_12BIT_64SAMP = 0x0E, /**< 12-bit, 64 samples averaged (34.05ms) */
    INA219_ADC_12BIT_128SAMP= 0x0F  /**< 12-bit, 128 samples averaged (68.10ms) */
} ina219_adc_res_avg_t;

/** @brief Operating modes */
typedef enum {
    INA219_MODE_POWER_DOWN     = 0x00,
    INA219_MODE_SHUNT_TRIG     = 0x01,
    INA219_MODE_BUS_TRIG       = 0x02,
    INA219_MODE_SHUNT_BUS_TRIG = 0x03,
    INA219_MODE_ADC_DISABLE    = 0x04,
    INA219_MODE_SHUNT_CONT     = 0x05,
    INA219_MODE_BUS_CONT       = 0x06,
    INA219_MODE_SHUNT_BUS_CONT = 0x07  /**< Default continuous mode */
} ina219_mode_t;

/** @brief I2C address and clock of the device. */
typedef struct {
    uint16_t address;      /**< 7-bit I2C address (0x40..0x4F) */
    uint32_t clock_speed;  /**< I2C clock in Hz (0 = INA219_DEFAULT_CLOCK) */
} ina219_info_t;

/** @brief Device configuration. */
typedef struct {
    ina219_bus_voltage_range_t bus_voltage_range;
    ina219_gain_t gain;
    ina219_adc_res_avg_t badc_res;   /**< Bus ADC resolution/averaging */
    ina219_adc_res_avg_t sadc_res;   /**< Shunt ADC resolution/averaging */
    ina219_mode_t mode;
    float shunt_resistor_ohms;       /**< Shunt resistor value in ohms (e.g. 0.1) */
    float max_expected_current_A;    /**< Maximum expected current in amps (e.g. 3.2) */
} ina219_config_t;

/** @brief One consistent snapshot of all measurements. */
typedef struct {
    float bus_voltage_V;
    float shunt_voltage_mV;
    float current_mA;
    float power_mW;
    bool  overflow;   /**< Math overflow: current and power values are NOT valid */
} ina219_measurement_t;

#ifdef __cplusplus
#define INA219_DEFAULT_INFO() ina219_info_t{INA219_DEFAULT_ADDR, INA219_DEFAULT_CLOCK}
#define INA219_DEFAULT_CONFIG() ina219_config_t{ \
    INA219_BUS_VOLTAGE_RANGE_32V, \
    INA219_GAIN_8_320MV, \
    INA219_ADC_12BIT_128SAMP, \
    INA219_ADC_12BIT_128SAMP, \
    INA219_MODE_SHUNT_BUS_CONT, \
    0.1f, \
    3.2f \
}
#else
#define INA219_DEFAULT_INFO() ((ina219_info_t){\
    .address = INA219_DEFAULT_ADDR,\
    .clock_speed = INA219_DEFAULT_CLOCK\
})
#define INA219_DEFAULT_CONFIG() ((ina219_config_t){\
    .bus_voltage_range = INA219_BUS_VOLTAGE_RANGE_32V,\
    .gain = INA219_GAIN_8_320MV,\
    .badc_res = INA219_ADC_12BIT_128SAMP,\
    .sadc_res = INA219_ADC_12BIT_128SAMP,\
    .mode = INA219_MODE_SHUNT_BUS_CONT,\
    .shunt_resistor_ohms = 0.1f,\
    .max_expected_current_A = 3.2f\
})
#endif

/** @brief Device handle. Always create it with ina219_create(), which zero-initialises it. */
typedef struct {
    i2c_master_dev_handle_t dev_handle; /**< I2C master device handle */
    float current_lsb;                  /**< Amps per bit of the current register (0 = not configured) */
    float power_lsb;                    /**< Watts per bit of the power register */
} ina219_handle_t;

/**
 * @brief Add the INA219 to an existing I2C master bus.
 * @return ESP_OK, ESP_ERR_INVALID_ARG (NULL args or address outside 0x40..0x4F), or the
 *         error of i2c_master_bus_add_device().
 */
esp_err_t ina219_create(i2c_master_bus_handle_t i2c_bus_handle, ina219_info_t info, ina219_handle_t *out_handle);

/**
 * @brief Calibrate and configure the device.
 *
 * Validates the settings before touching the hardware:
 *  - shunt resistor and max current must be > 0
 *  - max_current * shunt_resistor must fit in the selected PGA range (else ESP_ERR_INVALID_ARG;
 *    use ina219_pick_gain() to choose the gain)
 *  - the resulting calibration value must fit in the register
 *
 * The handle is only updated on success. Must be called again after ina219_reset().
 */
esp_err_t ina219_config(ina219_handle_t *handle, ina219_config_t config);

/** @brief Software reset (all registers back to default, calibration is lost: call ina219_config() again). */
esp_err_t ina219_reset(ina219_handle_t handle);

/** @brief Power-down mode. Not atomic against other tasks configuring the same device. */
esp_err_t ina219_sleep(ina219_handle_t handle);

/** @brief Return to the given mode after ina219_sleep(). */
#ifdef __cplusplus
esp_err_t ina219_wake_up(ina219_handle_t handle, ina219_mode_t mode = INA219_MODE_SHUNT_BUS_CONT);
#else
esp_err_t ina219_wake_up(ina219_handle_t handle, ina219_mode_t mode);
#endif

/**
 * @brief Smallest PGA gain that covers max_current_A * shunt_ohms (best resolution).
 * @return ESP_OK or ESP_ERR_INVALID_ARG if even 320 mV is not enough (use a smaller shunt).
 */
esp_err_t ina219_pick_gain(float shunt_ohms, float max_current_A, ina219_gain_t *out_gain);

/** @brief Bus voltage in volts. */
esp_err_t ina219_get_bus_voltage_V(ina219_handle_t handle, float *voltage_v);

/** @brief Shunt voltage in millivolts (signed). */
esp_err_t ina219_get_shunt_voltage_mV(ina219_handle_t handle, float *voltage_mv);

/** @brief Current in mA (signed). ESP_ERR_INVALID_STATE if ina219_config() was not called. Does not check overflow. */
esp_err_t ina219_get_current_mA(ina219_handle_t handle, float *current_ma);

/** @brief Power in mW. ESP_ERR_INVALID_STATE if ina219_config() was not called. Does not check overflow. */
esp_err_t ina219_get_power_mW(ina219_handle_t handle, float *power_mw);

/**
 * @brief Read the CNVR (conversion ready) and OVF (math overflow) flags.
 * Either output pointer may be NULL.
 */
esp_err_t ina219_get_status(ina219_handle_t handle, bool *conversion_ready, bool *overflow);

/**
 * @brief Read bus voltage, shunt voltage, current and power in one call, including the overflow flag.
 * Preferred over the individual getters: it is shorter and reports overflow.
 */
esp_err_t ina219_read_all(ina219_handle_t handle, ina219_measurement_t *out);

/** @brief Remove the device from the I2C bus. */
esp_err_t ina219_delete(ina219_handle_t handle);

#ifdef __cplusplus
}
#endif
