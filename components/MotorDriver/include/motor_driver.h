#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize GPIO and LEDC (PWM) peripherals used to drive
 *        two DC motors through a DRV8833 driver.
 *
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t motor_driver_init(void);

/**
 * @brief Set the right wheel's direction and speed directly.
 *
 * This is the function the PID loop should call to correct the
 * right wheel's speed while driving forward/backward.
 *
 * @param signed_speed_percent Signed speed from -100 to 100.
 *        Positive = forward, negative = backward, 0 = stop.
 */
esp_err_t motor_right_set(int8_t signed_speed_percent);

/**
 * @brief Set the left wheel's direction and speed directly.
 *
 * This is the function the PID loop should call to correct the
 * left wheel's speed while driving forward/backward.
 *
 * @param signed_speed_percent Signed speed from -100 to 100.
 *        Positive = forward, negative = backward, 0 = stop.
 */
esp_err_t motor_left_set(int8_t signed_speed_percent);

/**
 * @brief Turn the car right in place (left wheel forward,
 *        right wheel backward).
 *
 * @param speed_percent Speed from 0 to 100.
 */
esp_err_t car_turn_right(uint8_t speed_percent);

/**
 * @brief Turn the car left in place (right wheel forward,
 *        left wheel backward).
 *
 * @param speed_percent Speed from 0 to 100.
 */
esp_err_t car_turn_left(uint8_t speed_percent);

/**
 * @brief Stop both motors (coast, PWM duty set to 0).
 */
esp_err_t car_stop(void);

/**
 * @brief Read the DRV8833 nFAULT pin (ULT on the module).
 *
 * @return true if the driver reports a fault (overcurrent, thermal
 *         shutdown or undervoltage), false otherwise.
 */
bool motor_driver_fault(void);

#ifdef __cplusplus
}
#endif

#endif