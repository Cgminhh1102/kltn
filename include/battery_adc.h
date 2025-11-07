/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef BATTERY_ADC_H__
#define BATTERY_ADC_H__

#include <stdint.h>

/**
 * @brief Initialize ADC for battery voltage measurement
 * 
 * Configures ADC channel to read battery voltage through voltage divider.
 * For nRF54L15, typical setup:
 * - ADC input: AIN0 (P0.04) or VDD internal
 * - Reference: Internal 0.6V
 * - Gain: 1/6 for measuring up to 3.6V
 * - Resolution: 12-bit
 * 
 * @return 0 on success, negative errno on failure
 */
int battery_adc_init(void);

/**
 * @brief Read battery voltage in millivolts
 * 
 * Performs ADC conversion and returns battery voltage.
 * Typical range: 1800 mV (empty) to 3300 mV (full) for Li-Ion/LiPo
 * 
 * @param[out] voltage_mv Battery voltage in millivolts
 * 
 * @return 0 on success, negative errno on failure
 */
int battery_adc_read_mv(uint16_t *voltage_mv);

/**
 * @brief Get battery percentage from voltage
 * 
 * Converts battery voltage to percentage using linear approximation:
 * - 3.3V = 100%
 * - 3.0V = 50%
 * - 2.7V = 20%
 * - 2.4V = 5%
 * - <2.4V = 0%
 * 
 * @param voltage_mv Battery voltage in millivolts
 * 
 * @return Battery percentage (0-100)
 */
uint8_t battery_voltage_to_percent(uint16_t voltage_mv);

/**
 * @brief Get battery percentage directly
 * 
 * Reads ADC and converts to percentage in one call.
 * Returns 0 if ADC read fails.
 * 
 * @return Battery percentage (0-100)
 */
uint8_t battery_get_percentage(void);

#endif /* BATTERY_ADC_H__ */
