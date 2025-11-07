/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "battery_adc.h"
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(battery_adc, LOG_LEVEL_DBG);

// ADC configuration for nRF54L15
#define ADC_DEVICE_NODE DT_NODELABEL(adc)
#define ADC_RESOLUTION 14
#define ADC_GAIN ADC_GAIN_1_4
#define ADC_REFERENCE ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME_DEFAULT
#define ADC_CHANNEL_ID 0

// Pin selection based on device tree visualization:
// ADC channels map to PORT 2 (P1.xx):
// - AIN1 → P1.05 (recommended for battery)
// NOTE: Pin configuration is done in device tree overlay file.
// See: boards/nrf54l15dk_nrf54l15_cpuapp.overlay

static const struct device *adc_dev;
static struct adc_channel_cfg channel_cfg = {
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id = ADC_CHANNEL_ID,
	// Note: input_positive is configured in device tree overlay
};

static int16_t sample_buffer;
static struct adc_sequence sequence = {
	.buffer = &sample_buffer,
	.buffer_size = sizeof(sample_buffer),
	.resolution = ADC_RESOLUTION,
};

int battery_adc_init(void)
{
	int err;

	adc_dev = DEVICE_DT_GET(ADC_DEVICE_NODE);
	if (!device_is_ready(adc_dev)) {
		LOG_ERR("ADC device not ready");
		return -ENODEV;
	}

	err = adc_channel_setup(adc_dev, &channel_cfg);
	if (err) {
		LOG_ERR("ADC channel setup failed: %d", err);
		return err;
	}

	sequence.channels = BIT(ADC_CHANNEL_ID);

	LOG_INF("Battery ADC initialized (channel %d, gain 1/6, ref internal)",
		ADC_CHANNEL_ID);

	return 0;
}

int battery_adc_read_mv(uint16_t *voltage_mv)
{
	int err;

	if (!adc_dev) {
		LOG_ERR("ADC not initialized");
		return -ENODEV;
	}

	err = adc_read(adc_dev, &sequence);
	if (err) {
		LOG_ERR("ADC read failed: %d", err);
		return err;
	}

	// Convert ADC value to millivolts
	// Reference: 0.6V internal
	// Gain: 1/6 (can measure up to 3.6V)
	// Resolution: 12-bit (0-4095)
	
	int32_t val_mv = sample_buffer;
	
	// Formula: V = (ADC_value * V_ref * gain_multiplier) / ADC_max
	// For gain 1/6: actual input = measured * 6
	// V_ref = 600 mV (internal reference)
	val_mv = (val_mv * 600 * 6) / 4095;

	if (val_mv < 0) {
		val_mv = 0;
	} else if (val_mv > 5000) {
		// Sanity check (max realistic battery voltage)
		val_mv = 5000;
	}

	*voltage_mv = (uint16_t)val_mv;

	LOG_DBG("ADC raw: %d, Voltage: %u mV", sample_buffer, *voltage_mv);

	return 0;
}

uint8_t battery_voltage_to_percent(uint16_t voltage_mv)
{
	// Li-Ion/LiPo voltage curve (simplified linear segments)
	// Full charge: 4.2V → 100%
	// Nominal: 3.7V → 80%
	// Low: 3.3V → 20%
	// Critical: 3.0V → 5%
	// Dead: <2.7V → 0%

	if (voltage_mv >= 4200) {
		return 100;
	} else if (voltage_mv >= 3700) {
		// 3.7V - 4.2V: 80% - 100% (linear)
		return 80 + ((voltage_mv - 3700) * 20) / 500;
	} else if (voltage_mv >= 3300) {
		// 3.3V - 3.7V: 20% - 80% (linear)
		return 20 + ((voltage_mv - 3300) * 60) / 400;
	} else if (voltage_mv >= 3000) {
		// 3.0V - 3.3V: 5% - 20% (linear)
		return 5 + ((voltage_mv - 3000) * 15) / 300;
	} else if (voltage_mv >= 2700) {
		// 2.7V - 3.0V: 0% - 5% (linear)
		return ((voltage_mv - 2700) * 5) / 300;
	} else {
		return 0;
	}
}

uint8_t battery_get_percentage(void)
{
	uint16_t voltage_mv;
	int err;

	err = battery_adc_read_mv(&voltage_mv);
	if (err) {
		LOG_WRN("Failed to read battery voltage, using default 85%%");
		return 85;  // Fallback value
	}

	uint8_t percent = battery_voltage_to_percent(voltage_mv);
	
	LOG_DBG("Battery: %u mV = %u%%", voltage_mv, percent);

	return percent;
}
