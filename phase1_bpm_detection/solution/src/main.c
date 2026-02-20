/*
 * Phase 1: Beat Detection using PulseSensor Algorithm
 *
 * Builds on Phase 0 by adding the PulseSensor beat detection algorithm.
 * Algorithm ported from: https://github.com/WorldFamousElectronics/PulseSensorPlayground
 *
 * Note: For reliable readings, use the velcro strap to maintain consistent
 * pressure and block ambient light. See the PulseSensor Getting Started Guide.
 *
 * Wiring: Same as Phase 0
 *   Pulse Sensor Signal (S) -> P1.11 (AIN4)
 *   Pulse Sensor VCC (+)    -> 3.3V
 *   Pulse Sensor GND (-)    -> GND
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>

#include "pulse_sensor.h"

LOG_MODULE_REGISTER(pulse, LOG_LEVEL_INF);

/* 500 Hz sampling required for beat detection algorithm */
#define SAMPLE_INTERVAL_MS  2

/* Initial threshold for 12-bit ADC (algorithm will auto-adapt) */
#define INITIAL_THRESHOLD   2200

#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
    !DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "Devicetree overlay required"
#endif

static const struct adc_dt_spec adc_channel = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);

/* PulseSensor state */
static pulse_sensor_t pulse;

int main(void)
{
	int err;
	uint16_t buf;
	struct adc_sequence sequence = {
		.buffer = &buf,
		.buffer_size = sizeof(buf),
	};

	LOG_INF("Phase 1: Beat Detection");

	if (!adc_is_ready_dt(&adc_channel)) {
		LOG_ERR("ADC device not ready");
		return -1;
	}

	err = adc_channel_setup_dt(&adc_channel);
	if (err < 0) {
		LOG_ERR("ADC channel setup failed (%d)", err);
		return err;
	}

	/* Initialize PulseSensor algorithm */
	pulse_sensor_init(&pulse, INITIAL_THRESHOLD, SAMPLE_INTERVAL_MS);

	LOG_INF("Place finger on sensor...");

	while (1) {
		adc_sequence_init_dt(&adc_channel, &sequence);

		err = adc_read_dt(&adc_channel, &sequence);
		if (err < 0) {
			k_msleep(SAMPLE_INTERVAL_MS);
			continue;
		}

		/* Process through beat detection algorithm */
		bool beat = pulse_sensor_process(&pulse, (int16_t)buf);

		if (beat) {
			LOG_INF("BPM: %d", pulse_sensor_get_bpm(&pulse));
		}

		k_msleep(SAMPLE_INTERVAL_MS);
	}

	return 0;
}
