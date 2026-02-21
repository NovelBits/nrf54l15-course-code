/*
 * Phase 5: Bluetooth LE Heart Rate Monitor
 *
 * Combines three previous phases into a wireless heart rate monitor:
 *   - Phase 3: TIMER22 + DPPI -> SAADC autonomous sampling (200 Hz)
 *   - Phase 1: PulseSensor BPM detection algorithm
 *   - BLE: Heart Rate Service (0x180D) notifications
 *
 * Architecture:
 *   TIMER22 CC[0] fires every 5ms (200 Hz)
 *         |
 *         +--SHORT: COMPARE0 -> CLEAR (hardware auto-reload, no CPU)
 *         |
 *         +--> DPPI --> SAADC SAMPLE --> EasyDMA --> RAM Buffer[200]
 *
 *   When buffer full (200 samples = 1 second):
 *         SAADC interrupt --> CPU wakes
 *              |
 *              +--> pulse_sensor_process() x200 samples
 *              |
 *              +--> beat detected? --> bt_hrs_notify(bpm)
 *              |
 *              +--> nRF Connect Mobile shows BPM
 *
 * Wiring:
 *   Pulse Sensor Signal (S) -> P1.11 (AIN4)
 *   Pulse Sensor VCC (+)    -> 3.3V
 *   Pulse Sensor GND (-)    -> GND
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/devicetree.h>
#include <stdlib.h>

/* nrfx drivers (Phase 3) */
#include <nrfx_timer.h>
#include <nrfx_saadc.h>
#include <helpers/nrfx_gppi.h>

/* Bluetooth LE */
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/hrs.h>

/* BPM detection algorithm */
#include "pulse_sensor.h"

LOG_MODULE_REGISTER(phase5_ble_hrs, LOG_LEVEL_INF);

/* ──────────────────────────────────────────────
 * Sampling parameters (same as Phase 3)
 * ────────────────────────────────────────────── */
#define SAMPLE_INTERVAL_US      5000    /* 5ms = 200 Hz sampling */
#define BUFFER_SIZE             200     /* 1 second of samples at 200 Hz */
#define BUFFER_COUNT            2       /* Double buffering */

/* TIMER configuration - nRF54L15 uses TIMER22 */
#define TIMER_INST_IDX          22
static nrfx_timer_t timer_instance = NRFX_TIMER_INSTANCE(NRF_TIMER22);

/* Define TIMER22 IRQ handler (required by nrfx 4.x) */
NRFX_INSTANCE_IRQ_HANDLER_DEFINE(timer, TIMER_INST_IDX, &timer_instance);

/* SAADC channel configuration */
#define SAADC_CHANNEL           0

/* nRF54L15 uses NRFX_ANALOG_EXTERNAL_AIN* for analog inputs */
#if NRF_SAADC_HAS_AIN_AS_PIN
#define SAADC_INPUT_PIN         NRFX_ANALOG_EXTERNAL_AIN4  /* P1.11 on nRF54L15 DK */
#else
#define SAADC_INPUT_PIN         NRF_SAADC_INPUT_AIN4
#endif

/* DPPI handles */
static nrfx_gppi_handle_t gppi_sample_handle;
static nrfx_gppi_handle_t gppi_start_handle;

/* Statistics */
static volatile uint32_t buffer_full_count;
static volatile uint32_t sample_count;

/* Sample buffers (must be in RAM for EasyDMA) */
static int16_t sample_buffers[BUFFER_COUNT][BUFFER_SIZE];

/* SAADC channel configuration struct */
static nrfx_saadc_channel_t channel = NRFX_SAADC_DEFAULT_CHANNEL_SE(
	SAADC_INPUT_PIN, SAADC_CHANNEL);

/* ──────────────────────────────────────────────
 * BPM detection (Phase 1 algorithm)
 * ────────────────────────────────────────────── */
static pulse_sensor_t pulse;

/* ──────────────────────────────────────────────
 * Bluetooth LE
 * ────────────────────────────────────────────── */

/* Advertising data - includes Heart Rate Service UUID */
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_HRS_VAL)),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
		sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* Work queue item for restarting advertising after disconnect */
static struct k_work adv_restart_work;

static void adv_restart_handler(struct k_work *work)
{
	int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad),
				  sd, ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Failed to restart advertising (err %d)", err);
	} else {
		LOG_INF("Advertising restarted");
	}
}

/* ────────────────────────────────────────────────────────────
 * TODO 2: Define connection callbacks
 *
 * Use the BT_CONN_CB_DEFINE macro to register callbacks for:
 *   .connected    - Log the connection event
 *   .disconnected - Log the disconnect reason
 *   .recycled     - Submit adv_restart_work to restart advertising
 *
 * The .recycled callback is needed because NCS 3.0+ automatically
 * stops advertising when a connection is established. After
 * disconnect, .recycled fires when the connection object is
 * released — that's when it's safe to restart advertising.
 *
 * Hint: Use k_work_submit(&adv_restart_work) in the recycled callback.
 * ──────────────────────────────────────────────────────────── */


/* ────────────────────────────────────────────────────────────
 * TODO 1: Initialize Bluetooth
 *
 * Implement init_bluetooth() to:
 *   1. Initialize the work queue: k_work_init(&adv_restart_work, adv_restart_handler)
 *   2. Enable Bluetooth: bt_enable(NULL)
 *   3. Start advertising: bt_le_adv_start(BT_LE_ADV_CONN_FAST_1,
 *                                         ad, ARRAY_SIZE(ad),
 *                                         sd, ARRAY_SIZE(sd))
 *   4. Return 0 on success, or the error code on failure
 *
 * Remember to log the result of each step for debugging.
 * ──────────────────────────────────────────────────────────── */
static int init_bluetooth(void)
{
	/* Your implementation here */

	return 0;
}

/* ──────────────────────────────────────────────
 * SAADC handler (Phase 3 base + BPM processing)
 * ────────────────────────────────────────────── */

static void saadc_handler(nrfx_saadc_evt_t const *p_event)
{
	static uint8_t buffer_index;

	switch (p_event->type) {
	case NRFX_SAADC_EVT_READY:
		nrfx_timer_enable(&timer_instance);
		LOG_INF("SAADC ready, TIMER started — autonomous sampling active");
		break;

	case NRFX_SAADC_EVT_BUF_REQ:
		nrfx_saadc_buffer_set(
			sample_buffers[(buffer_index++) % BUFFER_COUNT],
			BUFFER_SIZE);
		break;

	case NRFX_SAADC_EVT_DONE: {
		buffer_full_count++;
		sample_count += p_event->data.done.size;

		/* ──────────────────────────────────────────────────────
		 * TODO 3: Process samples for BPM detection
		 *
		 * Loop through all samples in the buffer and feed each
		 * one to the BPM detection algorithm:
		 *
		 *   for each sample in p_event->data.done.p_buffer:
		 *     1. Call pulse_sensor_process(&pulse, sample)
		 *     2. If it returns true (beat detected):
		 *        a. Get BPM: pulse_sensor_get_bpm(&pulse)
		 *        b. If BPM > 0, log it and send via BLE:
		 *           bt_hrs_notify(bpm)
		 *
		 * The buffer has p_event->data.done.size samples.
		 * Each sample is an int16_t (12-bit ADC value).
		 * ────────────────────────────────────────────────────── */


		/* Log buffer stats for debugging */
		int32_t min_val = INT16_MAX;
		int32_t max_val = INT16_MIN;
		int64_t sum = 0;

		for (uint16_t i = 0; i < p_event->data.done.size; i++) {
			int16_t val = p_event->data.done.p_buffer[i];
			if (val < min_val) {
				min_val = val;
			}
			if (val > max_val) {
				max_val = val;
			}
			sum += val;
		}
		int32_t avg = sum / p_event->data.done.size;
		int32_t avg_mv = (900 * 4 * avg) / 4096;

		LOG_INF("Buffer %d: %d samples | AVG=%d (%d mV) | BPM=%d",
			buffer_full_count, p_event->data.done.size,
			avg, avg_mv, pulse_sensor_get_bpm(&pulse));
		break;
	}

	default:
		break;
	}
}

/* ──────────────────────────────────────────────
 * Hardware initialization (same as Phase 3)
 * ────────────────────────────────────────────── */

static int init_timer(void)
{
	int err;

	IRQ_CONNECT(DT_IRQN(DT_NODELABEL(timer22)),
		    DT_IRQ(DT_NODELABEL(timer22), priority),
		    nrfx_isr, nrfx_timer_22_irq_handler, 0);

	nrfx_timer_config_t timer_config = NRFX_TIMER_DEFAULT_CONFIG(1000000);

	err = nrfx_timer_init(&timer_instance, &timer_config, NULL);
	if (err != 0) {
		LOG_ERR("TIMER init failed: %d", err);
		return -EIO;
	}

	uint32_t ticks = nrfx_timer_us_to_ticks(&timer_instance,
						 SAMPLE_INTERVAL_US);

	nrfx_timer_extended_compare(&timer_instance,
				    NRF_TIMER_CC_CHANNEL0,
				    ticks,
				    NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
				    false);

	LOG_INF("TIMER22 configured: %d Hz (%d us interval)",
		1000000 / SAMPLE_INTERVAL_US, SAMPLE_INTERVAL_US);

	return 0;
}

static int init_saadc(void)
{
	int err;

	IRQ_CONNECT(DT_IRQN(DT_NODELABEL(adc)),
		    DT_IRQ(DT_NODELABEL(adc), priority),
		    nrfx_isr, nrfx_saadc_irq_handler, 0);

	err = nrfx_saadc_init(DT_IRQ(DT_NODELABEL(adc), priority));
	if (err != 0) {
		LOG_ERR("SAADC init failed: %d", err);
		return -EIO;
	}

	channel.channel_config.gain = NRF_SAADC_GAIN1_4;

	err = nrfx_saadc_channels_config(&channel, 1);
	if (err != 0) {
		LOG_ERR("SAADC channel config failed: %d", err);
		return -EIO;
	}

	nrfx_saadc_adv_config_t adv_config = NRFX_SAADC_DEFAULT_ADV_CONFIG;
	adv_config.internal_timer_cc = 0;
	adv_config.start_on_end = false;

	err = nrfx_saadc_advanced_mode_set(BIT(SAADC_CHANNEL),
					   NRF_SAADC_RESOLUTION_12BIT,
					   &adv_config,
					   saadc_handler);
	if (err != 0) {
		LOG_ERR("SAADC advanced mode failed: %d", err);
		return -EIO;
	}

	err = nrfx_saadc_buffer_set(sample_buffers[0], BUFFER_SIZE);
	if (err != 0) {
		LOG_ERR("SAADC buffer[0] set failed: %d", err);
		return -EIO;
	}

	err = nrfx_saadc_buffer_set(sample_buffers[1], BUFFER_SIZE);
	if (err != 0) {
		LOG_ERR("SAADC buffer[1] set failed: %d", err);
		return -EIO;
	}

	LOG_INF("SAADC initialized: 12-bit, AIN4 (P1.11), external trigger");
	return 0;
}

static int init_dppi(void)
{
	int err;

	uint32_t timer_compare_addr = nrfx_timer_compare_event_address_get(
		&timer_instance, NRF_TIMER_CC_CHANNEL0);
	uint32_t saadc_sample_addr = nrf_saadc_task_address_get(
		NRF_SAADC, NRF_SAADC_TASK_SAMPLE);
	uint32_t saadc_end_addr = nrf_saadc_event_address_get(
		NRF_SAADC, NRF_SAADC_EVENT_END);
	uint32_t saadc_start_addr = nrf_saadc_task_address_get(
		NRF_SAADC, NRF_SAADC_TASK_START);

	/* Connection 1: TIMER COMPARE -> SAADC SAMPLE */
	err = nrfx_gppi_conn_alloc(timer_compare_addr, saadc_sample_addr,
				   &gppi_sample_handle);
	if (err != 0) {
		LOG_ERR("GPPI alloc (TIMER->SAADC SAMPLE) failed: %d", err);
		return -EIO;
	}
	nrfx_gppi_conn_enable(gppi_sample_handle);

	/* Connection 2: SAADC END -> SAADC START (continuous) */
	err = nrfx_gppi_conn_alloc(saadc_end_addr, saadc_start_addr,
				   &gppi_start_handle);
	if (err != 0) {
		LOG_ERR("GPPI alloc (SAADC END->START) failed: %d", err);
		return -EIO;
	}
	nrfx_gppi_conn_enable(gppi_start_handle);

	LOG_INF("DPPI configured: TIMER22->SAMPLE, END->START");
	return 0;
}

/* ──────────────────────────────────────────────
 * Main
 * ────────────────────────────────────────────── */

int main(void)
{
	int err;

	LOG_INF("==========================================");
	LOG_INF("Phase 5: Bluetooth LE Heart Rate Monitor");
	LOG_INF("==========================================");
	LOG_INF("Sample rate: %d Hz (%d us interval)",
		1000000 / SAMPLE_INTERVAL_US, SAMPLE_INTERVAL_US);
	LOG_INF("Buffer: %d samples (%d ms per buffer)",
		BUFFER_SIZE, (BUFFER_SIZE * SAMPLE_INTERVAL_US) / 1000);
	LOG_INF("");

	/* ────────────────────────────────────────────────────────
	 * TODO 4: Initialize pulse sensor and Bluetooth
	 *
	 * Add two initialization calls before the hardware init:
	 *
	 *   1. Initialize the BPM detection algorithm:
	 *      pulse_sensor_init(&pulse, 2200, 5)
	 *        - 2200: threshold for 12-bit ADC (0-4095 range)
	 *        - 5: sample interval in ms (200 Hz = 5ms between samples)
	 *
	 *   2. Initialize Bluetooth (call init_bluetooth())
	 *      - Check for errors and return if it fails
	 *
	 * These must be called BEFORE init_timer/init_saadc/init_dppi.
	 * ──────────────────────────────────────────────────────── */


	/* Initialize sampling hardware (same as Phase 3) */
	err = init_timer();
	if (err) {
		LOG_ERR("TIMER init failed");
		return err;
	}

	err = init_saadc();
	if (err) {
		LOG_ERR("SAADC init failed");
		return err;
	}

	err = init_dppi();
	if (err) {
		LOG_ERR("DPPI init failed");
		return err;
	}

	/* Start the autonomous sampling chain */
	err = nrfx_saadc_mode_trigger();
	if (err != 0) {
		LOG_ERR("SAADC mode trigger failed: %d", err);
		return -EIO;
	}

	LOG_INF("");
	LOG_INF("Heart rate monitor running!");
	LOG_INF("Place finger on sensor and connect with nRF Connect Mobile");
	LOG_INF("");

	/* Main loop — CPU sleeps between buffer interrupts.
	 * BPM processing and BLE notifications happen in saadc_handler.
	 */
	while (1) {
		k_msleep(1000);
	}

	return 0;
}
