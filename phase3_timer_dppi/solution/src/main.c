/*
 * Phase 3: TIMER + DPPI -> SAADC Autonomous Sampling
 *
 * Based on Nordic Academy Lesson 6, Exercise 3 (nRF54L15 variant).
 *
 * This implementation demonstrates fully autonomous ADC sampling using:
 *   TIMER22 COMPARE event -> DPPI channel -> SAADC SAMPLE task
 *   SAADC END event -> DPPI channel -> SAADC START task (continuous)
 *
 * Key advantage over GRTC approach: TIMER has hardware SHORTS
 * (COMPARE -> CLEAR) for automatic periodic operation. The CPU does NOT
 * need to wake up for each sample — only when a full buffer is ready.
 *
 * Architecture:
 *   TIMER22 CC[0] fires every 5ms (200 Hz)
 *         |
 *         +--SHORT: COMPARE0 -> CLEAR (hardware auto-reload, no CPU)
 *         |
 *         +--> DPPI Ch.1 --> SAADC SAMPLE task --> EasyDMA --> RAM Buffer
 *
 *   SAADC END event --> DPPI Ch.2 --> SAADC START task (continuous)
 *
 *   When buffer full (200 samples = 1 second):
 *         SAADC interrupt --> CPU wakes to process buffer
 *
 * CPU wake-ups: 1 per second (buffer full only)
 * vs. GRTC approach: 200 per second (re-arm each compare)
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

#include <nrfx_timer.h>
#include <nrfx_saadc.h>
#include <helpers/nrfx_gppi.h>

LOG_MODULE_REGISTER(phase3_timer_dppi, LOG_LEVEL_INF);

/* Sampling parameters */
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

/*
 * SAADC event handler
 *
 * This is the ONLY place the CPU wakes up during sampling.
 * Called once per second when a 200-sample buffer is full.
 */
static void saadc_handler(nrfx_saadc_evt_t const *p_event)
{
	static uint8_t buffer_index;

	switch (p_event->type) {
	case NRFX_SAADC_EVT_READY:
		/* Start the timer now that SAADC is confirmed ready.
		 * This ensures no DPPI triggers arrive before the SAADC
		 * can handle them.
		 */
		nrfx_timer_enable(&timer_instance);
		LOG_INF("SAADC ready, TIMER started — autonomous sampling active");
		break;

	case NRFX_SAADC_EVT_BUF_REQ:
		/* Provide next buffer for continuous sampling */
		nrfx_saadc_buffer_set(
			sample_buffers[(buffer_index++) % BUFFER_COUNT],
			BUFFER_SIZE);
		break;

	case NRFX_SAADC_EVT_DONE: {
		/* Buffer full — this is the 1x/sec CPU wake-up */
		buffer_full_count++;
		sample_count += p_event->data.done.size;

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

		/* Convert to millivolts: (Vref * Gain^-1 * sample) / 2^12
		 * nRF54L15: Vref=0.9V, Gain=1/4, so: (900 * 4 * sample) / 4096
		 */
		int32_t avg_mv = (900 * 4 * avg) / 4096;
		int32_t min_mv = (900 * 4 * min_val) / 4096;
		int32_t max_mv = (900 * 4 * max_val) / 4096;

		LOG_INF("Buffer %d: %d samples | AVG=%d (%d mV) MIN=%d (%d mV) MAX=%d (%d mV)",
			buffer_full_count, p_event->data.done.size,
			avg, avg_mv, min_val, min_mv, max_val, max_mv);
		break;
	}

	default:
		break;
	}
}

/*
 * Initialize TIMER22 for periodic 200 Hz events
 *
 * Uses hardware SHORTS (COMPARE0 -> CLEAR) so the timer auto-reloads
 * without any CPU involvement. No ISR needed.
 */
static int init_timer(void)
{
	int err;

	/* Connect TIMER22 interrupt (required by nrfx even if unused) */
	IRQ_CONNECT(DT_IRQN(DT_NODELABEL(timer22)),
		    DT_IRQ(DT_NODELABEL(timer22), priority),
		    nrfx_isr, nrfx_timer_22_irq_handler, 0);

	/* Configure timer: 1 MHz clock, 32-bit counter */
	nrfx_timer_config_t timer_config = NRFX_TIMER_DEFAULT_CONFIG(1000000);

	err = nrfx_timer_init(&timer_instance, &timer_config, NULL);
	if (err != 0) {
		LOG_ERR("TIMER init failed: %d", err);
		return -EIO;
	}

	/* Set CC[0] for 5ms interval with auto-clear SHORT */
	uint32_t ticks = nrfx_timer_us_to_ticks(&timer_instance,
						 SAMPLE_INTERVAL_US);

	nrfx_timer_extended_compare(&timer_instance,
				    NRF_TIMER_CC_CHANNEL0,
				    ticks,
				    NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
				    false);  /* No interrupt needed */

	LOG_INF("TIMER22 configured: %d us interval (%d ticks at 1 MHz)",
		SAMPLE_INTERVAL_US, ticks);
	LOG_INF("  SHORT: COMPARE0 -> CLEAR (hardware auto-reload)");

	return 0;
}

/*
 * Initialize SAADC for hardware-triggered sampling
 *
 * Identical to Phase 4 GRTC version — the SAADC doesn't care
 * whether the trigger comes from TIMER or GRTC via DPPI.
 */
static int init_saadc(void)
{
	int err;

	/* Connect SAADC interrupt */
	IRQ_CONNECT(DT_IRQN(DT_NODELABEL(adc)),
		    DT_IRQ(DT_NODELABEL(adc), priority),
		    nrfx_isr, nrfx_saadc_irq_handler, 0);

	err = nrfx_saadc_init(DT_IRQ(DT_NODELABEL(adc), priority));
	if (err != 0) {
		LOG_ERR("SAADC init failed: %d", err);
		return -EIO;
	}

	/* Configure channel: AIN4 (P1.11), 1/4 gain for 0-3.6V range */
	channel.channel_config.gain = NRF_SAADC_GAIN1_4;

	err = nrfx_saadc_channels_config(&channel, 1);
	if (err != 0) {
		LOG_ERR("SAADC channel config failed: %d", err);
		return -EIO;
	}

	/* Advanced mode: external trigger via DPPI, no internal timer */
	nrfx_saadc_adv_config_t adv_config = NRFX_SAADC_DEFAULT_ADV_CONFIG;
	adv_config.internal_timer_cc = 0;     /* External trigger (DPPI) */
	adv_config.start_on_end = false;      /* We use DPPI END->START instead */

	err = nrfx_saadc_advanced_mode_set(BIT(SAADC_CHANNEL),
					   NRF_SAADC_RESOLUTION_12BIT,
					   &adv_config,
					   saadc_handler);
	if (err != 0) {
		LOG_ERR("SAADC advanced mode failed: %d", err);
		return -EIO;
	}

	/* Set up double buffers */
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

	LOG_INF("SAADC initialized: 12-bit, AIN4 (P1.11), external trigger via DPPI");
	return 0;
}

/*
 * Set up DPPI connections:
 *   1. TIMER22 COMPARE[0] -> SAADC SAMPLE (trigger each sample)
 *   2. SAADC END -> SAADC START (continuous buffer operation)
 */
static int init_dppi(void)
{
	int err;

	/* Get event/task addresses */
	uint32_t timer_compare_addr = nrfx_timer_compare_event_address_get(
		&timer_instance, NRF_TIMER_CC_CHANNEL0);
	uint32_t saadc_sample_addr = nrf_saadc_task_address_get(
		NRF_SAADC, NRF_SAADC_TASK_SAMPLE);
	uint32_t saadc_end_addr = nrf_saadc_event_address_get(
		NRF_SAADC, NRF_SAADC_EVENT_END);
	uint32_t saadc_start_addr = nrf_saadc_task_address_get(
		NRF_SAADC, NRF_SAADC_TASK_START);

	LOG_INF("DPPI endpoints:");
	LOG_INF("  TIMER22 COMPARE[0]: 0x%08x", timer_compare_addr);
	LOG_INF("  SAADC SAMPLE:       0x%08x", saadc_sample_addr);
	LOG_INF("  SAADC END:          0x%08x", saadc_end_addr);
	LOG_INF("  SAADC START:        0x%08x", saadc_start_addr);

	/* Connection 1: TIMER COMPARE -> SAADC SAMPLE */
	err = nrfx_gppi_conn_alloc(timer_compare_addr, saadc_sample_addr,
				   &gppi_sample_handle);
	if (err != 0) {
		LOG_ERR("GPPI alloc (TIMER->SAADC SAMPLE) failed: %d", err);
		return -EIO;
	}
	nrfx_gppi_conn_enable(gppi_sample_handle);
	LOG_INF("DPPI: TIMER22 COMPARE[0] -> SAADC SAMPLE (handle %d)",
		gppi_sample_handle);

	/* Connection 2: SAADC END -> SAADC START (continuous operation) */
	err = nrfx_gppi_conn_alloc(saadc_end_addr, saadc_start_addr,
				   &gppi_start_handle);
	if (err != 0) {
		LOG_ERR("GPPI alloc (SAADC END->START) failed: %d", err);
		return -EIO;
	}
	nrfx_gppi_conn_enable(gppi_start_handle);
	LOG_INF("DPPI: SAADC END -> SAADC START (handle %d)",
		gppi_start_handle);

	return 0;
}

static void print_results(void)
{
	LOG_INF("==========================================");
	LOG_INF("PHASE 3 TEST RESULTS");
	LOG_INF("==========================================");
	LOG_INF("Total samples collected: %d", sample_count);
	LOG_INF("Buffers completed: %d", buffer_full_count);
	LOG_INF("Expected buffers (10 sec): 10");
	LOG_INF("");
	LOG_INF("CPU wake-ups for sampling: 0 (TIMER SHORTS = hardware auto-reload)");
	LOG_INF("CPU wake-ups for buffers:  %d (1 per second)", buffer_full_count);
	LOG_INF("");

	if (buffer_full_count >= 9 && buffer_full_count <= 11) {
		LOG_INF("PASS: Autonomous sampling working correctly");
	} else {
		LOG_WRN("WARNING: Expected ~10 buffers, got %d", buffer_full_count);
	}
}

int main(void)
{
	int err;

	LOG_INF("==========================================");
	LOG_INF("Phase 3: TIMER + DPPI -> SAADC");
	LOG_INF("(Based on Nordic Academy L6 Exercise 3)");
	LOG_INF("==========================================");
	LOG_INF("Sample rate: %d Hz (%d us interval)",
		1000000 / SAMPLE_INTERVAL_US, SAMPLE_INTERVAL_US);
	LOG_INF("Buffer: %d samples (%d ms per buffer)",
		BUFFER_SIZE, (BUFFER_SIZE * SAMPLE_INTERVAL_US) / 1000);
	LOG_INF("Double buffering: %d buffers", BUFFER_COUNT);
	LOG_INF("");

	/* Initialize peripherals in order */
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

	/* Start the autonomous sampling chain:
	 * 1. mode_trigger() puts the SAADC in ready state
	 * 2. EVT_READY fires in saadc_handler(), which starts the TIMER
	 * 3. From there, TIMER COMPARE events trigger samples via DPPI
	 *
	 * Starting the timer inside EVT_READY (instead of here) ensures
	 * no DPPI triggers arrive before the SAADC is ready to handle them.
	 */
	err = nrfx_saadc_mode_trigger();
	if (err != 0) {
		LOG_ERR("SAADC mode trigger failed: %d", err);
		return -EIO;
	}

	LOG_INF("");
	LOG_INF("Autonomous sampling started!");
	LOG_INF("CPU is now idle — hardware handles all sampling.");
	LOG_INF("Running test for 10 seconds...");
	LOG_INF("");

	/* CPU sleeps here. Wakes only 1x/sec for buffer processing. */
	k_msleep(10000);

	/* Stop the hardware chain */
	nrfx_timer_disable(&timer_instance);
	nrfx_gppi_conn_disable(gppi_sample_handle);
	nrfx_gppi_conn_disable(gppi_start_handle);
	nrfx_saadc_abort();

	print_results();

	LOG_INF("");
	LOG_INF("==========================================");
	LOG_INF("Phase 3 test complete!");
	LOG_INF("==========================================");

	while (1) {
		k_msleep(1000);
	}

	return 0;
}
