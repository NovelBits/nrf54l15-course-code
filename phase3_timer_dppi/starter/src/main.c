/*
 * Phase 3: TIMER + DPPI -> SAADC Autonomous Sampling - STARTER
 *
 * Goal: Eliminate CPU involvement in sampling by connecting TIMER22
 * to the SAADC through DPPI. The CPU only wakes when a full buffer
 * of 200 samples is ready (once per second).
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
#define TIMER_INSTANCE_NUMBER   22
static nrfx_timer_t timer_instance = NRFX_TIMER_INSTANCE(TIMER_INSTANCE_NUMBER);

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
 * SAADC event handler (provided as scaffolding)
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
 */
static int init_timer(void)
{
	/* TODO 1: Initialize TIMER22
	 *
	 * a) Connect the TIMER22 interrupt using IRQ_CONNECT with nrfx_isr wrapper:
	 *    IRQ_CONNECT(DT_IRQN(DT_NODELABEL(timer22)),
	 *                DT_IRQ(DT_NODELABEL(timer22), priority),
	 *                nrfx_isr, nrfx_timer_22_irq_handler, 0);
	 *
	 * b) Create a timer config: NRFX_TIMER_DEFAULT_CONFIG(1000000) for 1 MHz clock
	 *
	 * c) Initialize with nrfx_timer_init(&timer_instance, &config, NULL)
	 *
	 * d) Calculate ticks: nrfx_timer_us_to_ticks(&timer_instance, SAMPLE_INTERVAL_US)
	 *
	 * e) Set up periodic compare with auto-clear SHORT:
	 *    nrfx_timer_extended_compare(&timer_instance,
	 *                                NRF_TIMER_CC_CHANNEL0,
	 *                                ticks,
	 *                                NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
	 *                                false);
	 */

	return 0;
}

/*
 * Initialize SAADC for hardware-triggered sampling
 */
static int init_saadc(void)
{
	/* TODO 2: Initialize SAADC in advanced mode with double buffering
	 *
	 * a) Connect the SAADC interrupt using IRQ_CONNECT with nrfx_isr wrapper:
	 *    IRQ_CONNECT(DT_IRQN(DT_NODELABEL(adc)),
	 *                DT_IRQ(DT_NODELABEL(adc), priority),
	 *                nrfx_isr, nrfx_saadc_irq_handler, 0);
	 *
	 * b) Initialize: nrfx_saadc_init(DT_IRQ(DT_NODELABEL(adc), priority))
	 *
	 * c) Set gain: channel.channel_config.gain = NRF_SAADC_GAIN1_4
	 *    Then configure: nrfx_saadc_channels_config(&channel, 1)
	 *
	 * d) Set up advanced mode (external trigger via DPPI):
	 *    nrfx_saadc_adv_config_t adv_config = NRFX_SAADC_DEFAULT_ADV_CONFIG;
	 *    adv_config.internal_timer_cc = 0;    // External trigger (DPPI)
	 *    adv_config.start_on_end = false;     // We use DPPI END->START instead
	 *    nrfx_saadc_advanced_mode_set(BIT(SAADC_CHANNEL),
	 *                                 NRF_SAADC_RESOLUTION_12BIT,
	 *                                 &adv_config, saadc_handler);
	 *
	 * e) Set up double buffers:
	 *    nrfx_saadc_buffer_set(sample_buffers[0], BUFFER_SIZE);
	 *    nrfx_saadc_buffer_set(sample_buffers[1], BUFFER_SIZE);
	 */

	return 0;
}

/*
 * Set up DPPI connections between peripherals
 */
static int init_dppi(void)
{
	/* TODO 3: Wire peripherals together using DPPI
	 *
	 * a) Get the 4 endpoint addresses:
	 *    - TIMER22 COMPARE[0] event:
	 *      nrfx_timer_compare_event_address_get(&timer_instance, NRF_TIMER_CC_CHANNEL0)
	 *    - SAADC SAMPLE task:
	 *      nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_SAMPLE)
	 *    - SAADC END event:
	 *      nrf_saadc_event_address_get(NRF_SAADC, NRF_SAADC_EVENT_END)
	 *    - SAADC START task:
	 *      nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_START)
	 *
	 * b) Connection 1 — TIMER COMPARE -> SAADC SAMPLE (triggers each sample):
	 *    nrfx_gppi_conn_alloc(timer_compare_addr, saadc_sample_addr,
	 *                         &gppi_sample_handle);
	 *    nrfx_gppi_conn_enable(gppi_sample_handle);
	 *
	 * c) Connection 2 — SAADC END -> SAADC START (continuous operation):
	 *    nrfx_gppi_conn_alloc(saadc_end_addr, saadc_start_addr,
	 *                         &gppi_start_handle);
	 *    nrfx_gppi_conn_enable(gppi_start_handle);
	 */

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

	/* TODO 4: Start the autonomous sampling chain
	 *
	 * Call nrfx_saadc_mode_trigger() to put the SAADC into ready state.
	 * This will fire EVT_READY in saadc_handler(), which starts the TIMER.
	 * From there, everything runs in hardware.
	 */

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
