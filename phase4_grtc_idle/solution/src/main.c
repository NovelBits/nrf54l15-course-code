/*
 * Phase 4: GRTC Idle Mode with Burst Sampling
 *
 * Builds on Phase 3 (TIMER22 + DPPI -> SAADC) by adding:
 *   - GRTC wake-up timer for periodic burst sampling
 *   - Button 1 toggles between ACTIVE and IDLE modes
 *   - LED feedback (LED1 = active, LED2 = burst in progress)
 *
 * ACTIVE MODE:
 *   TIMER22 + DPPI -> SAADC continuous 200 Hz (same as Phase 3)
 *
 * IDLE MODE:
 *   GRTC CC[n] fires every 5 sec -> CPU wakes
 *     -> Start TIMER22+DPPI (1 sec burst, 200 samples)
 *     -> Stop chain, process buffer
 *     -> Re-arm GRTC, go back to sleep
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
#include <nrfx_grtc.h>
#include <helpers/nrfx_gppi.h>
#include <dk_buttons_and_leds.h>

LOG_MODULE_REGISTER(phase4_grtc_idle, LOG_LEVEL_INF);

/* ──────────────── State Machine ──────────────── */
enum app_state {
	STATE_ACTIVE,          /* Continuous sampling (Phase 3 behavior) */
	STATE_IDLE,            /* Sleeping, waiting for GRTC wake-up */
	STATE_BURST_START,     /* GRTC fired, starting burst */
	STATE_BURST_SAMPLING,  /* Burst in progress (1 sec) */
};

static volatile enum app_state current_state = STATE_ACTIVE;

/* ──────────────── Sampling Parameters ──────────────── */
#define SAMPLE_INTERVAL_US      5000    /* 5ms = 200 Hz sampling */
#define BUFFER_SIZE             200     /* 1 second of samples at 200 Hz */
#define BUFFER_COUNT            2       /* Double buffering */

/* Burst / idle parameters */
#define BURST_INTERVAL_US       5000000 /* 5 seconds between bursts */
#define BURST_DURATION_MS       1000    /* 1 second burst */

/* ──────────────── TIMER Configuration ──────────────── */
#define TIMER_INSTANCE_NUMBER   22
static nrfx_timer_t timer_instance = NRFX_TIMER_INSTANCE(TIMER_INSTANCE_NUMBER);

/* ──────────────── SAADC Configuration ──────────────── */
#define SAADC_CHANNEL           0

#if NRF_SAADC_HAS_AIN_AS_PIN
#define SAADC_INPUT_PIN         NRFX_ANALOG_EXTERNAL_AIN4
#else
#define SAADC_INPUT_PIN         NRF_SAADC_INPUT_AIN4
#endif

/* ──────────────── DPPI Handles ──────────────── */
static nrfx_gppi_handle_t gppi_sample_handle;
static nrfx_gppi_handle_t gppi_start_handle;

/* ──────────────── GRTC ──────────────── */
static uint8_t grtc_channel;
static nrfx_grtc_channel_t grtc_chan_data;

/* ──────────────── Statistics ──────────────── */
static volatile uint32_t buffer_full_count;
static volatile uint32_t burst_count;

/* ──────────────── Sample Buffers ──────────────── */
static int16_t sample_buffers[BUFFER_COUNT][BUFFER_SIZE];

/* ──────────────── SAADC Channel Config ──────────────── */
static nrfx_saadc_channel_t channel = NRFX_SAADC_DEFAULT_CHANNEL_SE(
	SAADC_INPUT_PIN, SAADC_CHANNEL);

/* ──────────────── Forward Declarations ──────────────── */
static void start_sampling_chain(void);
static void stop_sampling_chain(void);
static void schedule_grtc_wakeup(void);

/*
 * SAADC event handler
 *
 * In ACTIVE mode, this works identically to Phase 3.
 * In BURST_SAMPLING, it processes the burst buffer and transitions to IDLE.
 */
static void saadc_handler(nrfx_saadc_evt_t const *p_event)
{
	static uint8_t buffer_index;

	switch (p_event->type) {
	case NRFX_SAADC_EVT_READY:
		nrfx_timer_enable(&timer_instance);
		if (current_state == STATE_BURST_START) {
			current_state = STATE_BURST_SAMPLING;
			LOG_INF("Burst sampling started");
		} else {
			LOG_INF("SAADC ready, TIMER started — continuous sampling active");
		}
		break;

	case NRFX_SAADC_EVT_BUF_REQ:
		nrfx_saadc_buffer_set(
			sample_buffers[(buffer_index++) % BUFFER_COUNT],
			BUFFER_SIZE);
		break;

	case NRFX_SAADC_EVT_DONE: {
		buffer_full_count++;

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

		const char *mode_str = (current_state == STATE_BURST_SAMPLING)
					       ? "BURST"
					       : "ACTIVE";
		LOG_INF("[%s] Buffer %d: %d samples | AVG=%d (%d mV) "
			"MIN=%d (%d mV) MAX=%d (%d mV)",
			mode_str, buffer_full_count,
			p_event->data.done.size, avg, avg_mv, min_val,
			min_mv, max_val, max_mv);

		/* In burst mode, stop after one buffer (1 sec of data) */
		if (current_state == STATE_BURST_SAMPLING) {
			stop_sampling_chain();
			dk_set_led_off(DK_LED2);
			burst_count++;
			LOG_INF("Burst %d complete — returning to idle",
				burst_count);

			/* Schedule next wake-up and go back to idle */
			current_state = STATE_IDLE;
			schedule_grtc_wakeup();
		}
		break;
	}

	default:
		break;
	}
}

/*
 * GRTC compare handler
 *
 * Fires when the GRTC CC channel reaches the scheduled value.
 * Transitions from IDLE to BURST_START.
 */
static void grtc_compare_handler(int32_t id, uint32_t cc_channel, void *p_context)
{
	ARG_UNUSED(id);
	ARG_UNUSED(cc_channel);
	ARG_UNUSED(p_context);

	if (current_state != STATE_IDLE) {
		/* Ignore if we're not in idle mode (e.g., button pressed
		 * during GRTC window)
		 */
		return;
	}

	LOG_INF("GRTC wake-up — starting burst");
	current_state = STATE_BURST_START;
	dk_set_led_on(DK_LED2);
	start_sampling_chain();
}

/*
 * Button handler
 *
 * Button 1 (DK_BTN1) toggles between ACTIVE and IDLE modes.
 * Ignores presses during burst states.
 */
static void button_handler(uint32_t button_state, uint32_t has_changed)
{
	if ((has_changed & DK_BTN1_MSK) && (button_state & DK_BTN1_MSK)) {
		/* Only handle press (not release) */
		switch (current_state) {
		case STATE_ACTIVE:
			LOG_INF("Button 1: ACTIVE -> IDLE");
			stop_sampling_chain();
			dk_set_led_off(DK_LED1);
			current_state = STATE_IDLE;
			schedule_grtc_wakeup();
			break;

		case STATE_IDLE:
			LOG_INF("Button 1: IDLE -> ACTIVE");
			current_state = STATE_ACTIVE;
			dk_set_led_on(DK_LED1);
			start_sampling_chain();
			break;

		case STATE_BURST_START:
		case STATE_BURST_SAMPLING:
			LOG_INF("Button 1: ignored during burst");
			break;
		}
	}
}

/* ──────────────── Peripheral Init Functions ──────────────── */

/*
 * Initialize TIMER22 for periodic 200 Hz events
 * Reused from Phase 3 — no changes needed.
 */
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

	LOG_INF("TIMER22 configured: %d us interval (%d ticks at 1 MHz)",
		SAMPLE_INTERVAL_US, ticks);

	return 0;
}

/*
 * Initialize SAADC for hardware-triggered sampling
 * Reused from Phase 3 — no changes needed.
 */
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

	LOG_INF("SAADC initialized: 12-bit, AIN4 (P1.11), external trigger");
	return 0;
}

/*
 * Set up DPPI connections
 * Reused from Phase 3 — no changes needed.
 */
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

	err = nrfx_gppi_conn_alloc(timer_compare_addr, saadc_sample_addr,
				   &gppi_sample_handle);
	if (err != 0) {
		LOG_ERR("GPPI alloc (TIMER->SAADC SAMPLE) failed: %d", err);
		return -EIO;
	}
	nrfx_gppi_conn_enable(gppi_sample_handle);
	LOG_INF("DPPI: TIMER22 COMPARE[0] -> SAADC SAMPLE");

	err = nrfx_gppi_conn_alloc(saadc_end_addr, saadc_start_addr,
				   &gppi_start_handle);
	if (err != 0) {
		LOG_ERR("GPPI alloc (SAADC END->START) failed: %d", err);
		return -EIO;
	}
	nrfx_gppi_conn_enable(gppi_start_handle);
	LOG_INF("DPPI: SAADC END -> SAADC START");

	return 0;
}

/*
 * Initialize GRTC for periodic wake-up
 *
 * Allocates a free CC channel (avoids CC[0] which Zephyr reserves),
 * sets up the compare callback.
 */
static int init_grtc(void)
{
	int err;

	err = nrfx_grtc_channel_alloc(&grtc_channel);
	if (err != 0) {
		LOG_ERR("GRTC channel alloc failed: %d", err);
		return -EIO;
	}

	nrfx_grtc_channel_callback_set(grtc_channel, grtc_compare_handler,
				       NULL);

	/* Prepare channel data struct for cc_relative_set */
	grtc_chan_data.channel = grtc_channel;

	LOG_INF("GRTC initialized: CC channel %d allocated", grtc_channel);
	return 0;
}

/*
 * Initialize buttons and LEDs via DK library
 */
static int init_buttons(void)
{
	int err;

	err = dk_leds_init();
	if (err != 0) {
		LOG_ERR("LED init failed: %d", err);
		return -EIO;
	}

	err = dk_buttons_init(button_handler);
	if (err != 0) {
		LOG_ERR("Button init failed: %d", err);
		return -EIO;
	}

	LOG_INF("Buttons and LEDs initialized");
	return 0;
}

/* ──────────────── Helper Functions ──────────────── */

/*
 * Start the TIMER22 + DPPI + SAADC sampling chain.
 *
 * Re-sets SAADC buffers (required after abort) and triggers mode_trigger().
 * The timer is started inside the EVT_READY handler to avoid DPPI race.
 */
static void start_sampling_chain(void)
{
	int err;

	/* Re-enable DPPI connections */
	nrfx_gppi_conn_enable(gppi_sample_handle);
	nrfx_gppi_conn_enable(gppi_start_handle);

	/* Re-set double buffers (required after nrfx_saadc_abort) */
	err = nrfx_saadc_buffer_set(sample_buffers[0], BUFFER_SIZE);
	if (err != 0) {
		LOG_ERR("SAADC buffer[0] set failed: %d", err);
		return;
	}

	err = nrfx_saadc_buffer_set(sample_buffers[1], BUFFER_SIZE);
	if (err != 0) {
		LOG_ERR("SAADC buffer[1] set failed: %d", err);
		return;
	}

	/* Trigger SAADC ready state — timer starts in EVT_READY handler */
	err = nrfx_saadc_mode_trigger();
	if (err != 0) {
		LOG_ERR("SAADC mode trigger failed: %d", err);
	}
}

/*
 * Stop the TIMER22 + DPPI + SAADC sampling chain.
 */
static void stop_sampling_chain(void)
{
	nrfx_timer_disable(&timer_instance);
	nrfx_gppi_conn_disable(gppi_sample_handle);
	nrfx_gppi_conn_disable(gppi_start_handle);
	nrfx_saadc_abort();
}

/*
 * Schedule the next GRTC wake-up after BURST_INTERVAL_US microseconds.
 *
 * Uses nrfx_grtc_syscounter_cc_relative_set() which sets the CC value
 * relative to the current SYSCOUNTER and enables the interrupt.
 */
static void schedule_grtc_wakeup(void)
{
	int err;

	err = nrfx_grtc_syscounter_cc_relative_set(&grtc_chan_data,
						   BURST_INTERVAL_US, true,
						   NRFX_GRTC_CC_RELATIVE_SYSCOUNTER);
	if (err != 0) {
		LOG_ERR("GRTC schedule failed: %d", err);
		return;
	}

	LOG_INF("GRTC: next wake-up in %d ms",
		BURST_INTERVAL_US / 1000);
}

/* ──────────────── Main ──────────────── */

int main(void)
{
	int err;

	LOG_INF("==========================================");
	LOG_INF("Phase 4: GRTC Idle Mode with Burst Sampling");
	LOG_INF("==========================================");
	LOG_INF("Sample rate: %d Hz (%d us interval)",
		1000000 / SAMPLE_INTERVAL_US, SAMPLE_INTERVAL_US);
	LOG_INF("Buffer: %d samples (%d ms per buffer)",
		BUFFER_SIZE, (BUFFER_SIZE * SAMPLE_INTERVAL_US) / 1000);
	LOG_INF("Burst interval: %d ms, burst duration: %d ms",
		BURST_INTERVAL_US / 1000, BURST_DURATION_MS);
	LOG_INF("");

	/* Initialize peripherals in order */
	err = init_timer();
	if (err) {
		return err;
	}

	err = init_saadc();
	if (err) {
		return err;
	}

	err = init_dppi();
	if (err) {
		return err;
	}

	err = init_grtc();
	if (err) {
		return err;
	}

	err = init_buttons();
	if (err) {
		return err;
	}

	/* Start in ACTIVE mode (continuous sampling, like Phase 3) */
	current_state = STATE_ACTIVE;
	dk_set_led_on(DK_LED1);
	start_sampling_chain();

	LOG_INF("");
	LOG_INF("Started in ACTIVE mode (continuous sampling)");
	LOG_INF("Press Button 1 to toggle ACTIVE <-> IDLE");
	LOG_INF("");

	/* Main loop: state machine runs via interrupts.
	 * The CPU sleeps in k_msleep() and wakes only for:
	 *   - SAADC buffer-full interrupts (1/sec in ACTIVE, 1/burst in IDLE)
	 *   - GRTC compare interrupts (every 5 sec in IDLE)
	 *   - Button interrupts
	 */
	while (1) {
		k_msleep(1000);
	}

	return 0;
}
