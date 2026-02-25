/*
 * TAMPC Active Shield Demo for nRF54L15 DK
 *
 * Demonstrates the Tamper Controller (TAMPC) active shield feature.
 * A PRBS signal is output on ASO[3] (P1.11) and sampled on ASI[3] (P1.12).
 * Connect P1.11 to P1.12 with a wire. Disconnecting the wire triggers a
 * tamper event (LED1 toggles, message logged).
 *
 * Hardware setup:
 *   - Wire from P1.11 to P1.12 on the DK headers
 *   - LED1 (DK onboard) used for tamper indication
 *   - BTN0 (P1.13) resets the tamper counter
 *
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/irq.h>
#include <hal/nrf_tampc.h>
#include <hal/nrf_gpio.h>

LOG_MODULE_REGISTER(tampc_demo, LOG_LEVEL_INF);

/* LED1 on nRF54L15 DK */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

/* Button 0 (P1.13) — resets the tamper counter */
static const struct gpio_dt_spec btn = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);

/* Tamper event counter */
static volatile uint32_t tamper_count;

static void tampc_irq_handler(const void *arg)
{
	ARG_UNUSED(arg);

	if (nrf_tampc_event_check(NRF_TAMPC, NRF_TAMPC_EVENT_TAMPER)) {
		nrf_tampc_event_clear(NRF_TAMPC, NRF_TAMPC_EVENT_TAMPER);
		/* Clear latched STATUS bits (W1C) */
		NRF_TAMPC->STATUS = NRF_TAMPC->STATUS;
		tamper_count++;
		gpio_pin_toggle_dt(&led);
	}

	if (nrf_tampc_event_check(NRF_TAMPC, NRF_TAMPC_EVENT_WRITE_ERROR)) {
		nrf_tampc_event_clear(NRF_TAMPC, NRF_TAMPC_EVENT_WRITE_ERROR);
	}
}

static int tampc_init(void)
{
	/*
	 * Step 1: Configure GPIO pins for active shield Channel 3
	 *   ASO[3] = P1.11 — leave as INPUT (high-Z) so TAMPC can drive
	 *                      PRBS freely through its dedicated path.
	 *                      Configuring as GPIO OUTPUT drives the pin LOW,
	 *                      conflicting with TAMPC's PRBS signal.
	 *   ASI[3] = P1.12 — input with buffer connected so TAMPC can sample.
	 */
	nrf_gpio_cfg(
		NRF_GPIO_PIN_MAP(1, 11),         /* P1.11 = ASO[3] */
		NRF_GPIO_PIN_DIR_INPUT,
		NRF_GPIO_PIN_INPUT_CONNECT,
		NRF_GPIO_PIN_NOPULL,
		NRF_GPIO_PIN_S0S1,
		NRF_GPIO_PIN_NOSENSE
	);

	nrf_gpio_cfg(
		NRF_GPIO_PIN_MAP(1, 12),         /* P1.12 = ASI[3] */
		NRF_GPIO_PIN_DIR_INPUT,
		NRF_GPIO_PIN_INPUT_CONNECT,
		NRF_GPIO_PIN_NOPULL,
		NRF_GPIO_PIN_S0S1,
		NRF_GPIO_PIN_NOSENSE
	);

	LOG_INF("GPIO configured: P1.11 (ASO[3]) high-Z, P1.12 (ASI[3]) input");

	/*
	 * Step 2: Disable glitch detectors — they are enabled at reset and
	 * fire on normal DK power supply noise, masking active shield events.
	 */
	nrf_tampc_protector_ctrl_value_set(NRF_TAMPC,
					   NRF_TAMPC_PROTECT_GLITCH_DOMAIN_SLOW,
					   false);
	nrf_tampc_protector_ctrl_value_set(NRF_TAMPC,
					   NRF_TAMPC_PROTECT_GLITCH_DOMAIN_FAST,
					   false);
	LOG_INF("Glitch detectors disabled");

	/*
	 * Step 3: Clear any stale STATUS from a previous run.
	 * TAMPC STATUS is cumulative and survives certain resets.
	 */
	NRF_TAMPC->STATUS = NRF_TAMPC->STATUS;
	LOG_INF("TAMPC STATUS cleared");

	/*
	 * Step 4: Enable the active shield detector (master enable)
	 *
	 * PROTECT registers require the write key (0x50FA) and clearing
	 * WRITEPROTECTION before writing VALUE. The HAL function handles this.
	 */
	nrf_tampc_protector_ctrl_value_set(NRF_TAMPC,
					   NRF_TAMPC_PROTECT_ACTIVE_SHIELD,
					   true);
	LOG_INF("Active shield detector enabled");

	/*
	 * Step 5: Enable Channel 3
	 */
	/* NRF_TAMPC_ACTIVESHIELD_CHANNEL_3_MASK is not defined for nRF54L15 by
	 * the HAL (the device header doesn't pre-define the channel count macro
	 * needed to unlock it in the enum). Use the raw MDK bit mask instead. */
	nrf_tampc_activeshield_channel_enable(NRF_TAMPC,
		TAMPC_ACTIVESHIELD_CHEN_CH3_Msk);
	LOG_INF("Active shield Channel 3 enabled");

	/*
	 * Step 6: Enable TAMPER interrupt
	 */
	nrf_tampc_int_enable(NRF_TAMPC, NRF_TAMPC_INT_TAMPER_MASK);

	IRQ_CONNECT(TAMPC_IRQn, 1, tampc_irq_handler, NULL, 0);
	irq_enable(TAMPC_IRQn);
	LOG_INF("TAMPC interrupt enabled (IRQn=%d)", TAMPC_IRQn);

	return 0;
}

static void draw_header(void)
{
	/* Clear screen, home cursor, hide cursor */
	printk("\033[2J\033[H\033[?25l");
	printk("================================\r\n");
	printk("  nRF54L15 TAMPC Active Shield\r\n");
	printk("================================\r\n");
	printk("  Wire : P1.11 <---> P1.12\r\n");
	printk("  Test : disconnect to trigger\r\n");
	printk("================================\r\n");
	printk("  Status : \r\n");    /* row 7, value at col 12 */
	printk("  Events : \r\n");    /* row 8, value at col 12 */
	printk("================================\r\n");
	printk("  BTN0   : reset counter\r\n");
}

int main(void)
{
	int ret;

	LOG_INF("=== TAMPC Active Shield Demo ===");
	LOG_INF("Connect a wire from P1.11 to P1.12");
	LOG_INF("Disconnect the wire to trigger a tamper event");

	/* Configure LED */
	if (!gpio_is_ready_dt(&led)) {
		LOG_ERR("LED device not ready");
		return -ENODEV;
	}
	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure LED: %d", ret);
		return ret;
	}

	/* Configure Button 0 (P1.13) as input */
	if (!gpio_is_ready_dt(&btn)) {
		LOG_ERR("Button device not ready");
		return -ENODEV;
	}
	ret = gpio_pin_configure_dt(&btn, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure button: %d", ret);
		return ret;
	}

	/* Initialize TAMPC active shield */
	ret = tampc_init();
	if (ret < 0) {
		LOG_ERR("TAMPC init failed: %d", ret);
		return ret;
	}

	/* Let init log messages flush before clearing the screen */
	k_sleep(K_MSEC(300));
	draw_header();

	uint32_t last_count = 0;
	uint32_t highlight = 0;  /* ticks remaining to show red */
	bool btn_prev = false;

	while (1) {
		k_sleep(K_MSEC(100));

		/* Button 0: reset counter on press (edge detection) */
		bool btn_now = (gpio_pin_get_dt(&btn) > 0);

		if (btn_now && !btn_prev) {
			tamper_count = 0;
			last_count = 0;
			highlight = 0;
			draw_header();
		}
		btn_prev = btn_now;

		uint32_t count = tamper_count;

		if (count != last_count) {
			last_count = count;
			highlight = 20;  /* ~2 s at 100 ms/tick */
		}
		if (highlight > 0) {
			highlight--;
		}

		/* Row 7, col 12: status */
		if (highlight > 0) {
			printk("\033[7;12H\033[31;1mTAMPER DETECTED!\033[0m     ");
		} else {
			printk("\033[7;12H\033[32mARMED\033[0m               ");
		}

		/* Row 8, col 12: event count — %-10u always overwrites
		 * any previous value up to the full uint32_t width. */
		printk("\033[8;12H\033[33m%-10u\033[0m", count);
	}

	return 0;
}
