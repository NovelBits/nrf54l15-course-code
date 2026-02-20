/*
 * Phase 2: nrfx SAADC Direct Control for nRF54L15 DK - STARTER
 *
 * Goal: Switch from Zephyr ADC API to nrfx driver for direct hardware control.
 * This is needed for DPPI hardware triggering in Phase 4.
 *
 * Note: nRF54L15 has a different nrfx SAADC API compared to older nRF5x devices.
 *
 * Wiring:
 *   Pulse Sensor Signal (S) -> P1.11 (AIN4)
 *   Pulse Sensor VCC (+)    -> 3.3V
 *   Pulse Sensor GND (-)    -> GND
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <nrfx_saadc.h>

LOG_MODULE_REGISTER(phase2_saadc, LOG_LEVEL_INF);

#define SAMPLE_RATE_HZ      50
#define SAMPLE_INTERVAL_MS  (1000 / SAMPLE_RATE_HZ)

/* P1.11 = AIN4 on nRF54L15 */
#define PULSE_SENSOR_PIN    NRF_PIN_PORT_TO_PIN_NUMBER(1U, 11)

/* Sample buffer */
static nrf_saadc_value_t sample_buffer[1];
static volatile bool sample_ready = false;

/* SAADC event handler */
static void saadc_handler(nrfx_saadc_evt_t const *p_event)
{
    if (p_event->type == NRFX_SAADC_EVT_DONE) {
        sample_ready = true;
    }
}

int main(void)
{
    nrfx_err_t err;

    LOG_INF("==========================================");
    LOG_INF("Phase 2: nrfx SAADC Direct Control");
    LOG_INF("==========================================");
    LOG_INF("Using nrfx driver instead of Zephyr ADC API");
    LOG_INF("Sample rate: %d Hz", SAMPLE_RATE_HZ);
    LOG_INF("");

    /* TODO: Initialize SAADC
     * Hint: Use IRQ_CONNECT to connect interrupt handler
     * Then call nrfx_saadc_init()
     */

    /* TODO: Configure SAADC channel for AIN4 (P1.11)
     * Hint: Create nrfx_saadc_channel_t struct with:
     * - Gain: NRF_SAADC_GAIN1_4
     * - Reference: NRF_SAADC_REFERENCE_INTERNAL
     * - Acquisition time: NRF_SAADC_ACQTIME_10US
     * - Mode: NRF_SAADC_MODE_SINGLE_ENDED
     * Then call nrfx_saadc_channels_config()
     */

    /* TODO: Configure simple mode
     * Hint: Use nrfx_saadc_simple_mode_set() with:
     * - Channel mask: BIT(0)
     * - Resolution: NRF_SAADC_RESOLUTION_12BIT
     * - Oversampling: NRF_SAADC_OVERSAMPLE_DISABLED
     * - Event handler: saadc_handler
     */

    /* TODO: Set up sample buffer
     * Hint: Use nrfx_saadc_buffer_set()
     */

    LOG_INF("Starting sampling... Place finger on sensor");
    LOG_INF("------------------------------------------");

    uint32_t count = 0;

    while (1) {
        /* TODO: Trigger a sample
         * Hint: Use nrfx_saadc_mode_trigger()
         */

        /* TODO: Wait for sample to complete
         * Hint: Poll sample_ready flag, use k_yield()
         */

        /* TODO: Log the sample value
         * Hint: Read from sample_buffer[0]
         */

        /* TODO: Re-arm buffer for next sample
         * Hint: Call nrfx_saadc_buffer_set()
         */

        k_msleep(SAMPLE_INTERVAL_MS);
    }

    return 0;
}
