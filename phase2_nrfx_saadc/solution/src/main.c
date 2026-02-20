/*
 * Phase 2: nrfx SAADC Direct Control for nRF54L15 DK
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

    /* Initialize SAADC - connect interrupt using nrfx_isr wrapper */
    IRQ_CONNECT(DT_IRQN(DT_NODELABEL(adc)),
                DT_IRQ(DT_NODELABEL(adc), priority),
                nrfx_isr, nrfx_saadc_irq_handler, 0);

    err = nrfx_saadc_init(DT_IRQ(DT_NODELABEL(adc), priority));
    if (err != NRFX_SUCCESS) {
        LOG_ERR("SAADC init failed: 0x%x", err);
        return -1;
    }
    LOG_INF("SAADC initialized");

    /* Channel configuration for AIN4 (P1.11) */
    nrfx_saadc_channel_t channel = {
        .channel_config = {
            .resistor_p = NRF_SAADC_RESISTOR_DISABLED,
            .resistor_n = NRF_SAADC_RESISTOR_DISABLED,
            .gain = NRF_SAADC_GAIN1_4,
            .reference = NRF_SAADC_REFERENCE_INTERNAL,
            .acq_time = NRF_SAADC_ACQTIME_10US,
            .mode = NRF_SAADC_MODE_SINGLE_ENDED,
            .burst = NRF_SAADC_BURST_DISABLED,
        },
        .pin_p = (nrf_saadc_input_t)PULSE_SENSOR_PIN,
        .pin_n = NRF_SAADC_INPUT_DISABLED,
        .channel_index = 0,
    };

    err = nrfx_saadc_channels_config(&channel, 1);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("Channel config failed: 0x%x", err);
        return -1;
    }
    LOG_INF("Channel configured for AIN4 (P1.11)");

    /* Configure simple mode - software triggered */
    err = nrfx_saadc_simple_mode_set(BIT(0),
                                      NRF_SAADC_RESOLUTION_12BIT,
                                      NRF_SAADC_OVERSAMPLE_DISABLED,
                                      saadc_handler);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("Simple mode set failed: 0x%x", err);
        return -1;
    }

    /* Set up sample buffer */
    err = nrfx_saadc_buffer_set(sample_buffer, 1);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("Buffer set failed: 0x%x", err);
        return -1;
    }

    LOG_INF("Starting sampling... Place finger on sensor");
    LOG_INF("------------------------------------------");

    uint32_t count = 0;

    while (1) {
        /* Trigger a sample */
        err = nrfx_saadc_mode_trigger();
        if (err != NRFX_SUCCESS) {
            LOG_ERR("Trigger failed: 0x%x", err);
            k_msleep(SAMPLE_INTERVAL_MS);
            continue;
        }

        /* Wait for sample to complete */
        while (!sample_ready) {
            k_yield();
        }
        sample_ready = false;

        /* Log the sample */
        if (count % 10 == 0) {
            LOG_INF("ADC: %d", sample_buffer[0]);
        }
        count++;

        /* Re-arm buffer for next sample */
        nrfx_saadc_buffer_set(sample_buffer, 1);

        k_msleep(SAMPLE_INTERVAL_MS);
    }

    return 0;
}
