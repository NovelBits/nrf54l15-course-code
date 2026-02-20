/*
 * Phase 0: Raw ADC Reading - STARTER
 *
 * Wiring:
 *   Pulse Sensor Signal (S) -> P1.11 (AIN4)
 *   Pulse Sensor VCC (+)    -> 3.3V
 *   Pulse Sensor GND (-)    -> GND
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(raw_adc, LOG_LEVEL_INF);

#define SAMPLE_INTERVAL_MS  100  /* 10 Hz sample rate */

#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
    !DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "Devicetree overlay required"
#endif

/* TODO: Get ADC channel from devicetree
 * Hint: Use ADC_DT_SPEC_GET_BY_IDX() to get the channel spec
 */

int main(void)
{
    int err;
    uint16_t buf;
    struct adc_sequence sequence = {
        .buffer = &buf,
        .buffer_size = sizeof(buf),
    };

    LOG_INF("Phase 0: Raw ADC Reading");
    LOG_INF("Sample rate: 10 Hz");

    /* TODO: Check if ADC device is ready
     * Hint: Use adc_is_ready_dt()
     */

    /* TODO: Set up ADC channel
     * Hint: Use adc_channel_setup_dt()
     */

    LOG_INF("ADC ready. Place finger on sensor...");

    while (1) {
        /* TODO: Initialize ADC sequence
         * Hint: Use adc_sequence_init_dt()
         */

        /* TODO: Read ADC value
         * Hint: Use adc_read_dt()
         * Log the value with LOG_INF()
         */

        k_msleep(SAMPLE_INTERVAL_MS);
    }

    return 0;
}
