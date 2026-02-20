/*
 * Phase 0: Raw ADC Reading
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

static const struct adc_dt_spec adc_channel =
    ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);

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

    if (!adc_is_ready_dt(&adc_channel)) {
        LOG_ERR("ADC device not ready");
        return -1;
    }

    err = adc_channel_setup_dt(&adc_channel);
    if (err < 0) {
        LOG_ERR("ADC channel setup failed (%d)", err);
        return err;
    }

    LOG_INF("ADC ready. Place finger on sensor...");

    while (1) {
        adc_sequence_init_dt(&adc_channel, &sequence);

        err = adc_read_dt(&adc_channel, &sequence);
        if (err < 0) {
            LOG_ERR("ADC read error (%d)", err);
        } else {
            LOG_INF("ADC: %d", (int16_t)buf);
        }

        k_msleep(SAMPLE_INTERVAL_MS);
    }

    return 0;
}
