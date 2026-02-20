/*
 * PulseSensor Beat Detection Algorithm
 *
 * Ported from PulseSensor Playground Library
 * Original: https://github.com/WorldFamousElectronics/PulseSensorPlayground
 *
 * Copyright World Famous Electronics LLC - MIT License
 * Contributors: Joel Murphy, Yury Gitman, Bradford Needham
 *
 * This software is not intended for medical use.
 */

#include "pulse_sensor.h"

/* Default values for 12-bit ADC (0-4095 range) */
#define DEFAULT_PEAK      2048    /* Mid-range */
#define DEFAULT_TROUGH    2048    /* Mid-range */
#define DEFAULT_AMP       400     /* ~10% of range */
#define DEFAULT_IBI       750     /* 750ms = 80 BPM */
#define TIMEOUT_MS        2500    /* Reset after 2.5 seconds of no beat */
#define MIN_BEAT_MS       250     /* Ignore beats faster than 240 BPM */

void pulse_sensor_init(pulse_sensor_t *ps, int threshold, int sample_interval_ms)
{
    ps->threshold_setting = threshold;
    ps->sample_interval_ms = sample_interval_ms;
    pulse_sensor_reset(ps);
}

void pulse_sensor_reset(pulse_sensor_t *ps)
{
    /* Clear rate array */
    for (int i = 0; i < 10; i++) {
        ps->rate[i] = 0;
    }

    /* Reset state */
    ps->qs = false;
    ps->bpm = 0;
    ps->ibi = DEFAULT_IBI;
    ps->pulse = false;
    ps->sample_counter = 0;
    ps->last_beat_time = 0;
    ps->peak = DEFAULT_PEAK;
    ps->trough = DEFAULT_TROUGH;
    ps->threshold = ps->threshold_setting;
    ps->amplitude = DEFAULT_AMP;
    ps->first_beat = true;
    ps->second_beat = false;
}

bool pulse_sensor_process(pulse_sensor_t *ps, int sample)
{
    ps->signal = sample;
    ps->sample_counter += ps->sample_interval_ms;

    /* N = time since last beat (used for noise filtering) */
    unsigned long n = ps->sample_counter - ps->last_beat_time;

    /* Clear the beat-detected flag from previous call */
    ps->qs = false;

    /*
     * Find the peak and trough of the pulse wave
     * Avoid dichrotic noise by waiting 3/5 of last IBI
     */
    if (sample < ps->threshold && n > (ps->ibi / 5) * 3) {
        if (sample < ps->trough) {
            ps->trough = sample;    /* Track lowest point */
        }
    }

    if (sample > ps->threshold && sample > ps->peak) {
        ps->peak = sample;          /* Track highest point */
    }

    /*
     * NOW IT'S TIME TO LOOK FOR THE HEART BEAT
     * Signal surges up in value every time there is a pulse
     */
    if (n > MIN_BEAT_MS) {          /* Avoid high frequency noise */
        if ((sample > ps->threshold) &&
            (ps->pulse == false) &&
            (n > (ps->ibi / 5) * 3)) {

            /* Beat detected! */
            ps->pulse = true;
            ps->ibi = ps->sample_counter - ps->last_beat_time;
            ps->last_beat_time = ps->sample_counter;

            if (ps->second_beat) {
                /* Second beat - seed the rate array for realistic startup BPM */
                ps->second_beat = false;
                for (int i = 0; i < 10; i++) {
                    ps->rate[i] = ps->ibi;
                }
            }

            if (ps->first_beat) {
                /* First beat - IBI is unreliable, discard it */
                ps->first_beat = false;
                ps->second_beat = true;
                return false;
            }

            /*
             * Calculate BPM from rolling average of last 10 IBI values
             */
            int running_total = 0;

            /* Shift data in the rate array, drop oldest */
            for (int i = 0; i < 9; i++) {
                ps->rate[i] = ps->rate[i + 1];
                running_total += ps->rate[i];
            }

            ps->rate[9] = ps->ibi;              /* Add latest IBI */
            running_total += ps->rate[9];
            running_total /= 10;                 /* Average */
            ps->bpm = 60000 / running_total;     /* Convert to BPM */
            ps->qs = true;                       /* Signal beat detected */
        }
    }

    /*
     * When signal drops below threshold, the beat is over
     */
    if (sample < ps->threshold && ps->pulse == true) {
        ps->pulse = false;
        ps->amplitude = ps->peak - ps->trough;

        /* Set adaptive threshold at 50% of amplitude */
        ps->threshold = ps->amplitude / 2 + ps->trough;

        /* Reset peak/trough for next beat */
        ps->peak = ps->threshold;
        ps->trough = ps->threshold;
    }

    /*
     * If 2.5 seconds go by without a beat, reset
     */
    if (n > TIMEOUT_MS) {
        ps->threshold = ps->threshold_setting;
        ps->peak = DEFAULT_PEAK;
        ps->trough = DEFAULT_TROUGH;
        ps->last_beat_time = ps->sample_counter;
        ps->first_beat = true;
        ps->second_beat = false;
        ps->qs = false;
        ps->bpm = 0;
        ps->ibi = DEFAULT_IBI;
        ps->pulse = false;
        ps->amplitude = DEFAULT_AMP;
    }

    return ps->qs;
}
