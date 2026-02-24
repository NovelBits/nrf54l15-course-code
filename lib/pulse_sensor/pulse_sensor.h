/*
 * PulseSensor Beat Detection Algorithm
 *
 * Ported from PulseSensor Playground Library
 * Original: https://github.com/WorldFamousElectronics/PulseSensorPlayground
 *
 * Copyright World Famous Electronics LLC - MIT License
 * Contributors: Joel Murphy, Yury Gitman, Bradford Needham
 *
 * Ported for Zephyr/nRF by: Novel Bits
 *
 * This software is not intended for medical use.
 */

#ifndef PULSE_SENSOR_H
#define PULSE_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

/**
 * PulseSensor state structure
 *
 * This algorithm provides:
 * - Adaptive threshold (auto-adjusts to signal amplitude)
 * - Rolling average of 10 IBI values for stable BPM
 * - Dichrotic noise filtering
 * - Auto-reset after 2.5 seconds of no beats
 */
typedef struct {
    /* Configuration */
    int threshold_setting;      /* User-defined initial threshold */
    int sample_interval_ms;     /* Time between samples in ms */

    /* Signal tracking */
    int signal;                 /* Latest ADC sample */
    int peak;                   /* Peak value (P) */
    int trough;                 /* Trough value (T) */
    int threshold;              /* Current adaptive threshold */
    int amplitude;              /* Beat amplitude (P - T) */

    /* Timing */
    unsigned long sample_counter;   /* Running sample time in ms */
    unsigned long last_beat_time;   /* Time of last beat in ms */
    int ibi;                        /* Inter-Beat Interval in ms */
    int rate[10];                   /* Array of last 10 IBI values */

    /* State flags */
    bool pulse;                 /* True during a beat */
    bool qs;                    /* Quantified Self - beat detected flag */
    bool first_beat;            /* Looking for first beat */
    bool second_beat;           /* Looking for second beat */

    /* Output */
    int bpm;                    /* Beats per minute */
} pulse_sensor_t;

/**
 * Initialize the pulse sensor with default values
 *
 * @param ps Pulse sensor state
 * @param threshold Initial threshold (typically 550 for 0-1023 range,
 *                  or 2200 for 0-4095 range)
 * @param sample_interval_ms Time between samples (typically 5ms for 200Hz)
 */
void pulse_sensor_init(pulse_sensor_t *ps, int threshold, int sample_interval_ms);

/**
 * Reset all variables to initial state
 * Call this when signal is lost or to restart detection
 *
 * @param ps Pulse sensor state
 */
void pulse_sensor_reset(pulse_sensor_t *ps);

/**
 * Process a new ADC sample
 * Call this at regular intervals (e.g., every 5ms)
 *
 * @param ps Pulse sensor state
 * @param sample ADC reading (0-4095 for 12-bit)
 * @return true if a beat was just detected (check bpm for value)
 */
bool pulse_sensor_process(pulse_sensor_t *ps, int sample);

/**
 * Get current BPM (0 if not yet calculated)
 */
static inline int pulse_sensor_get_bpm(pulse_sensor_t *ps) {
    return ps->bpm;
}

/**
 * Get inter-beat interval in milliseconds
 */
static inline int pulse_sensor_get_ibi(pulse_sensor_t *ps) {
    return ps->ibi;
}

/**
 * Get beat amplitude (useful for signal quality assessment)
 */
static inline int pulse_sensor_get_amplitude(pulse_sensor_t *ps) {
    return ps->amplitude;
}

/**
 * Check if currently inside a beat
 */
static inline bool pulse_sensor_is_beating(pulse_sensor_t *ps) {
    return ps->pulse;
}

/**
 * Get the latest signal value
 */
static inline int pulse_sensor_get_signal(pulse_sensor_t *ps) {
    return ps->signal;
}

#endif /* PULSE_SENSOR_H */
