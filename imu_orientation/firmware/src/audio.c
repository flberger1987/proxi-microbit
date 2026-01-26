/*
 * SPDX-License-Identifier: Apache-2.0
 * Audio Module - PWM Tone Generation
 *
 * Uses PWM on the micro:bit v2 speaker (P0.00) for tone generation.
 */

#include "audio.h"
#include "robot_state.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/printk.h>

/* Thread configuration */
#define AUDIO_STACK_SIZE 768
#define AUDIO_PRIORITY 7

K_THREAD_STACK_DEFINE(audio_stack, AUDIO_STACK_SIZE);
static struct k_thread audio_thread_data;

/*
 * PWM device for speaker - micro:bit v2 speaker is on P0.00
 * The board already defines pwm-led0 for the speaker
 */
#define SPEAKER_NODE DT_ALIAS(pwm_led0)

#if DT_NODE_EXISTS(SPEAKER_NODE)
static const struct pwm_dt_spec speaker = PWM_DT_SPEC_GET(SPEAKER_NODE);
#else
static const struct pwm_dt_spec speaker = {0};
#endif

/* Audio state */
static volatile bool is_playing = false;
static volatile bool is_muted = false;  /* Mutes regular sounds during calibration */

/* ============================================================================
 * Internal Functions
 * ============================================================================ */

/**
 * Set PWM for a specific frequency
 */
static void set_tone(uint32_t frequency_hz)
{
    if (!device_is_ready(speaker.dev)) {
        return;
    }

    if (frequency_hz == 0) {
        /* Silence - set duty cycle to 0 */
        pwm_set_pulse_dt(&speaker, 0);
    } else {
        /* Calculate period in nanoseconds */
        uint32_t period_ns = 1000000000U / frequency_hz;
        /* 50% duty cycle for square wave */
        uint32_t pulse_ns = period_ns / 2;
        pwm_set_dt(&speaker, period_ns, pulse_ns);
    }
}

/**
 * Play a sequence of notes
 */
static void play_melody(const uint16_t *notes, const uint16_t *durations, size_t count)
{
    is_playing = true;
    for (size_t i = 0; i < count && is_playing; i++) {
        set_tone(notes[i]);
        k_msleep(durations[i]);
    }
    set_tone(0);
    is_playing = false;
}

/**
 * Pairing start sound - ascending tones
 */
static void play_pairing_start(void)
{
    static const uint16_t notes[] = {NOTE_C4, NOTE_E4, NOTE_G4};
    static const uint16_t durations[] = {100, 100, 150};
    play_melody(notes, durations, ARRAY_SIZE(notes));
}

/**
 * Connected sound - success melody
 */
static void play_connected(void)
{
    static const uint16_t notes[] = {NOTE_C5, NOTE_E5, NOTE_G5, NOTE_C5};
    static const uint16_t durations[] = {80, 80, 80, 200};
    play_melody(notes, durations, ARRAY_SIZE(notes));
}

/**
 * Disconnected sound - descending tones
 */
static void play_disconnected(void)
{
    static const uint16_t notes[] = {NOTE_G4, NOTE_E4, NOTE_C4};
    static const uint16_t durations[] = {100, 100, 200};
    play_melody(notes, durations, ARRAY_SIZE(notes));
}

/**
 * Obstacle warning sound
 */
static void play_obstacle(void)
{
    static const uint16_t notes[] = {NOTE_A4, 0, NOTE_A4};
    static const uint16_t durations[] = {50, 30, 50};
    play_melody(notes, durations, ARRAY_SIZE(notes));
}

/**
 * Button press click
 */
static void play_button_press(void)
{
    set_tone(NOTE_C5);
    is_playing = true;
    k_msleep(30);
    set_tone(0);
    is_playing = false;
}

/**
 * Error sound
 */
static void play_error(void)
{
    static const uint16_t notes[] = {NOTE_C4, 0, NOTE_C4, 0, NOTE_C4};
    static const uint16_t durations[] = {100, 50, 100, 50, 200};
    play_melody(notes, durations, ARRAY_SIZE(notes));
}

/**
 * Shooting/laser sound effect - rapid descending sweep
 */
static void play_shoot(void)
{
    is_playing = true;

    /* Laser "pew pew" - rapid frequency sweep from high to low */
    for (uint32_t freq = 2000; freq >= 400 && is_playing; freq -= 80) {
        set_tone(freq);
        k_usleep(8000);  /* 8ms per step = ~160ms total */
    }

    /* Short tail */
    set_tone(300);
    k_msleep(30);
    set_tone(0);

    is_playing = false;
}

/**
 * Machine gun sound effect - rapid bursts of noise
 */
static void play_machinegun(void)
{
    is_playing = true;

    /* Machine gun: rapid alternating high/low tones to simulate rattling */
    for (int burst = 0; burst < 12 && is_playing; burst++) {
        /* High frequency burst */
        set_tone(800 + (burst % 3) * 100);
        k_usleep(15000);  /* 15ms */

        /* Short silence between shots */
        set_tone(0);
        k_usleep(10000);  /* 10ms gap */
    }

    set_tone(0);
    is_playing = false;
}

/**
 * Calibration beep - short high-pitched tick
 */
static void play_calibration_beep(void)
{
    set_tone(NOTE_E5);
    is_playing = true;
    k_msleep(50);
    set_tone(0);
    is_playing = false;
}

/**
 * Calibration done - success jingle (ascending fanfare)
 */
static void play_calibration_done(void)
{
    static const uint16_t notes[] = {NOTE_C5, NOTE_E5, NOTE_G5, NOTE_C5, 0, NOTE_G5, NOTE_C5};
    static const uint16_t durations[] = {100, 100, 100, 150, 50, 100, 300};
    play_melody(notes, durations, ARRAY_SIZE(notes));
}

/* ============================================================================
 * Audio Thread
 * ============================================================================ */

static void audio_thread_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    enum sound_event event;
    int ret;

    printk("Audio thread started\n");

    while (1) {
        /* Wait for sound event */
        ret = k_msgq_get(&sound_q, &event, K_FOREVER);
        if (ret != 0) {
            continue;
        }

        /* Play the appropriate sound */
        switch (event) {
        case SOUND_PAIRING_START:
            play_pairing_start();
            break;
        case SOUND_CONNECTED:
            play_connected();
            break;
        case SOUND_DISCONNECTED:
            play_disconnected();
            break;
        case SOUND_OBSTACLE:
            play_obstacle();
            break;
        case SOUND_BUTTON_PRESS:
            play_button_press();
            break;
        case SOUND_ERROR:
            play_error();
            break;
        case SOUND_SHOOT:
            play_shoot();
            break;
        case SOUND_MACHINEGUN:
            play_machinegun();
            break;
        case SOUND_CALIBRATION_BEEP:
            play_calibration_beep();
            break;
        case SOUND_CALIBRATION_DONE:
            play_calibration_done();
            break;
        case SOUND_NONE:
        default:
            break;
        }
    }
}

/* ============================================================================
 * Public API
 * ============================================================================ */

int audio_init(void)
{
    if (!device_is_ready(speaker.dev)) {
        printk("Audio: Speaker PWM device not ready\n");
        return -ENODEV;
    }

    /* Ensure speaker is off */
    set_tone(0);

    printk("Audio: Initialized\n");
    return 0;
}

void audio_start_thread(void)
{
    k_thread_create(&audio_thread_data, audio_stack,
                    K_THREAD_STACK_SIZEOF(audio_stack),
                    audio_thread_fn, NULL, NULL, NULL,
                    AUDIO_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&audio_thread_data, "audio");
}

void audio_play(enum sound_event event)
{
    /* When muted, only allow calibration sounds */
    if (is_muted &&
        event != SOUND_CALIBRATION_BEEP &&
        event != SOUND_CALIBRATION_DONE) {
        return;
    }

    /* Non-blocking send to audio queue */
    k_msgq_put(&sound_q, &event, K_NO_WAIT);
}

void audio_set_muted(bool muted)
{
    is_muted = muted;
    if (muted) {
        /* Stop any currently playing sound when entering mute */
        audio_stop();
    }
}

bool audio_is_muted(void)
{
    return is_muted;
}

void audio_tone(uint32_t frequency_hz, uint32_t duration_ms)
{
    is_playing = true;
    set_tone(frequency_hz);
    k_msleep(duration_ms);
    set_tone(0);
    is_playing = false;
}

void audio_stop(void)
{
    is_playing = false;
    set_tone(0);
}

bool audio_is_playing(void)
{
    return is_playing;
}
