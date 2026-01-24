/*
 * SPDX-License-Identifier: Apache-2.0
 * Audio Module - PWM Tone Generation
 *
 * Timer-based PWM audio for feedback sounds on the micro:bit v2 speaker.
 */

#ifndef AUDIO_H
#define AUDIO_H

#include "robot_state.h"

/**
 * Initialize the audio module
 *
 * @return 0 on success, negative errno on failure
 */
int audio_init(void);

/**
 * Start the audio thread
 * Processes sound events from sound_q
 */
void audio_start_thread(void);

/**
 * Play a sound event (non-blocking)
 * Sends event to audio thread via message queue
 *
 * @param event Sound event to play
 */
void audio_play(enum sound_event event);

/**
 * Play a tone at specified frequency for duration
 *
 * @param frequency_hz Frequency in Hz (0 = silence)
 * @param duration_ms Duration in milliseconds
 */
void audio_tone(uint32_t frequency_hz, uint32_t duration_ms);

/**
 * Stop any currently playing sound
 */
void audio_stop(void);

/**
 * Check if audio is currently playing
 *
 * @return true if a sound is playing
 */
bool audio_is_playing(void);

/* Musical note frequencies (Hz) */
#define NOTE_C4  262
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_D5  587
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_G5  784

#endif /* AUDIO_H */
