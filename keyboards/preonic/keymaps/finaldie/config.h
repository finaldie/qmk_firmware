#pragma once

#ifdef AUDIO_ENABLE
    #define STARTUP_SONG SONG(PREONIC_SOUND)
    // #define STARTUP_SONG SONG(NO_SOUND)

    #define DEFAULT_LAYER_SONGS { SONG(QWERTY_SOUND), \
                                  SONG(COLEMAK_SOUND), \
                                  SONG(DVORAK_SOUND) \
                                }
#endif

#define MUSIC_MASK (keycode != KC_NO)

/*
 * MIDI options
 */

/* enable basic MIDI features:
   - MIDI notes can be sent when in Music mode is on
*/

#define MIDI_BASIC

/* enable advanced MIDI features:
   - MIDI notes can be added to the keymap
   - Octave shift and transpose
   - Virtual sustain, portamento, and modulation wheel
   - etc.
*/
//#define MIDI_ADVANCED

/* override number of MIDI tone keycodes (each octave adds 12 keycodes and allocates 12 bytes) */
//#define MIDI_TONE_KEYCODE_OCTAVES 2

/************************
 * My own configurations
 ************************/
// Tap hold
#define TAPPING_TERM 175
//#define PERMISSIVE_HOLD

#define TAPPING_TOGGLE 2

// Mouse
#define MK_3_SPEED
#define MK_MOMENTARY_ACCEL

#define MK_C_OFFSET_UNMOD 16   // default is 16
#define MK_C_INTERVAL_UNMOD 16 // default is 16
#define MK_C_OFFSET_0 4        // default is 1
#define MK_C_INTERVAL_0 16     // default is 32
#define MK_C_OFFSET_1 8        // default is 4
#define MK_C_INTERVAL_1 16     // default is 16

#define MK_W_OFFSET_UNMOD 1    // default is 1
#define MK_W_INTERVAL_UNMOD 40 // default is 40
#define MK_W_OFFSET_0 1        // default is 1
#define MK_W_INTERVAL_0 200    // default is 360
