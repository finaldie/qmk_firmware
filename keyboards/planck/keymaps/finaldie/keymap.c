/* Copyright 2015-2017 Jack Humbert
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include QMK_KEYBOARD_H
#include "muse.h"


enum planck_layers {
  _QWERTY,
  _GAME,
  _COLEMAK,
  _DVORAK,
  _FINAL,
  _MOUSE,
  _LOWER,
  _RAISE,
  _PLOVER,
  _ADJUST
};

enum planck_keycodes {
  QWERTY = SAFE_RANGE,
  COLEMAK,
  DVORAK,
  PLOVER,
  BACKLIT,
  EXT_PLV
};

// switch layers
#define LOWER MO(_LOWER)
#define RAISE MO(_RAISE)

#define FL_1 MO(_GAME)
#define FL_2 MO(_COLEMAK)
#define FL_3 MO(_DVORAK)
#define FL_4 MO(_FINAL)
#define FL_5 MO(_MOUSE)

#define LT_MINS LT(_LOWER,   KC_MINS) // hold: lower; tap: -
#define LT_RBRC LT(_COLEMAK, KC_RBRC) // hold: FL_2;  tap: ]
#define LT_EQL  LT(_RAISE,   KC_EQL)  // hold: raise; tap: =
#define LT_L3_0 LT(_DVORAK,  KC_0)    // hold: FL_3;  tap: 0
#define LT_BSLS LT(_FINAL,   KC_BSLS) // hold: FL_4;  tap: '\'
#define LT_LBRC LT(_GAME,    KC_LBRC) // hold: FL_1;  tap: [

// hold or tap
#define SFT_CAP LSFT_T(KC_CAPS) // hold: left shift; tap: caps
#define CTL_ESC LCTL_T(KC_ESC)  // hold: left  ctrl; tap: esc
#define CTL_BRC RCTL_T(KC_LBRC) // hold: right ctrl; tap: [
#define GUI_1   RGUI_T(KC_1)    // hold: left   GUI; tap: 1

// Tap Dance
enum {
  TD_SCLN_QUOT = 0, // in use
  TD_O_MINS,
  TD_P_EQL,
  TD_RP_UNDER,
};

// Tap Dance Definitions
qk_tap_dance_action_t tap_dance_actions[] = {
  //Tap once for ; twice for '
  [TD_SCLN_QUOT] = ACTION_TAP_DANCE_DOUBLE(KC_SCLN, KC_QUOT),

  [TD_O_MINS]    = ACTION_TAP_DANCE_DOUBLE(KC_O, KC_MINS),
  [TD_P_EQL]     = ACTION_TAP_DANCE_DOUBLE(KC_P, KC_EQL),

  // Tap once for ) twice for _
  [TD_RP_UNDER]  = ACTION_TAP_DANCE_DOUBLE(KC_RPRN, KC_UNDS),

  // Other declarations would go here, separated by commas, if you have them
};

#define TD_SQ TD(TD_SCLN_QUOT)
#define TD_OM TD(TD_O_MINS)
#define TD_PE TD(TD_P_EQL)
#define TD_RU TD(TD_RP_UNDER)

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

/* Qwerty
 * ,-----------------------------------------------------------------------------------.
 * | Tab  |   Q  |   W  |   E  |   R  |   T  |   Y  |   U  |   I  |   O  |   P  | Bksp |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * | Esc  |   A  |   S  |   D  |   F  |   G  |   H  |   J  |   K  |   L  |   ;  |Enter |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * | Shift|   Z  |   X  |   C  |   V  |   B  |   N  |   M  |   ,  |   .  |   /  | FL_4 |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * | Ctrl | Alt  | FL_1 | GUI  |Lower |    Space    |Raise | FL_3 | FL_3 |   _  | FL_2 |
 * `-----------------------------------------------------------------------------------'
 */
[_QWERTY] = LAYOUT_planck_grid(
    KC_TAB,  KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,    KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    KC_BSPC,
    CTL_ESC, KC_A,    KC_S,    KC_D,    KC_F,    KC_G,    KC_H,    KC_J,    KC_K,    KC_L,    TD_SQ,   KC_ENT,
    SFT_CAP, KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,    KC_N,    KC_M,    KC_COMM, KC_DOT,  KC_SLSH, LT_BSLS,
    CTL_BRC, KC_LALT, FL_1,    GUI_1,   LT_MINS, KC_SPC,  KC_SPC,  LT_EQL,  LT_L3_0, FL_3,    KC_UNDS, LT_RBRC
),

/* Game - FL_1
 * ,-----------------------------------------------------------------------------------.
 * | Tab  | Home |  Up  | End  |  F1  |  F2  |  F3  |   7  |   8  |   9  |   +  | Bksp |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * | Esc  |   .  |   ,  |Right |  F4  |  F5  |  F6  |   4  |   5  |   6  |   -  |      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * | Shift|   *  |   .  |   ,  |  F7  |  F8  |  F9  |   1  |   2  |   3  |   /  |      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |      |      |Lower |    Space    |Raise |   0  |   .  |   *  |      |
 * `-----------------------------------------------------------------------------------'
 */
[_GAME] = LAYOUT_planck_grid(
    KC_TAB,  KC_HOME, KC_UP,   KC_END,  KC_F1,   KC_F2,   KC_F3,   KC_7,    KC_8,    KC_9,    KC_PLUS, KC_BSPC,
    _______, KC_DOT,  KC_COMM, KC_COMM, KC_F4,   KC_F5,   KC_F6,   KC_4,    KC_5,    KC_6,    KC_MINS, _______,
    _______, KC_ASTR, KC_DOT,  KC_COMM, KC_F7,   KC_F8,   KC_F9,   KC_1,    KC_2,    KC_3,    _______, _______,
    _______, _______, _______, _______, LT_MINS, KC_SPC,  KC_SPC,  LT_EQL,  KC_0,    KC_DOT,  KC_ASTR, _______
),

/* Colemak - FL_2
 * ,-----------------------------------------------------------------------------------.
 * | Tab  |      |      |      |      |      |   ^  |   &  |   *  |   _  |   +  | Bksp |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * | Esc  |      |      |      |      |      |      |      |      |  Up  |      |      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * | Shift|      |      |      |      |      |      |      | Left | Down |Right |      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |      |      |Lower |    Space    |Raise |      |      |      |      |
 * `-----------------------------------------------------------------------------------'
 */
[_COLEMAK] = LAYOUT_planck_grid(
    KC_TAB,  _______, _______, _______, _______, _______, KC_CIRC, KC_AMPR, KC_ASTR, KC_UNDS, KC_PLUS, KC_BSPC,
    _______, _______, _______, _______, _______, _______, _______, _______, _______, KC_UP,   _______, _______,
    KC_LSFT, _______, _______, _______, _______, _______, _______, _______, KC_LEFT, KC_DOWN, KC_RIGHT,_______,
    _______, _______, _______, _______, LOWER,   KC_SPC,  KC_SPC,  RAISE,   _______, _______, _______, _______
),

/* Dvorak - FL_3
 * ,-----------------------------------------------------------------------------------.
 * | Tab  |   1  |   2  |   3  |   4  |   5  |   6  |   7  |      |   -  |   +  | Bksp |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * | Esc  |   4  |   5  |   6  |   0  |      |      |      |   *  |   /  |   =  |  |   |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * | Shift|   7  |   8  |   9  |   *  |      |      |   -  |   >  |      |      |      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |      |      |Lower |    Space    |Raise |      |      |      |      |
 * `-----------------------------------------------------------------------------------'
 */
[_DVORAK] = LAYOUT_planck_grid(
    _______, KC_1,    KC_2,    KC_3,    KC_4,    KC_5,    KC_6,    KC_7,    _______, KC_MINS, KC_PLUS, KC_BSPC,
    _______, KC_4,    KC_5,    KC_6,    KC_0,    _______, _______, _______, KC_ASTR, KC_SLSH, KC_EQL,  KC_PIPE,
    KC_LSFT, KC_7,    KC_8,    KC_9,    KC_ASTR, _______, _______, KC_MINS, KC_GT,   _______, _______, _______,
    _______, _______, _______, _______, LOWER,   KC_SPC,  KC_SPC,  RAISE,   _______, _______, _______, _______
),

/* Final - FL_4
 * ,-----------------------------------------------------------------------------------.
 * |   ~  |   !  |   @  |   #  |   $  |   %  |   *  |   (  |   )  |   {  |   }  | Bksp |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * | Esc  |      |  UP  |      |      |      |      |   =  |   :  |   "  |   :  |Enter |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      | Left | Down |Right |      |      |      |   <  |   >  |   ?  |      |      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |      |      |      |             |      |      |      |      |      |
 * `-----------------------------------------------------------------------------------'
 */
[_FINAL] = LAYOUT_planck_grid(
    KC_TILD, KC_EXLM, KC_AT,   KC_HASH, KC_DLR,  KC_PERC, KC_ASTR, KC_LPRN,    KC_RPRN,    KC_LCBR, KC_RCBR,  KC_BSPC,
    _______, _______, KC_UP,   _______, _______, _______, _______, KC_EQL,     KC_COLN,    KC_DQUO, KC_COLN,  KC_ENT,
    _______, KC_LEFT, KC_DOWN, KC_RIGHT,_______, _______, _______, KC_LT,      KC_GT,      KC_QUES, _______,  _______,
    _______, _______, _______, _______, _______, _______, _______, _______,    _______,    _______, _______,  _______
),

/* Mouse - FL_5
 * ,-----------------------------------------------------------------------------------.
 * | Tab  | WH_L | MS_U | WH_R | WH_U |      |   ^  |   &  |   *  |   _  |   +  | Bksp |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * | Esc  | MS_L | MS_D | MS_R | WH_D |      |      |MS_B1 |MS_B2 | ACL0 | ACL1 |      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * | Shift|      |      |      |      |      |      | ACL0 | ACL1 | ACL2 |      |      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |      |      |Lower |    Space    |Raise |      |      |      |      |
 * `-----------------------------------------------------------------------------------'
 */
[_MOUSE] = LAYOUT_planck_grid(
    KC_TAB,  KC_WH_L, KC_MS_U, KC_WH_R, KC_WH_U, _______, KC_CIRC, KC_AMPR, KC_ASTR, KC_UNDS, KC_PLUS, KC_BSPC,
    _______, KC_MS_L, KC_MS_D, KC_MS_R, KC_WH_D, _______, _______, KC_BTN1, KC_BTN2, KC_ACL0, KC_ACL1, _______,
    KC_LSFT, _______, _______, _______, _______, _______, _______, KC_ACL0, KC_ACL1, KC_ACL2, _______, _______,
    _______, _______, _______, _______, LOWER,   KC_SPC,  KC_SPC,  RAISE,   _______, _______, _______, _______
),

/* Lower
 * ,-----------------------------------------------------------------------------------.
 * | Tab  |   1  |   2  |   3  |   4  |   5  |   6  |   7  |      |   -  |   =  | Bksp |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * | Esc  |   4  |   5  |   6  |   0  |      | Left | Down |  Up  |Right |   '  |      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |   7  |   8  |   9  |   *  |      | Home | PGUP | PGDN | End  |  Up  |      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |      |      |      |             |      |      | Left | Down |Right |
 * `-----------------------------------------------------------------------------------'
 */
[_LOWER] = LAYOUT_planck_grid(
    _______, KC_1,    KC_2,    KC_3,    KC_4,    KC_5,    KC_6,    KC_7,       _______,    KC_MINS, KC_EQL,  KC_BSPC,
    _______, KC_4,    KC_5,    KC_6,    KC_0,    _______, KC_LEFT, KC_DOWN,    KC_UP,      KC_RIGHT,KC_QUOT, _______,
    _______, KC_7,    KC_8,    KC_9,    KC_ASTR, _______, KC_HOME, KC_PGUP,    KC_PGDN,    KC_END,  KC_UP,   _______,
    _______, _______, _______, _______, _______, _______, _______, _______,    _______,    KC_LEFT, KC_DOWN, KC_RIGHT
),

/* Raise
 * ,-----------------------------------------------------------------------------------.
 * |   `  | WH_L | MS_U | WH_R | WH_U |      |   ^  |   &  |   *  |   _  |   +  | Bksp |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      | MS_L | MS_D | MS_R | WH_D |      |      |MS_B1 |MS_B2 | ACL0 | ACL1 |      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |      |      |      |      |      | ACL0 | ACL1 | ACL2 |      |      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |      |      |      |             |      |      |      |      | ACL0 |
 * `-----------------------------------------------------------------------------------'
 */
[_RAISE] = LAYOUT_planck_grid(
    KC_GRV,  KC_WH_L, KC_MS_U, KC_WH_R, KC_WH_U, _______, KC_CIRC, KC_AMPR, KC_ASTR, KC_UNDS, KC_PLUS, KC_BSPC,
    _______, KC_MS_L, KC_MS_D, KC_MS_R, KC_WH_D, _______, _______, KC_BTN1, KC_BTN2, KC_ACL0, KC_ACL1, _______,
    _______, _______, _______, _______, _______, _______, _______, KC_ACL0, KC_ACL1, KC_ACL2, _______, _______,
    _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, KC_ACL0
),

/* Plover layer (http://opensteno.org)
 * ,-----------------------------------------------------------------------------------.
 * |   #  |   #  |   #  |   #  |   #  |   #  |   #  |   #  |   #  |   #  |   #  |   #  |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |   S  |   T  |   P  |   H  |   *  |   *  |   F  |   P  |   L  |   T  |   D  |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |   S  |   K  |   W  |   R  |   *  |   *  |   R  |   B  |   G  |   S  |   Z  |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * | Exit |      |      |   A  |   O  |             |   E  |   U  |      |      |      |
 * `-----------------------------------------------------------------------------------'
 */
[_PLOVER] = LAYOUT_planck_grid(
    KC_1,    KC_1,    KC_1,    KC_1,    KC_1,    KC_1,    KC_1,    KC_1,    KC_1,    KC_1,    KC_1,    KC_1   ,
    XXXXXXX, KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,    KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    KC_LBRC,
    XXXXXXX, KC_A,    KC_S,    KC_D,    KC_F,    KC_G,    KC_H,    KC_J,    KC_K,    KC_L,    KC_SCLN, KC_QUOT,
    EXT_PLV, XXXXXXX, XXXXXXX, KC_C,    KC_V,    XXXXXXX, XXXXXXX, KC_N,    KC_M,    XXXXXXX, XXXXXXX, XXXXXXX
),

/* Adjust (Lower + Raise)
 *                      v------------------------RGB CONTROL--------------------v
 * ,-----------------------------------------------------------------------------------.
 * |      | Reset|Debug | RGB  |RGBMOD| HUE+ | HUE- | SAT+ | SAT- |BRGTH+|BRGTH-|  Del |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |MUSmod|Aud on|Audoff|AGnorm|AGswap|Qwerty|Colemk|Dvorak|Plover|      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |Voice-|Voice+|Mus on|Musoff|MIDIon|MIDIof|TermOn|TermOf|      |      |      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |      |      |      |             |      |      |      |      |      |
 * `-----------------------------------------------------------------------------------'
 */
[_ADJUST] = LAYOUT_planck_grid(
    _______, RESET,   DEBUG,   RGB_TOG, RGB_MOD, RGB_HUI, RGB_HUD, RGB_SAI, RGB_SAD,  RGB_VAI, RGB_VAD, KC_DEL ,
    _______, _______, MU_MOD,  AU_ON,   AU_OFF,  AG_NORM, AG_SWAP, QWERTY,  COLEMAK,  DVORAK,  PLOVER,  _______,
    _______, MUV_DE,  MUV_IN,  MU_ON,   MU_OFF,  MI_ON,   MI_OFF,  TERM_ON, TERM_OFF, _______, _______, _______,
    _______, _______, _______, _______, _______, _______, _______, _______, _______,  _______, _______, _______
)

};

#ifdef AUDIO_ENABLE
  float plover_song[][2]     = SONG(PLOVER_SOUND);
  float plover_gb_song[][2]  = SONG(PLOVER_GOODBYE_SOUND);
#endif

layer_state_t layer_state_set_user(layer_state_t state) {
  return update_tri_layer_state(state, _LOWER, _RAISE, _ADJUST);
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
    case QWERTY:
      if (record->event.pressed) {
        print("mode just switched to qwerty and this is a huge string\n");
        set_single_persistent_default_layer(_QWERTY);
      }
      return false;
      break;
    case COLEMAK:
      if (record->event.pressed) {
        set_single_persistent_default_layer(_COLEMAK);
      }
      return false;
      break;
    case DVORAK:
      if (record->event.pressed) {
        set_single_persistent_default_layer(_DVORAK);
      }
      return false;
      break;
    case BACKLIT:
      if (record->event.pressed) {
        register_code(KC_RSFT);
        #ifdef BACKLIGHT_ENABLE
          backlight_step();
        #endif
        #ifdef KEYBOARD_planck_rev5
          writePinLow(E6);
        #endif
      } else {
        unregister_code(KC_RSFT);
        #ifdef KEYBOARD_planck_rev5
          writePinHigh(E6);
        #endif
      }
      return false;
      break;
    case PLOVER:
      if (record->event.pressed) {
        #ifdef AUDIO_ENABLE
          stop_all_notes();
          PLAY_SONG(plover_song);
        #endif
        layer_off(_RAISE);
        layer_off(_LOWER);
        layer_off(_ADJUST);
        layer_on(_PLOVER);
        if (!eeconfig_is_enabled()) {
            eeconfig_init();
        }
        keymap_config.raw = eeconfig_read_keymap();
        keymap_config.nkro = 1;
        eeconfig_update_keymap(keymap_config.raw);
      }
      return false;
      break;
    case EXT_PLV:
      if (record->event.pressed) {
        #ifdef AUDIO_ENABLE
          PLAY_SONG(plover_gb_song);
        #endif
        layer_off(_PLOVER);
      }
      return false;
      break;
  }
  return true;
}

bool muse_mode = false;
uint8_t last_muse_note = 0;
uint16_t muse_counter = 0;
uint8_t muse_offset = 70;
uint16_t muse_tempo = 50;

void encoder_update(bool clockwise) {
  if (muse_mode) {
    if (IS_LAYER_ON(_RAISE)) {
      if (clockwise) {
        muse_offset++;
      } else {
        muse_offset--;
      }
    } else {
      if (clockwise) {
        muse_tempo+=1;
      } else {
        muse_tempo-=1;
      }
    }
  } else {
    if (clockwise) {
      #ifdef MOUSEKEY_ENABLE
        tap_code(KC_MS_WH_DOWN);
      #else
        tap_code(KC_PGDN);
      #endif
    } else {
      #ifdef MOUSEKEY_ENABLE
        tap_code(KC_MS_WH_UP);
      #else
        tap_code(KC_PGUP);
      #endif
    }
  }
}

void dip_switch_update_user(uint8_t index, bool active) {
    switch (index) {
        case 0: {
#ifdef AUDIO_ENABLE
            static bool play_sound = false;
#endif
            if (active) {
#ifdef AUDIO_ENABLE
                if (play_sound) { PLAY_SONG(plover_song); }
#endif
                layer_on(_ADJUST);
            } else {
#ifdef AUDIO_ENABLE
                if (play_sound) { PLAY_SONG(plover_gb_song); }
#endif
                layer_off(_ADJUST);
            }
#ifdef AUDIO_ENABLE
            play_sound = true;
#endif
            break;
        }
        case 1:
            if (active) {
                muse_mode = true;
            } else {
                muse_mode = false;
            }
    }
}

void matrix_scan_user(void) {
#ifdef AUDIO_ENABLE
    if (muse_mode) {
        if (muse_counter == 0) {
            uint8_t muse_note = muse_offset + SCALE[muse_clock_pulse()];
            if (muse_note != last_muse_note) {
                stop_note(compute_freq_for_midi_note(last_muse_note));
                play_note(compute_freq_for_midi_note(muse_note), 0xF);
                last_muse_note = muse_note;
            }
        }
        muse_counter = (muse_counter + 1) % muse_tempo;
    } else {
        if (muse_counter) {
            stop_all_notes();
            muse_counter = 0;
        }
    }
#endif
}

bool music_mask_user(uint16_t keycode) {
  switch (keycode) {
    case RAISE:
    case LOWER:
      return false;
    default:
      return true;
  }
}
