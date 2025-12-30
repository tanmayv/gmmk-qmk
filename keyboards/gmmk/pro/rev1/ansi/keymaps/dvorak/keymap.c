/* Copyright 2021 Glorious, LLC <salman@pcgamingrace.com>
 * Copyright 2024
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

enum layer_names {
    _BASE,
    _FN,
    _QWERTY,
    _NUM,
    _SYM,
    _MOUSE,
};

enum custom_keycodes {
    MOUSE_TG = SAFE_RANGE,
    MOUSE_SPC,
    WRD_DEL,
};

enum combo_events {
    EU_ESC,
    TH_BSPC,
    HC_SLSH,
    DOTU_BSLS,
    COMBO_LENGTH
};

uint16_t COMBO_LEN = COMBO_LENGTH;

const uint16_t PROGMEM eu_combo[]   = {KC_E, KC_U, COMBO_END};
const uint16_t PROGMEM th_combo[]   = {KC_T, KC_H, COMBO_END};
const uint16_t PROGMEM hc_combo[]   = {KC_H, KC_C, COMBO_END};
const uint16_t PROGMEM dotu_combo[] = {KC_DOT, KC_U, COMBO_END};

combo_t key_combos[] = {
    [EU_ESC]  = COMBO_ACTION(eu_combo),
    [TH_BSPC] = COMBO_ACTION(th_combo),
    [HC_SLSH] = COMBO_ACTION(hc_combo),
    [DOTU_BSLS] = COMBO_ACTION(dotu_combo),
};

// clang-format off
const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
    [_BASE] = LAYOUT(
        KC_ESC,  HYPR(KC_1), HYPR(KC_2), HYPR(KC_3), HYPR(KC_4), HYPR(KC_5), HYPR(KC_6), HYPR(KC_7), HYPR(KC_8), HYPR(KC_9), HYPR(KC_0), HYPR(KC_A), HYPR(KC_B), KC_PSCR,          KC_MUTE,
        KC_GRV,  KC_1,    KC_2,    KC_3,    KC_4,    KC_5,    KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    KC_MINS, KC_EQL,  WRD_DEL,          KC_DEL,
        KC_TAB,  KC_QUOT, KC_COMM, KC_DOT,  KC_P,    KC_Y,    KC_F,    KC_G,    KC_C,    KC_R,    KC_L,    KC_SLSH, KC_EQL,  KC_BSLS,          KC_PGUP,
        KC_LCTL, KC_A,    KC_O,    KC_E,    KC_U,    KC_I,    KC_D,    KC_H,    KC_T,    KC_N,    KC_S,    KC_MINS,          KC_ENT,           KC_PGDN,
        KC_LSFT,          LALT_T(KC_SCLN), LGUI_T(KC_Q), KC_J, KC_K, KC_X, KC_B, KC_M, KC_W, RGUI_T(KC_V), RALT_T(KC_Z),    KC_RSFT, KC_UP,   KC_END,
        MOUSE_TG, KC_LGUI, LT(_NUM, KC_LALT),                      KC_SPC,                             LT(_SYM, KC_RALT), MO(_FN), KC_RCTL, KC_LEFT, KC_DOWN, KC_RGHT
    ),

    [_FN] = LAYOUT(
        _______, KC_MYCM, KC_WHOM, KC_CALC, KC_MSEL, KC_MPRV, KC_MNXT, KC_MPLY, KC_MSTP, KC_MUTE, KC_VOLD, KC_VOLU, _______, _______,          _______,
        _______, KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,   KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_F11,  KC_F12,  _______,          _______,
        _______, DF(_QWERTY), RM_VALU, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, QK_BOOT,          _______,
        _______, _______, RM_VALD, _______, _______, _______, DF(_BASE), _______, _______, _______, _______, _______,          _______,          _______,
        _______,          _______, DF(_QWERTY), _______, _______, _______, NK_TOGG, _______, _______, _______, _______,          _______, RM_NEXT, _______,
        _______, _______, _______,                            _______,                            _______, _______, _______, RM_SPDD, RM_PREV, RM_SPDU
    ),

    [_QWERTY] = LAYOUT(
        KC_ESC,  HYPR(KC_1), HYPR(KC_2), HYPR(KC_3), HYPR(KC_4), HYPR(KC_5), HYPR(KC_6), HYPR(KC_7), HYPR(KC_8), HYPR(KC_9), HYPR(KC_0), HYPR(KC_A), HYPR(KC_B), KC_PSCR,          KC_MUTE,
        KC_GRV,  KC_1,    KC_2,    KC_3,    KC_4,    KC_5,    KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    KC_MINS, KC_EQL,  WRD_DEL,          KC_DEL,
        KC_TAB,  KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,    KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    KC_LBRC, KC_RBRC, KC_BSLS,          KC_PGUP,
        KC_LCTL, KC_A,    KC_S,    KC_D,    KC_F,    KC_G,    KC_H,    KC_J,    KC_K,    KC_L,    KC_SCLN, KC_QUOT,          KC_ENT,           KC_PGDN,
        KC_LSFT,          LALT_T(KC_Z), LCTL_T(KC_X), KC_C, KC_V, KC_B, KC_N, KC_M, KC_COMM, KC_DOT, KC_SLSH,          KC_RSFT, KC_UP,   KC_END,
        MOUSE_TG, KC_LGUI, LT(_NUM, KC_LALT),                      KC_SPC,                             LT(_SYM, KC_RALT), MO(_FN), KC_RCTL, KC_LEFT, KC_DOWN, KC_RGHT
    ),

    [_NUM] = LAYOUT(
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,          _______,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,          _______,
        _______, KC_1,    KC_2,    KC_3,    KC_4,    KC_5,    KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    _______, _______, _______,          _______,
        _______, KC_EXLM, KC_AT,   KC_HASH, KC_DLR,  KC_PERC, _______, _______, _______, _______, _______, _______,          _______,          _______,
        _______,          KC_CIRC, KC_AMPR, KC_ASTR, KC_LPRN, KC_RPRN, _______, _______, _______, _______, _______,          _______, _______, _______,
        _______, _______, _______,                            _______,                            _______, _______, _______, _______, _______, _______
    ),

    [_SYM] = LAYOUT(
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,          _______,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,          _______,
        _______, KC_LBRC, KC_RBRC, KC_LCBR, KC_RCBR, KC_PIPE, KC_BSLS, KC_GRV,  KC_TILD, KC_PLUS, KC_MINS, KC_UNDS, KC_EQL,  _______,          _______,
        _______, KC_LEFT, KC_DOWN, KC_UP,   KC_RGHT, KC_COLN, _______, _______, _______, _______, _______, _______,          _______,          _______,
        _______,          KC_LT,   KC_GT,   KC_QUES, KC_SLSH, KC_DQUO, _______, _______, _______, _______, _______,          _______, _______, _______,
        _______, _______, _______,                            _______,                            _______, _______, _______, _______, _______, _______
    ),

    [_MOUSE] = LAYOUT(
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,          _______,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,          _______,
        _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, MS_RGHT, _______, _______, _______,          _______,
        MOUSE_TG, MS_BTN1, _______, MS_WHLD, MS_WHLU, _______, _______, MS_LEFT, _______, _______, MS_BTN2, _______,          _______,          _______,
        MS_ACL0,          _______, _______, MS_DOWN, MS_UP,   _______, _______, _______, _______, _______, _______,          MS_ACL0, _______, _______,
        _______, _______, _______,                            MOUSE_SPC,                          _______, _______, _______, _______, _______, _______
    ),
};
// clang-format on

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
    switch (keycode) {
        case MOUSE_TG:
            if (record->event.pressed) {
                layer_invert(_MOUSE);
            }
            return false;
        case MOUSE_SPC:
            if (record->event.pressed) {
                tap_code16(MS_BTN1);
                layer_off(_MOUSE);
            }
            return false;
        case WRD_DEL:
            if (record->event.pressed) {
                tap_code16(LALT(KC_BSPC));
            }
            return false;
    }
    return true;
}

void process_combo_event(uint16_t combo_index, bool pressed) {
    if (!pressed) {
        return;
    }
    switch (combo_index) {
        case EU_ESC:
            tap_code(KC_ESC);
            break;
        case TH_BSPC:
            tap_code(KC_BSPC);
            break;
        case HC_SLSH:
            tap_code(KC_SLSH);
            break;
        case DOTU_BSLS:
            tap_code(KC_BSLS);
            break;
    }
}

#if defined(ENCODER_MAP_ENABLE)
const uint16_t PROGMEM encoder_map[][NUM_ENCODERS][NUM_DIRECTIONS] = {
    [_BASE] = { ENCODER_CCW_CW(KC_VOLD, KC_VOLU) },
    [_FN]   = { ENCODER_CCW_CW(KC_TRNS, KC_TRNS) },
    [_QWERTY] = { ENCODER_CCW_CW(KC_VOLD, KC_VOLU) },
    [_NUM]  = { ENCODER_CCW_CW(KC_TRNS, KC_TRNS) },
    [_SYM]  = { ENCODER_CCW_CW(KC_TRNS, KC_TRNS) },
    [_MOUSE]= { ENCODER_CCW_CW(KC_TRNS, KC_TRNS) }
};
#endif

#ifdef RGB_MATRIX_ENABLE
extern led_config_t g_led_config;

static void highlight_layer_keys(uint8_t layer, uint8_t r, uint8_t g, uint8_t b) {
    rgb_matrix_set_color_all(0, 0, 0);
    for (uint8_t row = 0; row < MATRIX_ROWS; row++) {
        for (uint8_t col = 0; col < MATRIX_COLS; col++) {
            uint16_t keycode = pgm_read_word(&keymaps[layer][row][col]);
            if (keycode == KC_TRNS || keycode == KC_NO) {
                continue;
            }
            uint8_t index = g_led_config.matrix_co[row][col];
            if (index != NO_LED) {
                rgb_matrix_set_color(index, r, g, b);
            }
        }
    }
}

bool rgb_matrix_indicators_user(void) {
    switch (get_highest_layer(layer_state)) {
        case _FN:
            highlight_layer_keys(_FN, 128, 0, 255);
            return true;
        case _QWERTY:
            highlight_layer_keys(_QWERTY, 255, 255, 0);
            return true;
        case _NUM:
            highlight_layer_keys(_NUM, 0, 64, 255);
            return true;
        case _SYM:
            highlight_layer_keys(_SYM, 0, 255, 128);
            return true;
        case _MOUSE:
            highlight_layer_keys(_MOUSE, 255, 64, 0);
            return true;
        default:
            return false;
    }
}
#endif
