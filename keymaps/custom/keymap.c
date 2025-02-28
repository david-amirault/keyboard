// Copyright 2023 QMK
// SPDX-License-Identifier: GPL-2.0-or-later

#include QMK_KEYBOARD_H

enum layers {
    DEF,
    NUM,
    NMX,
    SYS,
    PDS,
    SYM,
    SMX,
    NAV,
    TAB,
};

#define TOP_G LT(PDS,KC_G)
#define TOP_F LT(TAB,KC_F)
#define TOP_Y LT(NMX,KC_Y)
#define TOP_O LT(SMX,KC_O)
#define TOP_L LT(NUM,KC_L)
#define TOP_U LT(SYM,KC_U)

#define HOME_T LT(SYS,KC_T)
#define HOME_N LT(NAV,KC_N)
#define HOME_S LT(NMX,KC_S)
#define HOME_A LT(SMX,KC_A)
#define HOME_R LT(NUM,KC_R)
#define HOME_E LT(SYM,KC_E)

#define BOTM_D LGUI_T(KC_D)
#define BOTM_H RGUI_T(KC_H)
#define BOTM_V LALT_T(KC_V)
#define BOTM_CM RALT_T(KC_COMM)
#define BOTM_X LCTL_T(KC_X)
#define BOTM_DT RCTL_T(KC_DOT)

#define THMB_SP LSFT_T(KC_SPC)
#define THMB_BS RSFT_T(KC_BSPC)

#define EN_DASH A(KC_MINS)
#define EM_DASH S(A(KC_MINS))
#define NOT_EQL A(KC_EQL)
#define PL_MINS S(A(KC_EQL))

#define TM_PRFX C(KC_SPC)

enum custom_keycodes {
    // Special keycodes.
    CAPSWRD = SAFE_RANGE,
    REPEAT,
    MAGIC,

    // Macros.
    XCH,
    PHA,
    MKT,
    INF,

    PD_DF,
    PD_SER,
    PD_TS,
    PD_TD,

    PD_IDX,
    PD_QCUT,
    GROUPBY,
    DROPNA,

    TM_NXTW,
    TM_PRVW,
    TM_NXTS,
    TM_PRVS,

    TM_NAME,
    TM_SPWN,
    TM_FORK,
    TM_VIZ,

    VIM_Q,
    VIM_WQ,
    VIM_W,

    LOGIN,
    EMAIL,
};

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
    [DEF] = LAYOUT_split_3x6_3(
        _______, KC_Q,    TOP_L,   TOP_Y,   TOP_G,   KC_K,                      KC_B,    TOP_F,   TOP_O,   TOP_U,   KC_J,    _______,
        _______, KC_C,    HOME_R,  HOME_S,  HOME_T,  KC_M,                      MAGIC,   HOME_N,  HOME_A,  HOME_E,  KC_I,    _______,
        _______, KC_Z,    BOTM_X,  BOTM_V,  BOTM_D,  KC_W,                      KC_P,    BOTM_H,  BOTM_CM, BOTM_DT, KC_QUOT, _______,
                                            REPEAT,  THMB_SP, KC_TAB,  KC_ENT,  THMB_BS, KC_ESC
    ),

    [NUM] = LAYOUT_split_3x6_3(
        _______, _______, _______, _______, _______, _______,                   XXXXXXX, KC_8,    KC_9,    KC_0,    XXXXXXX, _______,
        _______, _______, _______, _______, _______, _______,                   _______, KC_1,    KC_2,    KC_3,    KC_4,    _______,
        _______, _______, _______, _______, _______, _______,                   XXXXXXX, KC_5,    KC_6,    KC_7,    XXXXXXX, _______,
                                            _______, _______, _______, _______, _______, _______
    ),

    [NMX] = LAYOUT_split_3x6_3(
        _______, _______, _______, _______, _______, _______,                   XXXXXXX, KC_ASTR, KC_LPRN, KC_RPRN, XXXXXXX, _______,
        _______, _______, _______, _______, _______, _______,                   _______, KC_EXLM, KC_AT,   KC_HASH, KC_DLR,  _______,
        _______, _______, _______, _______, _______, _______,                   XXXXXXX, KC_PERC, KC_CIRC, KC_AMPR, XXXXXXX, _______,
                                            _______, _______, _______, _______, _______, _______
    ),

    [SYS] = LAYOUT_split_3x6_3(
        _______, _______, _______, _______, _______, QK_BOOT,                   XXXXXXX, KC_MUTE, KC_VOLD, KC_VOLU, XXXXXXX, _______,
        _______, _______, _______, _______, _______, _______,                   _______, XCH,     PHA,     MKT,     INF,     _______,
        _______, _______, _______, _______, _______, _______,                   XXXXXXX, KC_PSCR, KC_BRID, KC_BRIU, XXXXXXX, _______,
                                            _______, _______, _______, _______, _______, _______
    ),

    [PDS] = LAYOUT_split_3x6_3(
        _______, _______, _______, _______, _______, _______,                   XXXXXXX, GROUPBY, DROPNA,  PD_QCUT, XXXXXXX, _______,
        _______, _______, _______, _______, _______, _______,                   _______, PD_DF,   PD_SER,  PD_TS,   PD_TD,   _______,
        _______, _______, _______, _______, _______, _______,                   XXXXXXX, PD_IDX,  LOGIN,   EMAIL,   XXXXXXX, _______,
                                            _______, _______, _______, _______, _______, _______
    ),

    [SYM] = LAYOUT_split_3x6_3(
        _______, XXXXXXX, KC_LBRC, KC_RBRC, KC_GRV,  XXXXXXX,                   _______, _______, _______, _______, _______, _______,
        _______, KC_BSLS, KC_SLSH, KC_MINS, KC_SCLN, CAPSWRD,                   _______, _______, _______, _______, _______, _______,
        _______, XXXXXXX, NOT_EQL, EN_DASH, KC_EQL,  XXXXXXX,                   _______, _______, _______, _______, _______, _______,
                                            _______, _______, _______, _______, _______, _______
    ),

    [SMX] = LAYOUT_split_3x6_3(
        _______, XXXXXXX, KC_LCBR, KC_RCBR, KC_TILD, XXXXXXX,                   _______, _______, _______, _______, _______, _______,
        _______, KC_PIPE, KC_QUES, KC_UNDS, KC_COLN, KC_CAPS,                   _______, _______, _______, _______, _______, _______,
        _______, XXXXXXX, PL_MINS, EM_DASH, KC_PLUS, XXXXXXX,                   _______, _______, _______, _______, _______, _______,
                                            _______, _______, _______, _______, _______, _______
    ),

    [NAV] = LAYOUT_split_3x6_3(
        _______, XXXXXXX, KC_HOME, KC_END,  KC_Q,    XXXXXXX,                   _______, _______, _______, _______, _______, _______,
        _______, KC_LEFT, KC_UP,   KC_DOWN, KC_RGHT, KC_INS,                    _______, _______, _______, _______, _______, _______,
        _______, XXXXXXX, KC_PGUP, KC_PGDN, KC_DEL,  XXXXXXX,                   _______, _______, _______, _______, _______, _______,
                                            _______, _______, _______, _______, _______, _______
    ),

    [TAB] = LAYOUT_split_3x6_3(
        _______, XXXXXXX, VIM_W,   VIM_WQ,  VIM_Q,   XXXXXXX,                   _______, _______, _______, _______, _______, _______,
        _______, TM_PRVW, TM_PRVS, TM_NXTS, TM_NXTW, TM_VIZ,                    _______, _______, _______, _______, _______, _______,
        _______, XXXXXXX, TM_FORK, TM_SPWN, TM_NAME, XXXXXXX,                   _______, _______, _______, _______, _______, _______,
                                            _______, _______, _______, _______, _______, _______
    ),
};

bool cw_enabled = false;

bool should_cw_continue(uint16_t keycode) {
    switch (keycode) {
        case KC_A ... KC_Z:
        case KC_MINS:
        case KC_1 ... KC_0:
        case KC_BSPC:
        case KC_DEL:
        case KC_UNDS:
            return true;
        default:
            return false;
    }
}

bool should_cw_shift(uint16_t keycode) {
    switch (keycode) {
        case KC_A ... KC_Z:
        case KC_MINS:
            return true;
        default:
            return false;
    }
}

uint16_t get_cw_keycode(uint16_t keycode) {
    if (cw_enabled) {
        cw_enabled = should_cw_continue(keycode);
        if (should_cw_shift(keycode)) {
            return S(keycode);
        }
    }
    return keycode;
}

void cw_tap(uint16_t keycode) {
    tap_code16(get_cw_keycode(keycode));
}

void cw_prep(uint16_t keycode) {
    if (keycode != get_cw_keycode(keycode)) {
        // Apply shift to the next key.
        add_weak_mods(MOD_BIT(KC_LSFT));
    }
}

uint16_t get_magic_keycode(uint16_t keycode) {
    if (IS_LAYER_ON(PDS) || IS_LAYER_ON(SYS)) {
        switch (keycode) {
            case KC_1 ... KC_0:
            case KC_DQUO:
            case KC_RPRN:
                return KC_COLN;
            default:
                return KC_SLSH;
        }
    }

    if (IS_LAYER_ON(NMX)) {
        switch (keycode) {
            case KC_EXLM:
                return KC_EQL;
            default:
                return KC_DQUO;
        }
    }

    if (IS_LAYER_ON(NUM)) {
        switch (keycode) {
            case KC_1 ... KC_0:
                return KC_DOT;
            default:
                return KC_MINS;
        }
    }

    switch (keycode) {
        // Letters.
        case KC_Q:
            cw_tap(KC_U);
            cw_tap(KC_E);
            return KC_N;
        case KC_C:
            return KC_Q;
        case KC_Z:
        case KC_X:
            return KC_C;
        case KC_L:
            return KC_R;
        case KC_R:
            return KC_L;
        case KC_Y:
        case KC_V:
            return KC_S;
        case KC_S:
            return KC_Y;
        case KC_G:
        case KC_M:
            return KC_D;
        case KC_T:
        case KC_K:
            return KC_W;
        case KC_D:
            return KC_G;
        case KC_W:
            cw_tap(KC_T);
            return KC_H;

        case KC_B:
        case KC_N:
            return KC_P;
        case KC_P:
            return KC_B;
        case KC_O:
            return KC_A;
        case KC_A:
            return KC_COMM;
        case KC_U:
        case KC_J:
        case KC_DOT:
            return KC_E;
        case KC_E:
        case KC_RPRN:
        case KC_BSLS:
        case KC_GRV:
            return KC_DOT;
        case KC_I:
            return KC_QUOT;

        // Punctuation.
        case KC_TAB:
            return KC_SPC;
        case KC_SPC:
            cw_tap(KC_T);
            cw_tap(KC_H);
            return KC_E;
        case KC_COMM:
            cw_tap(KC_SPC);
            cw_tap(KC_A);
            cw_tap(KC_N);
            return KC_D;
        case KC_BSPC:
            return KC_ESC;
        case KC_ENT:
            return KC_BSPC;

        // Symbols.
        case KC_MINS:
        case KC_UNDS:
            return KC_RABK;
        case KC_RABK:
        case KC_LABK:
            return KC_EQL;
        case KC_1 ... KC_0:
            return KC_MINS;
        case KC_LPRN:
            return KC_LBRC;
        case KC_LBRC:
        case KC_LCBR:
        case KC_EQL:
            return KC_DQUO;
        case KC_DQUO:
            return KC_RBRC;
        case KC_RBRC:
        case KC_RCBR:
            return KC_RPRN;

        default:
            return KC_NO;
    }
}

static bool on_left_hand(keypos_t pos) {
#ifdef SPLIT_KEYBOARD
    return pos.row < MATRIX_ROWS / 2;
#else
    return (MATRIX_COLS > MATRIX_ROWS) ? pos.col < MATRIX_COLS / 2
                                       : pos.row < MATRIX_ROWS / 2;
#endif
}

bool on_same_hands(keyrecord_t* record, keyrecord_t* other_record) {
    return on_left_hand(record->event.key)
        == on_left_hand(other_record->event.key);
}

uint16_t get_primary_keycode(uint16_t keycode) {
    const bool is_lt = IS_QK_LAYER_TAP(keycode);
    const bool is_tap_hold = is_lt || IS_QK_MOD_TAP(keycode);
    const uint16_t basic = keycode & (is_tap_hold ? 0xff : S(0xff));
    const bool shifted = get_mods() & MOD_MASK_SHIFT;
    switch (basic) {
        case KC_A ... KC_Z:
            return basic;
        default:
            return shifted ? S(basic) : basic;
    }
}

uint16_t press_timer = 0;
uint16_t repeat_keycode = KC_NO;
uint16_t magic_keycode = KC_NO;
uint16_t prev_keycode = KC_NO;
uint16_t queued_keycode = KC_NO;
keyrecord_t queued_record;

bool process_record_user(uint16_t keycode, keyrecord_t* record) {
    const bool is_lt = IS_QK_LAYER_TAP(keycode);
    const bool is_tap_hold = is_lt || IS_QK_MOD_TAP(keycode);
    const bool is_held = record->tap.count == 0;
    const bool is_tapped = !(is_tap_hold && is_held);

    if (record->event.pressed) {
        press_timer = record->event.time;

        // Send queued tap code when we register a same-handed keypress.
        if (queued_keycode != KC_NO) {
            if (is_tapped && on_same_hands(record, &queued_record)) {
                cw_tap(queued_keycode);
            }
            queued_keycode = KC_NO;
        }

        switch (keycode) {
            // Special keycodes.
            case CAPSWRD:
                cw_enabled = !cw_enabled;
                return true;
            case REPEAT:
                repeat_keycode = get_cw_keycode(prev_keycode);
                register_code16(repeat_keycode);
                return true;
            case MAGIC:
                prev_keycode = get_magic_keycode(prev_keycode);
                magic_keycode = get_cw_keycode(prev_keycode);
                register_code16(magic_keycode);
                return true;

            // Macros.
            case XCH:
                SEND_STRING("xch");
                return true;
            case PHA:
                SEND_STRING("pha");
                return true;
            case MKT:
                SEND_STRING("mkt");
                return true;
            case INF:
                SEND_STRING("inf");
                return true;

            case PD_DF:
                SEND_STRING("pd.DataFrame");
                return true;
            case PD_SER:
                SEND_STRING("pd.Series");
                return true;
            case PD_TS:
                SEND_STRING("pd.Timestamp");
                return true;
            case PD_TD:
                SEND_STRING("pd.Timedelta");
                return true;

            case PD_IDX:
                SEND_STRING("pd.Index");
                return true;
            case PD_QCUT:
                SEND_STRING("pd.qcut");
                return true;
            case GROUPBY:
                SEND_STRING("groupby");
                return true;
            case DROPNA:
                SEND_STRING("dropna");
                return true;

            case TM_NXTW:
                tap_code16(TM_PRFX);
                tap_code16(KC_N);
                return true;
            case TM_PRVW:
                tap_code16(TM_PRFX);
                tap_code16(KC_P);
                return true;
            case TM_NXTS:
                tap_code16(TM_PRFX);
                tap_code16(KC_RPRN);
                return true;
            case TM_PRVS:
                tap_code16(TM_PRFX);
                tap_code16(KC_LPRN);
                return true;

            case TM_NAME:
                tap_code16(TM_PRFX);
                tap_code16(KC_COMM);
                return true;
            case TM_SPWN:
                tap_code16(TM_PRFX);
                tap_code16(KC_C);
                return true;
            case TM_FORK:
                tap_code16(TM_PRFX);
                tap_code16(KC_E);
                return true;
            case TM_VIZ:
                tap_code16(TM_PRFX);
                tap_code16(KC_LBRC);
                return true;

            case VIM_Q:
                SEND_STRING("\e" SS_DELAY(40) ":q\n");
                return true;
            case VIM_WQ:
                SEND_STRING("\e" SS_DELAY(40) ":wq\n");
                return true;
            case VIM_W:
                SEND_STRING("\e" SS_DELAY(40) ":w\n");
                return true;

            case LOGIN:
                SEND_STRING("david.j.amirault");
                return true;
            case EMAIL:
                SEND_STRING("@gmail.com");
                return true;
        }

        if (is_tapped) {
            prev_keycode = get_primary_keycode(keycode);
            cw_prep(prev_keycode);
        }

        // When an LT key is held, queue it for potential tapping.
        if (is_lt && is_held) {
            queued_keycode = get_primary_keycode(keycode);
            queued_record = *record;
        }

    } else {
        switch (keycode) {
            case REPEAT:
                unregister_code16(repeat_keycode);
                return true;
            case MAGIC:
                unregister_code16(magic_keycode);
                return true;
            case CAPSWRD:
            case XCH ... EMAIL:
                return true;
        }

        // When a queued LT key is raised, unqueue it.
        if (is_lt && queued_keycode == get_primary_keycode(keycode)) {
            queued_keycode = KC_NO;
        }
    }

    return true;
}

uint16_t get_tapping_term(uint16_t keycode, keyrecord_t *record) {
    switch (keycode) {
        case THMB_SP:
        case THMB_BS:
            return TAPPING_TERM - 20;
        case BOTM_D:
        case BOTM_H:
            return TAPPING_TERM + 40;
        default:
            // Increase tapping term if a key was pressed recently.
            if (timer_elapsed(press_timer) < TAPPING_TERM) {
                return TAPPING_TERM + 40;
            }
            return TAPPING_TERM;
    }
}
