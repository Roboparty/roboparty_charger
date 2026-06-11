

// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 zz020




#include "key.h"

typedef enum {
    KEY_IDLE,               // ??
    KEY_PRESS_WAIT,         // ??????
    KEY_PRESSED,            // ???(??)
    KEY_RELEASE_WAIT,       // ??????
    KEY_WAIT_DOUBLE,        // ??????
    KEY_DOUBLE_DONE,        // ????,????
    KEY_LONG_PRESSED        // ?????,????(??)
} KeyState;

static KeyState keyState = KEY_IDLE;
static uint32_t lastTick = 0;
static uint32_t pressTick = 0;   // ????????

// ????(??:ms)
#define DEBOUNCE_TIME      20
#define DOUBLE_CLICK_TIME  300
#define LONG_PRESS_TIME    1000    // ??????

// ????
uint8_t KEY1_EVENT = 0;
uint8_t KEY2_EVENT = 0;
uint8_t KEY_LONG_EVENT = 0;
uint8_t KEY1_VAL = 1;
uint8_t KEY2_VAL = 1;

void KEY_Scan(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t level = HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin);
    uint8_t isPressed = (level == GPIO_PIN_RESET);

    switch (keyState)
    {
        case KEY_IDLE:
            if (isPressed) {
                keyState = KEY_PRESS_WAIT;
                lastTick = now;
            }
            break;

        case KEY_PRESS_WAIT:
            if (isPressed) {
                if ((now - lastTick) >= DEBOUNCE_TIME) {
                    pressTick = now;          // ????????
                    keyState = KEY_PRESSED;
                }
            } else {
                keyState = KEY_IDLE;          // ??????,????
            }
            break;

        case KEY_PRESSED:
            if (!isPressed) {
                // ??,????????(????????????)
                keyState = KEY_RELEASE_WAIT;
                lastTick = now;
            } else {
                // ????
                if ((now - pressTick) >= LONG_PRESS_TIME) {
                    // ??????
                    KEY_LONG_EVENT = 1;
                    keyState = KEY_LONG_PRESSED;   // ??????????
                    lastTick = now;
                }
            }
            break;

        case KEY_RELEASE_WAIT:
            if (!isPressed) {
                if ((now - lastTick) >= DEBOUNCE_TIME) {
                    // ????,????????
                    keyState = KEY_WAIT_DOUBLE;
                    lastTick = now;
                }
            } else {
                // ????????,????????
                keyState = KEY_PRESS_WAIT;
                lastTick = now;
            }
            break;

        case KEY_WAIT_DOUBLE:
            if (isPressed) {
                if ((now - lastTick) >= DEBOUNCE_TIME) {
                    // ????????,?????
                    KEY2_EVENT = 1;
                    keyState = KEY_DOUBLE_DONE;
                    lastTick = now;
                }
            } else {
                if ((now - lastTick) >= DOUBLE_CLICK_TIME) {
                    // ????????,?????
                    KEY1_EVENT = 1;
                    keyState = KEY_IDLE;
                }
            }
            break;

        case KEY_DOUBLE_DONE:
            // ?????????
            if (!isPressed) {
                if ((now - lastTick) >= DEBOUNCE_TIME) {
                    keyState = KEY_IDLE;
                }
            } else {
                lastTick = now;
            }
            break;

        case KEY_LONG_PRESSED:
            // ????????
            if (!isPressed) {
                if ((now - lastTick) >= DEBOUNCE_TIME) {
                    keyState = KEY_IDLE;
                }
            } else {
                lastTick = now;
            }
            break;
    }
}
