/**
 * A lossless event based rotary encoder button wrapper for
 * Paul Stoffregen's Encoder library
 * https://www.pjrc.com/teensy/td_libs_Encoder.html
 * and Thomas Fredericks' Bounce2 library
 * https://github.com/thomasfredericks/Bounce2
 * and inspired by Lennart Hennigs Button2
 * https://github.com/LennartHennigs/Button2
 *
 * Written for Teensy but tested on Arduino Uno and others.
 *
 * It is important, but not essential, to use hardware interrupt pins (any
 * pin on a Teensy, but only pins 2 & 3 on an UNO). The library will work on software
 * interrupts but may not avoid loss of steps.
 * Do not use setRateLimit() with software interrupts.
 * See https://www.pjrc.com/teensy/td_libs_Encoder.html
 * for more details.
 *
 * Works with Encoder+button, Encoder alone or just a Button.
 *
 * Encoder can fire rotation callbacks when pressed or not and optionally
 * in quadrature mode. Default is once per click (ie once per four encoder positions).
 *
 * Button events are fired for both the encoder button and/or a standalone
 * momentary switch.
 *
 * GPLv2 Licence https://www.gnu.org/licenses/old-licenses/gpl-2.0.txt
 *
 * Copyright (c) 2022 Philip Fletcher <philip.fletcher@stutchbury.com>
 *
 */

#pragma once

#ifdef STM32G4xx

#ifndef __STM32HWENCODER_H__
#define __STM32HWENCODER_H__

#include "Arduino.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_tim.h"
#include "stm32g4xx_hal_exti.h"

class EncoderButton
{
public:
    typedef void (*CallbackFunction)(EncoderButton &);
    bool initialized = false;
    uint32_t cpr;                     // Number of counts per rotation
    PinName _pinA, _pinB, _pinSwitch; //

    EncoderButton(uint32_t ppr, int8_t pinA, int8_t pinB, int8_t switchPin);
    // EncoderButton(byte encoderPin1, byte encoderPin2);
    // EncoderButton(byte switchPin);

    /**
     * Set up the timer peripheral under the hood.
     */
    void init(void);

    void update(void);

    /**
     * Fires when button state changes
     */
    void setChangedHandler(CallbackFunction f);

    /**
     * Fires after button is pressed down
     */
    void setPressedHandler(CallbackFunction f);

    /**
     * Fires after button is released.
     * Note: if the encoder is turned while pressed, then
     * the 'encoder released' callback is fired instead.
     */
    void setReleasedHandler(CallbackFunction f);

    /**
     * Fires after button is clicked (when pressed duration
     * is less than setLongClickDuration() (default 750ms)
     * Note: the number of multi clicks can be read from
     * EncoderButton.clickCount() in the callback so you are not
     * limited to double or triple clicks - any number can be actioned.
     * If double or triple click callbacks are set, this will not be
     * fired for those events.
     */
    void setClickHandler(CallbackFunction f);

    /**
     * Fires after button is double clicked
     * Determined by setMultiClickInterval() (default 250ms)
     * Syntactic sugar for click handler + clickCount == 2
     */
    void setDoubleClickHandler(CallbackFunction f);

    /**
     * Fires when button is triple clicked
     * Syntactic sugar for click handler + clickCount == 3
     */
    void setTripleClickHandler(CallbackFunction f);

    /** ***************************************************
     * set encoder callback handlers
     */

    /**
     * Fired when the encoder is turned. By default is fired once
     * per 'click' but can fire full quadrature encoding if
     * useQuadPrecision(bool) is set to 'true'
     */
    void setEncoderHandler(CallbackFunction f);

    /**
     * Reset the counted position of the encoder.
     */
    void resetPosition(long pos = 0);

    /**
     * Reset the counted pressed position of the encoder.
     */
    void resetPressedPosition(long pos = 0);

    /**
     * The number of times the long press handler has  been fired in the
     * button pressed event
     */
    uint8_t longPressCount();

    /**
     * Returns true if pressed
     */
    bool isPressed();

    /** ***************************************
     *  encoder state
     */

    /**
     * Returns a positive (CW) or negative (CCW) integer. Is normally 1 or -1 but if your
     * loop() has lots of processing, your Arduino is slow or you setRateLimit()
     * this will report the actual number of increments made by the encoder since
     * the last encoder handler event.
     */
    int16_t increment();

    /**
     * The current position of the encoder. Can be reset by resetPosition()
     */
    long position();

    /**
     * Returns true if enabled
     */
    bool enabled();

    /**
     * Set enabled to true of false
     * This will enable/disable all event callbacks.
     * When disabled the encoder positions will not be updated.
     */
    void enable(bool e = true);

protected:
    TIM_HandleTypeDef encoder_handle;

    CallbackFunction changed_cb = NULL;
    CallbackFunction pressed_cb = NULL;
    CallbackFunction released_cb = NULL;
    CallbackFunction click_cb = NULL;
    CallbackFunction long_click_cb = NULL;
    CallbackFunction double_click_cb = NULL;
    CallbackFunction triple_click_cb = NULL;
    CallbackFunction long_press_cb = NULL;
    // Encoder
    CallbackFunction encoder_cb = NULL;
    CallbackFunction encoder_pressed_cb = NULL;
    CallbackFunction encoder_released_cb = NULL;
    // Common
    CallbackFunction idle_cb = NULL;

private:
    bool haveButton = false;
    bool haveEncoder = false;
    int16_t last_position;
    int16_t encoderPosition = 0;
    int16_t currentPosition = 0;
    int16_t currentPressedPosition = 0;
    int encoderIncrement = 0;
    bool encodingPressed = false;
    unsigned char _buttonState = HIGH;
    unsigned int multiClickInterval = 250;
    unsigned int longClickDuration = 750;
    bool clickFired = true;
    unsigned char clickCounter = 0;
    unsigned int longPressCounter = 0;
    unsigned long lastEventMs = millis();
    unsigned long idleTimeout = 10000;
    bool idleFlagged = false;
    bool previousState = LOW;
    unsigned char prevClickCount = 0;
    bool repeatLongPress = false;

    bool _enabled = true;
};

#endif
#endif