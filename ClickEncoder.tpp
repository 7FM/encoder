// ----------------------------------------------------------------------------
// Rotary Encoder Driver with Acceleration
// Supports Click, DoubleClick, Long Click
//
// (c) 2010 karl@pitrich.com
// (c) 2014 karl@pitrich.com
//
// Timer-based rotary encoder logic by Peter Dannegger
// http://www.mikrocontroller.net/articles/Drehgeber
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------

#if ENC_DECODER != ENC_NORMAL
#if ENC_HALFSTEP
// decoding table for hardware with flaky notch (half resolution)
TEMPLATE_TYPES
const int8_t ClickEncoder<TEMPLATE_TYPE_NAMES>::table[16] __attribute__((__progmem__)) = {
    0, 0, -1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, -1, 0, 0};
#else
// decoding table for normal hardware
TEMPLATE_TYPES
const int8_t ClickEncoder<TEMPLATE_TYPE_NAMES>::table[16] __attribute__((__progmem__)) = {
    0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};
#endif
#endif

#include <FastPin.h>

// ----------------------------------------------------------------------------
TEMPLATE_TYPES
void ClickEncoder<TEMPLATE_TYPE_NAMES>::init(uint8_t stepsPerNotch) {

    steps = stepsPerNotch;
    FastPin<pinA>::setInput();
    FastPin<pinB>::setInput();
    if (pinsActive) {
        FastPin<pinA>::lo();
        FastPin<pinB>::lo();

#ifndef WITHOUT_BUTTON
        if (pinBTN >= 0) {
            FastPin<pinBTN>::setInput();
            FastPin<pinBTN>::lo();
        }
#endif
    } else {
        FastPin<pinA>::hi();
        FastPin<pinB>::hi();

#ifndef WITHOUT_BUTTON
        if (pinBTN >= 0) {
            FastPin<pinBTN>::setInput();
            FastPin<pinBTN>::hi();
        }
#endif
    }

    if (FastPin<pinA>::digitalRead() == pinsActive) {
        last = 3;
    }

    if (FastPin<pinB>::digitalRead() == pinsActive) {
        last ^= 1;
    }
}

// ----------------------------------------------------------------------------
// call this every 1 millisecond via timer ISR
//
TEMPLATE_TYPES
void ClickEncoder<TEMPLATE_TYPE_NAMES>::service() {
    if (accelerationEnabled) { // decelerate every tick
        uint16_t prevAcc = acceleration;
        acceleration -= ENC_ACCEL_DEC;
        if (acceleration > prevAcc) { // handle overflow of MSB is set
            acceleration = 0;
        }
    }

#if ENC_DECODER == ENC_FLAKY
    last = (last << 2) & 0x0F;

    if (FastPin<pinA>::digitalRead() == pinsActive) {
        last |= 2;
    }

    if (FastPin<pinB>::digitalRead() == pinsActive) {
        last |= 1;
    }

    int8_t tbl = pgm_read_byte(&table[last]);
    if (tbl) {
        delta += tbl;
#elif ENC_DECODER == ENC_NORMAL
    int8_t curr = 0;

    if (FastPin<pinA>::digitalRead() == pinsActive) {
        curr = 3;
    }

    if (FastPin<pinB>::digitalRead() == pinsActive) {
        curr ^= 1;
    }

    int8_t diff = last - curr;

    if (diff & 1) { // bit 0 = step
        last = curr;
        delta += (diff & 2) - 1; // bit 1 = direction (+/-)
#else
#error "Error: define ENC_DECODER to ENC_NORMAL or ENC_FLAKY"
#endif
        if (accelerationEnabled) {
            // increment accelerator if encoder has been moved
            uint16_t increasedAcc = acceleration + ENC_ACCEL_INC;
            if (increasedAcc <= ENC_ACCEL_TOP) {
                acceleration = increasedAcc;
            }
        }
    }

    // handle buttonState
#ifndef WITHOUT_BUTTON
    unsigned long currentMillis = millis();
    if (currentMillis < lastButtonCheck) {
        lastButtonCheck = 0;
    }
    // Handle case when millis() wraps back around to zero
    // check buttonState only, if a pin has been provided
    // checking buttonState is sufficient every 10-30ms
    if (pinBTN >= 0 && ((currentMillis - lastButtonCheck) >= ENC_BUTTONINTERVAL)) {
        lastButtonCheck = currentMillis;

        if (getPinState() == pinsActive) { // key is down
            ++keyDownTicks;
            if (buttonHeldEnabled && keyDownTicks > (buttonHoldTime / ENC_BUTTONINTERVAL)) {
                buttonState = Held;
            }
        } else {                    // key is now up
            if (keyDownTicks > 1) { //Make sure key was down through 1 complete tick to prevent random transients from registering as click
                if (buttonState == Held) {
                    buttonState = Released;
                    doubleClickTicks = 0;
                } else {
#define ENC_SINGLECLICKONLY 1
                    if (doubleClickTicks > ENC_SINGLECLICKONLY) { // prevent trigger in single click mode
                        if (doubleClickTicks < (buttonDoubleClickTime / ENC_BUTTONINTERVAL)) {
                            buttonState = DoubleClicked;
                            doubleClickTicks = 0;
                        }
                    } else {
                        doubleClickTicks = (doubleClickEnabled) ? (buttonDoubleClickTime / ENC_BUTTONINTERVAL) : ENC_SINGLECLICKONLY;
                    }
                }
            }

            keyDownTicks = 0;
        }

        if (doubleClickTicks > 0) {
            --doubleClickTicks;
            if (doubleClickTicks == 0) {
                buttonState = Clicked;
            }
        }
    }
#endif // WITHOUT_BUTTON
}

// ----------------------------------------------------------------------------
TEMPLATE_TYPES
int16_t ClickEncoder<TEMPLATE_TYPE_NAMES>::getValue() {
    noInterrupts();
    int16_t val = delta;

    if (steps == 2) {
        delta = val & 1;
        val >>= 1;
    } else if (steps == 4) {
        delta = val & 3;
        val >>= 2;
    } else {
        delta = 0; // default to 1 step per notch
    }

    int16_t accel = ((accelerationEnabled) ? (acceleration >> 8) + 1 : 1);

    interrupts();

    int16_t r = 0;

    if (val < 0) {
        r = -accel;
    } else if (val > 0) {
        r = accel;
    }

    return r;
}

// ----------------------------------------------------------------------------

#ifndef WITHOUT_BUTTON
TEMPLATE_TYPES
ButtonState ClickEncoder<TEMPLATE_TYPE_NAMES>::getButtonState() {
    ButtonState ret = buttonState;
    if (ret != Held) {
        noInterrupts();
        buttonState = Open; // reset
        interrupts();
    }

    return ret;
}

TEMPLATE_TYPES
bool ClickEncoder<TEMPLATE_TYPE_NAMES>::getPinState() {
    bool pinState;
    if (analogInput) {
        int16_t pinValue = FastPin<pinBTN>::analogRead();
        pinState = ((pinValue >= anlogActiveRangeLow) && (pinValue <= anlogActiveRangeHigh)) ? pinsActive : !pinsActive; // set result to LOW (buttonState pressed) if analog input is in range
    } else {
        pinState = FastPin<pinBTN>::digitalRead();
    }
    return pinState;
}

#endif
