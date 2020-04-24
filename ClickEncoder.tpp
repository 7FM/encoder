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
// Button configuration (values for 1ms timer service calls)
//
#define ENC_BUTTONINTERVAL 10 // check button every x milliseconds, also debouce time

// ----------------------------------------------------------------------------
// Acceleration configuration (for 1000Hz calls to ::service())
//
#define ENC_ACCEL_TOP 3072 // max. acceleration: *12 (val >> 8)
#define ENC_ACCEL_INC 25
#define ENC_ACCEL_DEC 2

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
ClickEncoder<TEMPLATE_TYPE_NAMES>::ClickEncoder(uint8_t stepsPerNotch)
    : steps(stepsPerNotch), accelerationEnabled(true)
#ifndef WITHOUT_BUTTON
      ,
      doubleClickEnabled(true), buttonHeldEnabled(true),
      button(Open)
#endif
{
    if (pinsActive) {
        FastPin<pinA>::setInput();
        FastPin<pinA>::setInput();
        FastPin<pinB>::lo();
        FastPin<pinB>::lo();

#ifndef WITHOUT_BUTTON
        if (pinBTN >= 0) {
            FastPin<pinBTN>::setInput();
            FastPin<pinBTN>::lo();
        }
#endif
    } else {
        FastPin<pinA>::setInput();
        FastPin<pinA>::setInput();
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
    bool moved = false;

    if (pinA >= 0 && pinB >= 0) {
        if (accelerationEnabled) { // decelerate every tick
            acceleration -= ENC_ACCEL_DEC;
            if (acceleration & 0x8000) { // handle overflow of MSB is set
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
            moved = true;
        }
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
            moved = true;
        }
#else
#error "Error: define ENC_DECODER to ENC_NORMAL or ENC_FLAKY"
#endif

        if (accelerationEnabled && moved) {
            // increment accelerator if encoder has been moved
            if (acceleration <= (ENC_ACCEL_TOP - ENC_ACCEL_INC)) {
                acceleration += ENC_ACCEL_INC;
            }
        }
    }
    // handle button
    //
#ifndef WITHOUT_BUTTON
    unsigned long currentMillis = millis();
    if (currentMillis < lastButtonCheck) {
        lastButtonCheck = 0;
    }
    // Handle case when millis() wraps back around to zero
    // check button only, if a pin has been provided
    // checking button is sufficient every 10-30ms
    if ((pinBTN >= 0 || (pinBTN == 0 && buttonOnPinZeroEnabled)) && ((currentMillis - lastButtonCheck) >= ENC_BUTTONINTERVAL)) {
        lastButtonCheck = currentMillis;

        bool pinRead = getPinState();

        if (pinRead == pinsActive) { // key is down
            ++keyDownTicks;
            if ((keyDownTicks > (buttonHoldTime / ENC_BUTTONINTERVAL)) && (buttonHeldEnabled)) {
                button = Held;
            }
        }

        if (pinRead == !pinsActive) { // key is now up
            if (keyDownTicks > 1) {   //Make sure key was down through 1 complete tick to prevent random transients from registering as click
                if (button == Held) {
                    button = Released;
                    doubleClickTicks = 0;
                } else {
#define ENC_SINGLECLICKONLY 1
                    if (doubleClickTicks > ENC_SINGLECLICKONLY) { // prevent trigger in single click mode
                        if (doubleClickTicks < (buttonDoubleClickTime / ENC_BUTTONINTERVAL)) {
                            button = DoubleClicked;
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
                button = Clicked;
            }
        }
    }
#endif // WITHOUT_BUTTON
}

// ----------------------------------------------------------------------------
TEMPLATE_TYPES
int16_t ClickEncoder<TEMPLATE_TYPE_NAMES>::getValue() {
    int16_t val;

    noInterrupts();
    val = delta;

    if (steps == 2) {
        delta = val & 1;
        val >>= 1;
    } else if (steps == 4) {
        delta = val & 3;
        val >>= 2;
    } else {
        delta = 0; // default to 1 step per notch
    }

    int16_t r = 0;
    int16_t accel = ((accelerationEnabled) ? (acceleration >> 8) : 0);

    if (val < 0) {
        r -= 1 + accel;
    } else if (val > 0) {
        r += 1 + accel;
    }
    interrupts();

    return r;
}

// ----------------------------------------------------------------------------

#ifndef WITHOUT_BUTTON
TEMPLATE_TYPES
typename ClickEncoder<TEMPLATE_TYPE_NAMES>::Button ClickEncoder<TEMPLATE_TYPE_NAMES>::getButton() {
    noInterrupts();
    ClickEncoder<TEMPLATE_TYPE_NAMES>::Button ret = button;
    if (button != ClickEncoder<TEMPLATE_TYPE_NAMES>::Held && ret != ClickEncoder<TEMPLATE_TYPE_NAMES>::Open) {
        button = ClickEncoder<TEMPLATE_TYPE_NAMES>::Open; // reset
    }
    interrupts();

    return ret;
}

TEMPLATE_TYPES
bool ClickEncoder<TEMPLATE_TYPE_NAMES>::getPinState() const {
    bool pinState;
    if (analogInput) {
        int16_t pinValue = analogRead(pinBTN);
        pinState = ((pinValue >= anlogActiveRangeLow) && (pinValue <= anlogActiveRangeHigh)) ? false : true; // set result to LOW (button pressed) if analog input is in range
    } else {
        pinState = FastPin<pinBTN>::digitalRead();
    }
    return pinState;
}

#endif
