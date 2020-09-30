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
void ClickEncoder<TEMPLATE_TYPE_NAMES>::init() {
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

TEMPLATE_TYPES
void ClickEncoder<TEMPLATE_TYPE_NAMES>::incAcceleration() {
#ifndef ROTARY_ACCEL_OPTIMIZATION
    if (accelerationEnabled) {
        // increment accelerator if encoder has been moved
        uint16_t increasedAcc = acceleration + ENC_ACCEL_INC;
        // Apply acceleration changes
        // We ignore overflows here because it is very unlikely to achieve a accerleration which exceeds 65535
        acceleration = increasedAcc < ENC_ACCEL_TOP ? increasedAcc : ENC_ACCEL_TOP;
    }
#else
    // We can always change this field as it is faster than checking first if acceleration is anabled
    ++accelInc;
#endif
}

TEMPLATE_TYPES
void ClickEncoder<TEMPLATE_TYPE_NAMES>::decAcceleration() {
#ifndef ROTARY_ACCEL_OPTIMIZATION
    if (accelerationEnabled) {
        uint16_t nextAcceleration = acceleration;
        // decelerate every tick
        uint16_t decreasedAcc = nextAcceleration - ENC_ACCEL_DEC;
        // handle underflow
        // Apply acceleration changes
        acceleration = decreasedAcc > nextAcceleration ? 0 : decreasedAcc;
    }
#else
    // We can always change this field as it is faster than checking first if acceleration is anabled
    ++accelDec;
#endif
}

#if defined(ROTARY_ISR_SERVICE) && defined(SPLIT_ROTARY_ISR_SERVICE)
#define _LAST_PIN_A_STATE_MASK _BV(7)
#define _LAST_PIN_B_STATE_MASK _BV(6)

TEMPLATE_TYPES
bool ClickEncoder<TEMPLATE_TYPE_NAMES>::servicePinA() {

    int8_t _last = last;

    bool lastPinAState = (_last & _LAST_PIN_A_STATE_MASK) == _LAST_PIN_A_STATE_MASK;
    bool currentPinAState = FastPin<pinA>::digitalRead() == pinsActive;

    // Validate if there really was a toggle and this was not called due to jitter
    if (currentPinAState != lastPinAState) {
        int8_t curr = _last ^ 3 ^ _LAST_PIN_A_STATE_MASK;

        last = curr;

        // Remove last pin state information
        curr &= 0x03;
        _last &= 0x03;

        int8_t diff = _last - curr;

        delta += (diff & 0b010) - 1; // bit at index 1 = direction (1/0) means (+/-)

        incAcceleration();

        return true;
    }
    return false;
}

TEMPLATE_TYPES
bool ClickEncoder<TEMPLATE_TYPE_NAMES>::servicePinB() {

    int8_t _last = last;

    bool lastPinBState = (_last & _LAST_PIN_B_STATE_MASK) == _LAST_PIN_B_STATE_MASK;
    bool currentPinBState = FastPin<pinB>::digitalRead() == pinsActive;

    // Validate if there really was a toggle and this was not called due to jitter
    if (currentPinBState != lastPinBState) {
        int8_t curr = _last ^ 1 ^ _LAST_PIN_B_STATE_MASK;

        last = curr;

        // Remove last pin state information
        curr &= 0x03;
        _last &= 0x03;

        delta += curr - _last;

        incAcceleration();

        return true;
    }
    return false;
}
#endif

#if !defined(ROTARY_ISR_SERVICE) || !defined(SPLIT_ROTARY_ISR_SERVICE)
// We expect that interrupts will be disabled during executing this function inside a ISR
TEMPLATE_TYPES
bool ClickEncoder<TEMPLATE_TYPE_NAMES>::rotaryService() {
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

    bool detectedStep = (diff & 0b01) == 0b01;

    if (detectedStep) { // bit at index 0 = step
        last = curr;
        delta += (diff & 0b010) - 1; // bit at index 1 = direction (1/0) means (+/-)
#endif
        incAcceleration();
    }

    return detectedStep;
}
#endif

// This function still needs to be polled... else we have no decreasing acceleration!
TEMPLATE_TYPES
void ClickEncoder<TEMPLATE_TYPE_NAMES>::service() {

    decAcceleration();

#ifndef ROTARY_ISR_SERVICE
    rotaryService();
#endif

    // handle buttonState
#ifndef WITHOUT_BUTTON

// NOTE: We expect that this routine does not get called with 1 kHz anymore, as this would be wasteful AF!
// Therefore it is expected that this service routine already gets called in the wanted ENC_BUTTONINTERVAL => simpler code
#ifndef ROTARY_ISR_SERVICE
    unsigned long currentMillis = millis();
    /*
    if (currentMillis < lastButtonCheck) {
        lastButtonCheck = 0;
    }
    */
#endif
    // check buttonState only, if a pin has been provided
    if (pinBTN >= 0) {

#ifndef ROTARY_ISR_SERVICE
        // Handle case when millis() wraps back around to zero
        // checking buttonState is sufficient every 10-30ms
        if ((unsigned long)(currentMillis - lastButtonCheck) >= ENC_BUTTONINTERVAL) {
            lastButtonCheck = currentMillis;
#endif

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
#ifndef ROTARY_ISR_SERVICE
        }
#endif
    }
#endif // WITHOUT_BUTTON
}

// ----------------------------------------------------------------------------
TEMPLATE_TYPES
int16_t ClickEncoder<TEMPLATE_TYPE_NAMES>::getValue() {

    int16_t accel = 1;

    if (accelerationEnabled) {

#ifndef ROTARY_ACCEL_OPTIMIZATION
        uint16_t currentAccel;
        // Interrupt safe multibyte read
        do {
            currentAccel = acceleration;
        } while (currentAccel != acceleration);

        accel += (currentAccel >> 8);
#else
        int16_t accelChange = accelInc * ENC_ACCEL_INC - accelDec * ENC_ACCEL_DEC;
        acceleration += accelChange;

        if (acceleration > ENC_ACCEL_TOP) {
            acceleration = accelChange < 0 ? 0 : ENC_ACCEL_TOP;
        }

        // Reset acceleration change counters
        accelDec = 0;
        accelInc = 0;

        accel += (acceleration >> 8);
#endif
    }

    // Reads of one byte values is fine when interrupts are enabled (needs only one cycle)
    int8_t val = delta;

    int8_t deltaChange = stepsPerNotch;
    // We need some special treatment for negative values see: https://github.com/soligen2010/encoder/issues/14
    if (val < 0) {
        val = -val;
        accel = -accel;
        deltaChange = -deltaChange;
    }

    // Check if we have enough steps for a notch
    if (val >= stepsPerNotch) {

        // Apply delta changes
        // Because we do not use modulo here this might cause "ghost moves" if not processed fast enough
        // or you simply set the wrong value for stepsPerNotch
        delta -= deltaChange;

        return accel;
    }

    return 0;
}

// ----------------------------------------------------------------------------

#ifndef WITHOUT_BUTTON
TEMPLATE_TYPES
ButtonState ClickEncoder<TEMPLATE_TYPE_NAMES>::getButtonState() {
    ButtonState ret = buttonState;
    if (ret != Held) {
        // Reads & writes of one byte values is fine when interrupts are enabled (needs only one cycle)
        buttonState = Open; // reset
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

#if defined(ROTARY_ISR_SERVICE) && defined(SPLIT_ROTARY_ISR_SERVICE)
#undef _LAST_PIN_A_STATE_MASK
#undef _LAST_PIN_B_STATE_MASK
#endif

#endif
