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

#ifndef __have__ClickEncoder_h__
#define __have__ClickEncoder_h__

// ----------------------------------------------------------------------------
// Button configuration (values for 1ms timer service calls)
//
#define DEFAULT_ENC_BUTTONINTERVAL 10 // check buttonState every x milliseconds, also debouce time

// ---Button defaults-------------------------------------------------------------

#define BTN_DOUBLECLICKTIME 400 // second click within 400ms
#define BTN_HOLDTIME 1000       // report held button after 1s

// ----------------------------------------------------------------------------

#include <stdint.h>
#if defined(__AVR__)
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#endif
#include <Arduino.h>

// ----------------------------------------------------------------------------

#define ENC_NORMAL (1 << 1) // use Peter Danneger's decoder
#define ENC_FLAKY (1 << 2)  // use Table-based decoder

// ----------------------------------------------------------------------------

#ifndef ENC_DECODER
#define ENC_DECODER ENC_NORMAL
#endif

#if ENC_DECODER == ENC_FLAKY
#ifndef ENC_HALFSTEP
#define ENC_HALFSTEP 1 // use table for half step per default
#endif
#endif

// ----------------------------------------------------------------------------
#if defined(__arm__) && (defined(__STM32F1__) || defined(__STM32F4__))
typedef WiringPinMode pinMode_t;
#else
typedef uint8_t pinMode_t;
#endif

// ----------------------------------------------------------------------------
// Acceleration configuration (for 1000Hz calls to ::service())
//
#define DEFAULT_ENC_ACCEL_TOP 3072 // max. acceleration: 12 encoded as maxAccel * pow(2, 8)
#define DEFAULT_ENC_ACCEL_INC 25
#define DEFAULT_ENC_ACCEL_DEC 2
#define DEFAULT_STEPS_PER_NOTCH 4

#ifndef WITHOUT_BUTTON
#define TEMPLATE_PARAMETERS                                           \
    uint8_t pinA,                                                     \
        uint8_t pinB,                                                 \
        int8_t pinBTN,                                                \
        bool pinsActive = false,                                      \
             uint8_t stepsPerNotch = DEFAULT_STEPS_PER_NOTCH,                 \
             uint16_t ENC_ACCEL_TOP = DEFAULT_ENC_ACCEL_TOP,          \
             uint16_t ENC_ACCEL_INC = DEFAULT_ENC_ACCEL_INC,          \
             uint16_t ENC_ACCEL_DEC = DEFAULT_ENC_ACCEL_DEC,          \
             uint8_t ENC_BUTTONINTERVAL = DEFAULT_ENC_BUTTONINTERVAL, \
             uint16_t buttonHoldTime = BTN_HOLDTIME,                  \
             uint16_t buttonDoubleClickTime = BTN_DOUBLECLICKTIME,    \
             bool analogInput = false,                                \
             int16_t anlogActiveRangeLow = 0,                         \
             int16_t anlogActiveRangeHigh = 0

#define TEMPLATE_TYPES template <uint8_t pinA,                   \
                                 uint8_t pinB,                   \
                                 int8_t pinBTN,                  \
                                 bool pinsActive,                \
                                 uint8_t stepsPerNotch,                  \
                                 uint16_t ENC_ACCEL_TOP,         \
                                 uint16_t ENC_ACCEL_INC,         \
                                 uint16_t ENC_ACCEL_DEC,         \
                                 uint8_t ENC_BUTTONINTERVAL,     \
                                 uint16_t buttonHoldTime,        \
                                 uint16_t buttonDoubleClickTime, \
                                 bool analogInput,               \
                                 int16_t anlogActiveRangeLow,    \
                                 int16_t anlogActiveRangeHigh>

#define TEMPLATE_TYPE_NAMES pinA,                  \
                            pinB,                  \
                            pinBTN,                \
                            pinsActive,            \
                            stepsPerNotch,                 \
                            ENC_ACCEL_TOP,         \
                            ENC_ACCEL_INC,         \
                            ENC_ACCEL_DEC,         \
                            ENC_BUTTONINTERVAL,    \
                            buttonHoldTime,        \
                            buttonDoubleClickTime, \
                            analogInput,           \
                            anlogActiveRangeLow,   \
                            anlogActiveRangeHigh
#else
#define TEMPLATE_PARAMETERS uint8_t pinA, uint8_t pinB,                          \
                            bool pinsActive = false,                             \
                                 uint8_t stepsPerNotch = DEFAULT_STEPS_PER_NOTCH,        \
                                 uint16_t ENC_ACCEL_TOP = DEFAULT_ENC_ACCEL_TOP, \
                                 uint16_t ENC_ACCEL_INC = DEFAULT_ENC_ACCEL_INC, \
                                 uint16_t ENC_ACCEL_DEC = DEFAULT_ENC_ACCEL_DEC

#define TEMPLATE_TYPES template <uint8_t pinA, uint8_t pinB, \
                                 bool pinsActive,            \
                                 uint8_t stepsPerNotch,              \
                                 uint16_t ENC_ACCEL_TOP,     \
                                 uint16_t ENC_ACCEL_INC,     \
                                 uint16_t ENC_ACCEL_DEC>

#define TEMPLATE_TYPE_NAMES pinA, pinB,    \
                            pinsActive,    \
                            stepsPerNotch,         \
                            ENC_ACCEL_TOP, \
                            ENC_ACCEL_INC, \
                            ENC_ACCEL_DEC
#endif
#define TEMPLATE_DEFINITION template <TEMPLATE_PARAMETERS>

typedef enum {
    Open = 0,

    Held,
    Released,

    Clicked,
    DoubleClicked
} ButtonState;

TEMPLATE_DEFINITION
class ClickEncoder {
  public:
    static void init();

    static inline void service() __attribute__((always_inline));
    static int16_t getValue();

#ifndef WITHOUT_BUTTON
    static ButtonState getButtonState();
#endif

#ifndef WITHOUT_BUTTON

    static inline void setDoubleClickEnabled(bool d) {
        doubleClickEnabled = d;
    }

    static inline bool getDoubleClickEnabled() __attribute__((always_inline)) {
        return doubleClickEnabled;
    }

    static inline void setButtonHeldEnabled(bool d) {
        buttonHeldEnabled = d;
    }

    static inline bool getButtonHeldEnabled() __attribute__((always_inline)) {
        return buttonHeldEnabled;
    }
#endif

    static inline void setAccelerationEnabled(bool a) {
        accelerationEnabled = a;
        if (!accelerationEnabled) {
            acceleration = 0;
        }
    }

    static inline bool getAccelerationEnabled() __attribute__((always_inline)) {
        return accelerationEnabled;
    }

#ifndef WITHOUT_BUTTON
  protected:
    inline static bool getPinState() __attribute__((always_inline));
#endif

  protected:
    static bool accelerationEnabled;
    static volatile int8_t delta;
    static volatile int8_t last;
    static volatile uint16_t acceleration;

#ifndef WITHOUT_BUTTON
    static bool doubleClickEnabled;
    static bool buttonHeldEnabled;
    static uint16_t keyDownTicks;
    static uint16_t doubleClickTicks;
    static unsigned long lastButtonCheck;
    static volatile ButtonState buttonState;
#endif
#if ENC_DECODER != ENC_NORMAL
    static const int8_t table[16];
#endif
};

#ifndef WITHOUT_BUTTON
TEMPLATE_TYPES
volatile ButtonState ClickEncoder<TEMPLATE_TYPE_NAMES>::buttonState = Open;
TEMPLATE_TYPES
bool ClickEncoder<TEMPLATE_TYPE_NAMES>::doubleClickEnabled = true;
TEMPLATE_TYPES
bool ClickEncoder<TEMPLATE_TYPE_NAMES>::buttonHeldEnabled = true;
TEMPLATE_TYPES
uint16_t ClickEncoder<TEMPLATE_TYPE_NAMES>::keyDownTicks = 0;
TEMPLATE_TYPES
uint16_t ClickEncoder<TEMPLATE_TYPE_NAMES>::doubleClickTicks = 0;
TEMPLATE_TYPES
unsigned long ClickEncoder<TEMPLATE_TYPE_NAMES>::lastButtonCheck = 0;
#endif

TEMPLATE_TYPES
bool ClickEncoder<TEMPLATE_TYPE_NAMES>::accelerationEnabled = true;
TEMPLATE_TYPES
volatile int8_t ClickEncoder<TEMPLATE_TYPE_NAMES>::delta = 0;
TEMPLATE_TYPES
volatile int8_t ClickEncoder<TEMPLATE_TYPE_NAMES>::last = 0;
TEMPLATE_TYPES
volatile uint16_t ClickEncoder<TEMPLATE_TYPE_NAMES>::acceleration = 0;
// ----------------------------------------------------------------------------

#include "ClickEncoder.tpp"

#endif // __have__ClickEncoder_h__
