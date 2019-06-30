

#ifndef __ARDUINO_H__
#define __ARDUINO_H__

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "stdutils.h"
//#include "bit_constants.h"
#include "gpio.h"
#include "adc.h"
#include "systick.h"
#include "timer.h"
//#include "uart.h"
#include "delay.h"
#include "reset.h"


#include "interrupt_lpc.h"


#include "ecv.h"        // included to compile rrf
#undef yield            // eCv definition clashes with function 'yield' in wiring.c (can use _ecv_yield instead within annotations)


extern void loop();
extern void setup();
extern void init( void );



//SD: Added for compatibility with RRF
//typedef uint8_t Pin;
//static const Pin NoPin = 0xFF;
typedef gpioPins_et Pin;
static const Pin NoPin = P_NC; //which =0xff

#include "exti.h"
#include "board.h"
#include "wiring_digital.h"
//#include "wiring_analog.h"
#include "wirish_time.h"
#include "wdt.h"

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus
    void CoreSysTick(void);

    
#ifdef __cplusplus
} // extern "C"

    
// Usage:
// 1. Enable the channels you need by making calls to AnalogEnableChannel.
// 2. If desired, call AnalogSetCallback to set a callback for when conversion is complete.
// 3. Call AnalogStartConversion. This may be done e.g. in a tick interrupt if regular conversions are wanted.
// 4. Either use the callback to determine when conversion is complete, or call AnalogCheckReady to poll the status.
// 5. Call AnalogReadChannel to read the most recent converted result for each channel of interest.

enum AnalogChannelNumber
{
    NO_ADC=-1,
    ADC0=0,
    ADC1,
    ADC2,
    ADC3,
    ADC4,
    ADC5,
    ADC6,
    ADC7
};

// Definitions for PWM channels
typedef enum _EPWMChannel
{
    NOT_ON_PWM=-1,
    PWM1_1=0,
    PWM1_2,
    PWM1_3,
    PWM1_4,
    PWM1_5,
    PWM1_6,
} EPWMChannel ;

// Pin Attributes to be OR-ed
#define PIN_ATTR_NONE        (0UL)
#define PIN_ATTR_COMBO        (1UL<<0)
#define PIN_ATTR_ANALOG        (1UL<<1)
#define PIN_ATTR_DIGITAL    (1UL<<2)
#define PIN_ATTR_PWM        (1UL<<3)
#define PIN_ATTR_TIMER        (1UL<<4)
#define PIN_ATTR_DAC        (1UL<<5)


//cut down version
struct PinDescription{
    uint8_t pPort;
    //uint32_t ulPin;
    //uint32_t ulPeripheralId;
    //pio_type_t ulPinType;
    //uint32_t ulPinConfiguration;
    uint32_t ulPinAttribute;
    AnalogChannelNumber ulADCChannelNumber; // ADC or DAC channel number
    EPWMChannel ulPWMChannel;
    //ETCChannel ulTCChannel;
};

/* Pins table to be instantiated into variant.cpp */
extern const PinDescription g_APinDescription[];


enum SSPChannel {
    SSP0 = 0,
    SSP1
};




//#include "platform_memory.h"
#include "new.h"
#include "WCharacter.h"
#include "HardwareSerial.h"
#include "Stream.h"
//#include "eeprom.h"

#include "usb_serial.h"
#include "WMath.h"

#include "AnalogIn.h"
#include "AnalogOut.h"

#endif // __cplusplus


#include "variant.h"


#ifndef __STATIC_INLINE
 #define __STATIC_INLINE  static inline
#endif
#endif /* __ARDUINO_H__ */

/*lint -restore */
