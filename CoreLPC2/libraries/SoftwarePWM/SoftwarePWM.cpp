//Author sdavi

#include "SoftwarePWM.h"
#include "SoftwarePWMTimer.h"

extern "C" void debugPrintf(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));

SoftwarePWM::SoftwarePWM(Pin softPWMPin) noexcept:
        pwmRunning(false),
        pin(softPWMPin),
        gpioPort((LPC_GPIO_T*)(LPC_GPIO0_BASE + ((softPWMPin & 0xE0)))),
        gpioPortPinBitPosition( 1 << (softPWMPin & 0x1f) ),
        period(0),
        onTime(0)

{
    pinMode(pin, OUTPUT_LOW);
}

void SoftwarePWM::Enable() noexcept
{
    pwmRunning = true;
    softwarePWMTimer.EnableChannel(this);
}
void SoftwarePWM::Disable() noexcept
{
    pwmRunning = false;
    pinMode(pin, OUTPUT_LOW);
    softwarePWMTimer.DisableChannel(this);
}

void SoftwarePWM::AnalogWrite(float ulValue, uint16_t freq, Pin pin) noexcept
{
    //Note: AnalogWrite gets called repeatedly by RRF for heaters
    
    //debugPrintf("[SoftwarePWM] Write -  %d.%d %f %" PRIu32 "\n", (pin >> 5), (pin & 0x1f), ulValue, freq);
    const uint32_t newPeriod = (1000000/freq);
    const uint32_t newOnTime = CalculateDutyCycle(ulValue, newPeriod);

    //Common Frequnecies used in RRF:
    //Freq:   10Hz,     250Hz  and 500Hz
    //Period: 100000us, 4000us and 2000us
    //Typically 10Hz are used for Heat beds 250Hz is used for Hotends/fans and some fans may need up to 500Hz
    

    //if enforcing a minimum on/off time to prevent the same channel firing in rapid succession, we get:
    // 100us = Min Duty:   0.1%,  2.5%  and 5%
    // 50us  = Min Duty:   0.05%, 1.25% and 2.5%
    // 10us  = Min Duey:   0.01%, 0.25% and 0.5%
    constexpr uint16_t MinimumTime = 100; //microseconds

    if(onTime < MinimumTime){ onTime = 0; }
    if(onTime > (period-MinimumTime)){ onTime = period; }
    
    if(newOnTime != onTime || newPeriod != period)
    {
        //Frequency or duty has changed, requires update
        
        Disable(); //Disable the channel and stop the interrupt
        
        period = newPeriod;
        onTime = newOnTime;
        
        //check for 100% on or 100% off, no need for interrupts
        if(onTime == 0)
        {
            gpioPort->CLR = gpioPortPinBitPosition; //Pin Low
        }
        else if(onTime == period)
        {
            gpioPort->SET = gpioPortPinBitPosition; //Pin High
        }
        else
        {
            //Enable and use interrupts to generate the PWM signal
            Enable();
        }
    }
    else
    {
        //PWM not changed
        
        if(pwmRunning == false)
        {
            // this pwm is not running, keep setting to zero as a precaution incase the same pin has accidently been used elsewhere
            gpioPort->CLR = gpioPortPinBitPosition; //Pin Low
        }
    }
}

uint32_t SoftwarePWM::CalculateDutyCycle(float newValue, uint32_t newPeriod)
{
    uint32_t ot = (uint32_t) (newPeriod * newValue);
    if(ot > newPeriod) ot = newPeriod;
    return ot;
}
