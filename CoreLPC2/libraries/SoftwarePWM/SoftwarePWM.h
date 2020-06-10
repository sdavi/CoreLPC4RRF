//Author: sdavi

#ifndef SOFTWAREPWM_H
#define SOFTWAREPWM_H

#include "Core.h"
#include "SoftwarePWMTimer.h"

class SoftwarePWM
{
public:
    SoftwarePWM(Pin softPWMPin) noexcept;
    
    void Enable() noexcept;
    void Disable() noexcept;
    void AnalogWrite(float ulValue, uint16_t freq, Pin pin) noexcept;

    Pin GetPin() const noexcept { return pin; }
    uint16_t GetFrequency() const noexcept { return (period!=0)?(1000000/period):0; }

    bool IsRunning() noexcept { return pwmRunning; }
    
    uint32_t lastState() noexcept;
    bool updateState(uint32_t *nextEvent) noexcept;

private:

    volatile bool pwmRunning;
    const Pin pin;
    LPC_GPIO_T * const gpioPort;
    const uint32_t gpioPortPinBitPosition;
    
    volatile uint32_t period;
    volatile uint32_t onTime;

    uint32_t CalculateDutyCycle(float newValue, uint32_t newPeriod);
    
};


//functions called by timer interrupt
inline uint32_t SoftwarePWM::lastState() noexcept
{
    return (gpioPort->PIN & gpioPortPinBitPosition);
}

inline bool SoftwarePWM::updateState(uint32_t *nextEvent) noexcept
{
    if(pwmRunning)
    {
        //read the current state from the pin
        if( gpioPort->PIN & gpioPortPinBitPosition )
        {
            //last pwm state was on
            *nextEvent = (period - onTime); //next change in offTime
            gpioPort->CLR = gpioPortPinBitPosition; //Pin Low
        }
        else
        {
            //last pwm state was off
            *nextEvent = onTime; //next change in onTime
            gpioPort->SET = gpioPortPinBitPosition; //Pin High
        }
        return true; //schedule next interrupt
    }
    else
    {
        gpioPort->CLR = gpioPortPinBitPosition; //Pin Low
    }
    return false; //disable interrupts
}


#endif
