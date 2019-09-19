//Author: sdavi

#ifndef SOFTWAREPWM_H
#define SOFTWAREPWM_H

#include "Core.h"
#include "us_ticker_api.h"

class SoftwarePWM
{
public:
    SoftwarePWM(Pin softPWMPin);
    
    void Enable();
    void Disable();

    void SetFrequency(uint16_t freq);
    void SetDutyCycle(float duty);

    void PWMOn();
    void PWMOff();
    
    bool IsRunning() {return pwmRunning;}
    
    Pin GetPin() const {return pin;}
    uint16_t GetFrequency() const {return frequency;}

    static void Interrupt(uint32_t ptr);

    
private:

    enum pwmState_t: uint8_t{
        PWM_OFF = 0,
        PWM_ON
    };

    bool pwmRunning;
    pwmState_t state;
    Pin pin;

    uint16_t frequency;
    volatile uint32_t period;
    volatile uint32_t onTime;
    
    ticker_event_t event;
    
    void ScheduleEvent(uint32_t timeout);
    void Handler();

};



#endif
