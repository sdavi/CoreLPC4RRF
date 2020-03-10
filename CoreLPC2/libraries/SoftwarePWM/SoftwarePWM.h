//Author: sdavi

#ifndef SOFTWAREPWM_H
#define SOFTWAREPWM_H

#include "Core.h"
#include "SoftwarePWMTimer.h"

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
    void Check();

    
    Pin GetPin() const {return pin;}
    uint16_t GetFrequency() const {return frequency;}

    void Interrupt();

#ifdef LPC_DEBUG
    void IncrementLateCount();
    uint32_t GetLateCount(){ return lateCount; };
#endif
    
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
    
    volatile uint32_t nextRun;
    volatile uint32_t lateCount;
    
    ticker_event_t event;
    
    void ScheduleEvent(uint32_t timeout);

    
};



#endif
