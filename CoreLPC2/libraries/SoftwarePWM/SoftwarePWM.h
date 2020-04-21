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

    void SetFrequency(uint16_t freq) noexcept;
    void SetDutyCycle(float duty) noexcept;

    void PWMOn() noexcept;
    void PWMOff() noexcept;
    
    bool IsRunning() noexcept {return pwmRunning;}
    void Check() noexcept;

    
    Pin GetPin() const noexcept {return pin;}
    uint16_t GetFrequency() const noexcept {return frequency;}

    void Interrupt() noexcept;

#ifdef LPC_DEBUG
    void IncrementLateCount() noexcept;
    uint32_t GetLateCount() noexcept { return lateCount; };
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
    
    void ScheduleEvent(uint32_t timeout) noexcept;

    
};



#endif
