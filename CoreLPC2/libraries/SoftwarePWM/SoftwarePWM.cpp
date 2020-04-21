//Author sdavi

#include "SoftwarePWM.h"
#include "SoftwarePWMTimer.h"

extern "C" void debugPrintf(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));


SoftwarePWM::SoftwarePWM(Pin softPWMPin) noexcept
{
    SetFrequency(1); //default to 1Hz
    
    pwmRunning = false;
#ifdef LPC_DEBUG
    lateCount = 0;
#endif
    pin = softPWMPin;
    pinMode(pin, OUTPUT_LOW);
    state = PWM_OFF;

}

#ifdef LPC_DEBUG
void SoftwarePWM::IncrementLateCount() noexcept
{
    lateCount++;
}
#endif

void SoftwarePWM::Enable() noexcept
{
    pinMode(pin, OUTPUT_LOW);
    state = PWM_OFF;
    
    pwmRunning = true;
    
    ScheduleEvent(1); //Schedule to start the PWM

}
void SoftwarePWM::Disable() noexcept
{
    softwarePWMTimer.RemoveEvent(&event); //remove event from the ticker

    pinMode(pin, OUTPUT_LOW);
    state = PWM_OFF;
    
    pwmRunning = false;
}

//Sets the freqneucy in Hz
void SoftwarePWM::SetFrequency(uint16_t freq) noexcept
{
    frequency = freq;
    //find the period in us
    period = 1000000/freq;
    
}


void SoftwarePWM::SetDutyCycle(float duty) noexcept
{
    uint32_t ot = (uint32_t) ((float)(period * duty));
    if(ot > period) ot = period;
    
    onTime = ot; //update the Duty
}

//PWM On phase
void SoftwarePWM::PWMOn() noexcept
{
    state = PWM_ON;
    pinMode(pin, OUTPUT_HIGH);
}
//PWM Off Phase
void SoftwarePWM::PWMOff() noexcept
{
    state = PWM_OFF;
    pinMode(pin, OUTPUT_LOW);
}


//Schedule next even in now+timeout microseconds
inline void SoftwarePWM::ScheduleEvent(uint32_t timeout) noexcept
{
    softwarePWMTimer.ScheduleEventInMicroseconds(&event, timeout, this);
    nextRun = event.timestamp;
}

void SoftwarePWM::Check() noexcept
{
    if(pwmRunning == true && (int)(nextRun - softwarePWMTimer.TickerRead()) < -4*(int)softwarePWMTimer.TicksPerMicrosecond()) // is it more than 4us overdue?
    {
        //PWM is overdue, has it stopped running ?
        //TODO:: check if ticker int has not fired recently.
        
        PWMOff(); // Disable the PWM for protection
        debugPrintf("PWM Overdue! (%d.%d)\n", (pin >> 5), (pin & 0x1f));
        
    }
}

void SoftwarePWM::Interrupt() noexcept
{
    //handle 100% on/off
    if(onTime==0)
    {
        PWMOff();
        //schedule next int in +period
        ScheduleEvent(period);
        return;
    }
    else if(onTime==period)
    {
        PWMOn();
        //schedule next int in +period
        ScheduleEvent(period);
        return;
    }
    
    
    if(state==PWM_OFF)
    {
        //last state was off, turn on
        PWMOn();
        ScheduleEvent(onTime);
    }
    else
    {
        //last state was On, turn off
        PWMOff();
        ScheduleEvent(period-onTime);
    }

}


