//Author sdavi

#include "SoftwarePWM.h"
#include "SoftwarePWMTimer.h"

extern "C" void debugPrintf(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));


SoftwarePWM::SoftwarePWM(Pin softPWMPin)
{
    SetFrequency(1); //default to 1Hz
    
    pwmRunning = false;
    lateCount = 0;
    
    //us_ticker_set_handler(&SoftwarePWM::Interrupt);
    pin = softPWMPin;
    pinMode(pin, OUTPUT_LOW);
    state = PWM_OFF;

}

void SoftwarePWM::IncrementLateCount()
{
    lateCount++;
}

void SoftwarePWM::Enable()
{
    pinMode(pin, OUTPUT_LOW);
    state = PWM_OFF;
    
    pwmRunning = true;
    
    ScheduleEvent(1); //Schedule to start the PWM

}
void SoftwarePWM::Disable()
{
    softwarePWMTimer.us_ticker_remove_event(&event); //remove event from the ticker

    pinMode(pin, OUTPUT_LOW);
    state = PWM_OFF;
    
    pwmRunning = false;
}

//Sets the freqneucy in Hz
void SoftwarePWM::SetFrequency(uint16_t freq)
{
    frequency = freq;
    //ticker runs in microseconds, find the period in us
    period = 1000000/freq;
    
}


void SoftwarePWM::SetDutyCycle(float duty)
{
    uint32_t ot = (uint32_t) ((float)(period * duty));
    if(ot > period) ot = period;
    
    onTime = ot; //update the Duty
}

//PWM On phase
void SoftwarePWM::PWMOn()
{
    state = PWM_ON;
    pinMode(pin, OUTPUT_HIGH);
}
//PWM Off Phase
void SoftwarePWM::PWMOff()
{
    state = PWM_OFF;
    pinMode(pin, OUTPUT_LOW);
}


//Schedule next even in now+timeout microseconds
inline void SoftwarePWM::ScheduleEvent(uint32_t timeout)
{
    const uint32_t next = softwarePWMTimer.us_ticker_read() + timeout;
    softwarePWMTimer.us_ticker_insert_event(&event, next, this);
    nextRun = next;
}

void SoftwarePWM::Check()
{
    if(pwmRunning == true && (int)(nextRun - softwarePWMTimer.us_ticker_read()) < -4) // is it more than 4us overdue?
    {
        //PWM is overdue, has it stopped running ?
        //TODO:: check if ticker int has not fired recently.
        
        PWMOff(); // Disable the PWM for protection
        debugPrintf("PWM Overdue! (%d.%d)\n", (pin >> 5), (pin & 0x1f));
        
    }
}

void SoftwarePWM::Interrupt()
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


