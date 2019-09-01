//Author sdavi

#include "SoftwarePWM.h"


extern "C" void debugPrintf(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));


SoftwarePWM::SoftwarePWM(Pin softPWMPin)
{
    SetFrequency(1); //default to 1Hz
    
    pwmRunning = false;
    
    us_ticker_set_handler(&SoftwarePWM::Interrupt);
    pin = softPWMPin;
    pinMode(pin, OUTPUT_LOW);
    state = PWM_OFF;

}

void SoftwarePWM::Enable()
{
    pinMode(pin, OUTPUT_LOW);
    state = PWM_OFF;
    
    pwmRunning = true;
    
    //run the handler to kick off the PWM
    Handler();

}
void SoftwarePWM::Disable()
{
    pinMode(pin, OUTPUT_LOW);
    state = PWM_OFF;
    
    pwmRunning = false;
    
    us_ticker_remove_event(&event); //remove event from the ticker
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
void SoftwarePWM::ScheduleEvent(uint32_t timeout)
{
    us_ticker_insert_event(&event, us_ticker_read() + timeout, (uint32_t)this);
}

void SoftwarePWM::Handler()
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

/*static*/ void SoftwarePWM::Interrupt(uint32_t id)
{
    SoftwarePWM *p = (SoftwarePWM*)id;
    p->Handler();
}



