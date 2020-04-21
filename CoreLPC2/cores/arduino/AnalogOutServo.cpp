//Author: sdavi
#include "Core.h"
#include "chip.h"

//Timer PWM
#define TimerPWM_Slot1 (0x01)
#define TimerPWM_Slot2 (0x02)
#define TimerPWM_Slot3 (0x04)


extern "C" void debugPrintf(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));


struct TimerPwm_t
{
    LPC_TIMER_T *timer;
    Pin* const timerPins;
    uint16_t frequency;
};

Pin Timer2PWMPins[MaxTimerEntries] = {NoPin, NoPin, NoPin};
static uint8_t servoOutputUsed = 0;
static bool servoTimerInitialised = false;
static const TimerPwm_t ServoTimerPWM = {LPC_TIMER2, Timer2PWMPins, 50};
static uint32_t pinsOnATimer[5] = {0}; // 5 Ports



bool IsServoCapable(Pin pin) noexcept
{
    const uint8_t port = (pin >> 5);
    const uint32_t portPinPosition = 1 << (pin & 0x1f);
    uint8_t count = 0;

    if(pin > MaxPinNumber || port > 4) return false;
    
    //First see if the pin can use the HWPWM
    if(HardwarePWMFrequency == 50 && CanDoHWPWM(pin)) return true; //HWPWM configured at servo freq and is PWM capable
    
    //Check if there is a free slot on the Timer
    
    if( (pinsOnATimer[port] & portPinPosition) ) return true; //already setup as a Timer servo
    for(uint8_t t=0; t<MaxTimerEntries; t++)
    {
        if(ServoTimerPWM.timerPins[t] == NoPin)
        {
            count++;
        }
    }
    
    return (count <= MaxTimerEntries);
}

void ReleaseServoPin(Pin pin) noexcept
{
    const uint8_t port = (pin >> 5);
    const uint32_t portPinPosition = 1 << (pin & 0x1f);

    
    //First check if pin was running on HWPWM
    if(ReleaseHWPWMPin(pin)) return; //Servo was running on HWPWM and was released


    if( !(pinsOnATimer[port] & portPinPosition) ) return; //pin not configured as Timer Servo
    //Send one more AnalogWriteServo with value 0 to turn off the match interrupt for that slot
    AnalogWriteServo(0.0, ServoTimerPWM.frequency, pin);
    
    //find the pin (this pin has already been checked if it's on a timer)
    for(size_t j=0; j<MaxTimerEntries; j++)
    {
        if(ServoTimerPWM.timerPins[j] == pin)
        {
            ServoTimerPWM.timerPins[j] = NoPin;
            return;
        }
    }
}


bool ConfigurePinForServo(Pin pin, bool outputHigh) noexcept
{
    const uint8_t port = (pin >> 5);
    const uint32_t portPinPosition = 1 << (pin & 0x1f);
    
    if (pin == NoPin || pin > MaxPinNumber || port > 4) return false;
    
    
    //First try HW PWM for servo
    if(ConfigurePinForHWPWM(pin, outputHigh))
    {
        return true; //Pin was configured to use HWPWM
    }

    
    //Next try Servo PWM on Timer
    
    if( (pinsOnATimer[port] & portPinPosition) ) return true; // This pin is already configured to be a Timer Servo
    //get next free Slot
    for(uint8_t t=0; t<MaxTimerEntries; t++)
    {
        if(ServoTimerPWM.timerPins[t] == NoPin)
        {
            //Timer has slot free, add pin to the timerPins array
            ServoTimerPWM.timerPins[t] = pin;
            
            //Update the  bitmask to tell analogWrite that pin is now ServoPWM capable
            pinsOnATimer[port] |= 1 << (pin & 0x1f);
            return true;
        }
    }
    
    return false;
}


//Initialse a timer
static inline void initServoTimer() noexcept
{
    Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_TIMER2); //enable power and clocking

    NVIC_EnableIRQ(TIMER2_IRQn);

    ServoTimerPWM.timer->PR   =  getPrescalarForUs(PCLK_TIMER2); // Prescalar for 1us... every 1us TC is incremented
    ServoTimerPWM.timer->MR[0] = 1000000/ServoTimerPWM.frequency; //The PWM Period in us
    ServoTimerPWM.timer->TC  = 0x00;  // Reset the Timer Count to 0
    ServoTimerPWM.timer->MCR = ((1<<SBIT_MR0I) | (1<<SBIT_MR0R));     // Int on MR0 match and Reset Timer on MR0 match
    ServoTimerPWM.timer->TCR  = (1 <<SBIT_CNTEN); //Start Timer

    servoTimerInitialised = true;
}


bool AnalogWriteServo(float ulValue, uint16_t freq, Pin pin) noexcept
pre(0.0 <= ulValue; ulValue <= 1.0)
{
    uint8_t slot = 0;
    const uint8_t port = (pin >> 5);
    const uint32_t portPinPosition = 1 << (pin & 0x1f);

    if( !(pinsOnATimer[port] & portPinPosition) )
    {
        return false; // This pin is not configured to be a TimerServo
    }

    
    //find the pin (this pin has already been checked if it's on a timer)
    for(size_t j=0; j<MaxTimerEntries; j++)
    {
        if(ServoTimerPWM.timerPins[j] == pin)
        {
            slot = (1 << j); //Timer slot bitmask. TimerPWM_Slot1 -> TimerPWM_Slot3
            break;
        }
    }
    
    if(slot == 0) return false; // failed to find slot

    //once timers are turned on, we dont turn off again. We only turn them on when first requested.
    if( !servoTimerInitialised )
    {
        initServoTimer();
    }
    
    uint32_t onTime = 0; //default to off
    onTime = (uint32_t)((float)(1000000/ServoTimerPWM.frequency) * ulValue); //if the requested freq was not 0, but we use the fixed TimerFreq
    
    if(onTime >= ServoTimerPWM.timer->MR[0])
    {
        onTime = ServoTimerPWM.timer->MR[0]+1; // set above MR0 (prevents triggering an int and briefly turning off before reset)
    }
        

    switch(slot)
    {
        case TimerPWM_Slot1:
            if((servoOutputUsed & TimerPWM_Slot1) && ServoTimerPWM.timer->MR[1] == onTime)
            {
                //already running
            }
            else
            {
                ServoTimerPWM.timer->MR[1] = onTime; //The LOW time in us
                if(onTime == 0)
                {
                    ServoTimerPWM.timer->MCR &= ~(1<<SBIT_MR1I);//  No Int on MR1 Match
                }
                else
                {
                    ServoTimerPWM.timer->MCR |= (1<<SBIT_MR1I);//  Int on MR1 Match
                }
                servoOutputUsed |= TimerPWM_Slot1;
            }
            break;
            
        case TimerPWM_Slot2:
            if((servoOutputUsed & TimerPWM_Slot2) && ServoTimerPWM.timer->MR[2] == onTime)
            {
                //already running
            }
            else
            {
                ServoTimerPWM.timer->MR[2] = onTime; //The LOW time in us
                if(onTime == 0)
                {
                    ServoTimerPWM.timer->MCR &= ~(1<<SBIT_MR2I);//  No Int on MR2 Match
                }
                else
                {
                    ServoTimerPWM.timer->MCR |= (1<<SBIT_MR2I);     //  Int on MR2 Match
                }
                servoOutputUsed |= TimerPWM_Slot2;
            }
            break;

        case TimerPWM_Slot3:
            if((servoOutputUsed & TimerPWM_Slot3) && ServoTimerPWM.timer->MR[3] == onTime)
            {
                //already running
            }
            else
            {
                ServoTimerPWM.timer->MR[3] = onTime; //The LOW time in us
                if(onTime == 0)
                {
                    ServoTimerPWM.timer->MCR &= ~(1<<SBIT_MR3I);//  No Int on MR3 Match
                }
                else
                {
                    ServoTimerPWM.timer->MCR |= (1<<SBIT_MR3I);     //  Int on MR3 Match
                }
                servoOutputUsed |= TimerPWM_Slot3;

            }
            break;
    }

    return true;
}



static inline void servoFunctionResetPeriod() noexcept
{
    if( servoOutputUsed == 0 ) return; // nothing to do

    
    //Determine which pins need to be set High for the start of the PWM Period
    if( servoOutputUsed & TimerPWM_Slot1 ) // if output pin is enabled
    {
        if(ServoTimerPWM.timer->MR[1] > 0){
            pinMode(ServoTimerPWM.timerPins[0], OUTPUT_HIGH ); //go HIGH if we have a OnTime Set
        } else {
            pinMode(ServoTimerPWM.timerPins[0], OUTPUT_LOW ); //else keep next cycle off
        }
    }
    
    if( servoOutputUsed & TimerPWM_Slot2 ) // if output pin is enabled
    {
        if(ServoTimerPWM.timer->MR[2] > 0){
            pinMode(ServoTimerPWM.timerPins[1], OUTPUT_HIGH );
        } else {
            pinMode(ServoTimerPWM.timerPins[1], OUTPUT_LOW );
        }
    }
    
    if( servoOutputUsed & TimerPWM_Slot3 ) // if output pin is enabled
    {
        if(ServoTimerPWM.timer->MR[3] > 0){

            pinMode(ServoTimerPWM.timerPins[2], OUTPUT_HIGH );
        } else {
            pinMode(ServoTimerPWM.timerPins[2], OUTPUT_LOW );
        }
    }

}


extern "C" void TIMER2_IRQHandler(void) noexcept
{
    LPC_TIMER_T *timer = ServoTimerPWM.timer;
    uint32_t regval = timer->IR;
    
    timer->TCR  = 0; //Stop the timer
    
    //When this int is called, we check if MR1-3 triggered int, if so, the output needs to go LOW
    
    if (regval & (1 << SBIT_MRI1_IFM)) //Interrupt flag for match channel 1.
    {
        timer->IR |= (1<<SBIT_MRI1_IFM); //clear interrupt on MR1 (setting bit will clear int)
        if(servoOutputUsed & TimerPWM_Slot1)
        {
            pinMode(ServoTimerPWM.timerPins[0], OUTPUT_LOW );
        }
    }
    
    if (regval & (1 << SBIT_MRI2_IFM)) //Interrupt flag for match channel 2.
    {
        timer->IR |= (1<<SBIT_MRI2_IFM); //clear interrupt on MR2 (setting bit will clear int)
        if(servoOutputUsed & TimerPWM_Slot2)
        {
            pinMode(ServoTimerPWM.timerPins[1], OUTPUT_LOW );
        }
    }
    
    if (regval & (1 << SBIT_MRI3_IFM)) //Interrupt flag for match channel 3.
    {
        timer->IR |= (1<<SBIT_MRI3_IFM); //clear interrupt on MR3 (setting bit will clear int)
        if(servoOutputUsed & TimerPWM_Slot3)
        {
            pinMode(ServoTimerPWM.timerPins[2], OUTPUT_LOW );
        }
    }
    
    //MR0 is the Period, we need to reset all the channels to the start of the period
    if (regval & (1 << SBIT_MRI0_IFM)) //Interrupt flag for match channel 0 (
    {
        // TC will be reset... set all connected pins high for start of the cycle
        timer->IR |= (1<<SBIT_MRI0_IFM); //clear interrupt on MR0 (setting bit will clear int)
        servoFunctionResetPeriod(); //start the cycle HIGH, MR1-3 will set the "on" times for each channel.
    }
    timer->TCR  = (1 <<SBIT_CNTEN); //start the timer again
}



