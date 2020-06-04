//Author: sdavi
//Servo PWM running on Timer 2

#include "Core.h"
#include "chip.h"

extern "C" void debugPrintf(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));

Pin Timer2PWMPins[MaxTimerEntries] = {NoPin, NoPin, NoPin};
static uint8_t servoOutputUsed = 0;
static bool servoTimerInitialised = false;
static const uint16_t ServoTimerFrequency = 50; //50Hz
static uint32_t pinsOnATimer[5] = {0}; // 5 Ports



bool IsServoCapable(Pin pin) noexcept
{
    const uint8_t port = (pin >> 5);
    const uint32_t portPinPosition = 1 << (pin & 0x1f);
    uint8_t count = 0;

    if(pin > MaxPinNumber || port > 4) return false;
    
    //Check if there is a free slot on the Timer
    if( (pinsOnATimer[port] & portPinPosition) ) return true; //already setup as a Timer servo
    for(uint8_t t=0; t<MaxTimerEntries; t++)
    {
        if(Timer2PWMPins[t] == NoPin)
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

    if( !(pinsOnATimer[port] & portPinPosition) ) return; //pin not configured as Timer Servo
    
    //Send one more AnalogWriteServo with value 0 to turn off the match interrupt for that slot
    AnalogWriteServo(0.0, ServoTimerFrequency, pin);
    
    //find the pin (this pin has already been checked if it's on a timer)
    for(size_t j=0; j<MaxTimerEntries; j++)
    {
        if(Timer2PWMPins[j] == pin)
        {
            Timer2PWMPins[j] = NoPin;
            pinsOnATimer[port] &= ~(portPinPosition); //update the bitmask
            return;
        }
    }
}


bool ConfigurePinForServo(Pin pin, bool outputHigh) noexcept
{
    const uint8_t port = (pin >> 5);
    const uint32_t portPinPosition = 1 << (pin & 0x1f);
    
    if (pin == NoPin || pin > MaxPinNumber || port > 4) return false;
    
    if( (pinsOnATimer[port] & portPinPosition) ) return true; // This pin is already configured to be a Timer Servo

    //get next free Slot
    for(uint8_t t=0; t<MaxTimerEntries; t++)
    {
        if(Timer2PWMPins[t] == NoPin)
        {
            //Timer has slot free, add pin to the Timer2PWMPins array
            Timer2PWMPins[t] = pin;
            
            //Update the  bitmask to tell analogWrite that pin is now ServoPWM capable
            pinsOnATimer[port] |= portPinPosition;
            
            pinMode(pin, OUTPUT_LOW );
            
            return true;
        }
    }
    
    return false;
}


//Initialse a timer
static inline void initServoTimer() noexcept
{
    Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_TIMER2); //enable power and clocking

    Chip_TIMER_PrescaleSet(LPC_TIMER2, (Chip_Clock_GetPeripheralClockRate(SYSCTL_PCLK_TIMER2)/1000000 - 1)); // Prescalar for 1us... every 1us TC is incremented
    
    //Servo PWM signal is driven by MR0
    Chip_TIMER_SetMatch(LPC_TIMER2, 0, 1000000/ServoTimerFrequency); //The PWM Period in us
    // Int on MR0 match and Reset Timer on MR0 match
    Chip_TIMER_MatchEnableInt(LPC_TIMER2, 0);
    Chip_TIMER_ResetOnMatchEnable(LPC_TIMER2, 0);

    
    LPC_TIMER2->TC  = 0x00;  // Reset the Timer Count to 0
    Chip_TIMER_Enable(LPC_TIMER2); //Start Timer

    NVIC_EnableIRQ(TIMER2_IRQn);

    servoTimerInitialised = true;
}


static inline void setServoChannel(uint8_t slot, uint32_t onTime) noexcept
{
    LPC_TIMER2->MR[slot] = onTime;
    if(onTime == 0)
    {
        Chip_TIMER_MatchDisableInt(LPC_TIMER2, slot); //  No Int on MR[x] Match
    }
    else
    {
        Chip_TIMER_MatchEnableInt(LPC_TIMER2, slot);//  Int on MR[x] Match
    }
    servoOutputUsed |= (1 << slot);
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
        if(Timer2PWMPins[j] == pin)
        {
            slot = (j+1);
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
    onTime = (uint32_t)((float)(1000000/ServoTimerFrequency) * ulValue); //if the requested freq was not 0, but we use the fixed TimerFreq
    
    if(onTime >= LPC_TIMER2->MR[0])
    {
        onTime = LPC_TIMER2->MR[0]+1; // set above MR0 (prevents triggering an int and briefly turning off before reset)
    }
        
    setServoChannel(slot, onTime);
    
    return true;
}



extern "C" void TIMER2_IRQHandler(void) noexcept __attribute__((hot));
void TIMER2_IRQHandler(void) noexcept
{
    uint32_t regval = LPC_TIMER2->IR;
    LPC_TIMER2->IR = regval; // clear the ints
    
    regval &= 0xF;
    
    //MR0 is the Servo Period, we need to reset all the channels to the start of the period
    if (regval & TIMER_MATCH_INT(0)) //Interrupt flag for match channel 0
    {
        // TC will be reset... set all connected pins high for start of the cycle

        Chip_TIMER_Disable(LPC_TIMER2); //Stop the timer
    
        uint32_t servoOutputs = servoOutputUsed;
        while(servoOutputs != 0)
        {
            unsigned int indx = LowestSetBitNumber(servoOutputs);
            servoOutputs &= ~(1u << indx);
            GPIO_PinWrite(Timer2PWMPins[indx-1], (LPC_TIMER2->MR[indx] > 0)); //Set HIGH if we have a OnTime Set
        }
    
        Chip_TIMER_Enable(LPC_TIMER2);
        return;
    }

    
    //When the int is called, we check if MR1-3 triggered int, if so, the output needs to go LOW
    while(regval != 0)
    {
        unsigned int indx = LowestSetBitNumber(regval);
        regval &= ~(1u << indx);
        GPIO_PinWrite(Timer2PWMPins[indx-1], 0);
    }
    
}



