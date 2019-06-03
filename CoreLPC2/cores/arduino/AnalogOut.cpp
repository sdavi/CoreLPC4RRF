/*
 Copyright (c) 2011 Arduino.  All right reserved.

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 See the GNU Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

//SD: Modified to support LPC

#include "Core.h"
#include "AnalogOut.h"

#include "FreeRTOS.h"
#include "Task.h"


#define TCR_CNT_EN       0x00000001
#define TCR_RESET        0x00000002
#define TCR_PWM_EN       0x00000008

#define MR0_INT            (1 << 0)
#define MR1_INT            (1 << 1)
#define MR2_INT            (1 << 2)
#define MR3_INT            (1 << 3)
#define MR4_INT            (1 << 8)
#define MR5_INT            (1 << 9)
#define MR6_INT            (1 << 10)


#define PWMSEL2            (1 << 2)
#define PWMSEL3            (1 << 3)
#define PWMSEL4            (1 << 4)
#define PWMSEL5            (1 << 5)
#define PWMSEL6            (1 << 6)
#define PWMENA1            (1 << 9)
#define PWMENA2            (1 << 10)
#define PWMENA3            (1 << 11)
#define PWMENA4            (1 << 12)
#define PWMENA5            (1 << 13)
#define PWMENA6            (1 << 14)


#define LER0_EN            (1 << 0)
#define LER1_EN            (1 << 1)
#define LER2_EN            (1 << 2)
#define LER3_EN            (1 << 3)
#define LER4_EN            (1 << 4)
#define LER5_EN            (1 << 5)
#define LER6_EN            (1 << 6)

template<class T, size_t N>
constexpr size_t size(T (&)[N]) { return N; }


typedef struct
{
    gpioPins_et pinNumber;
    uint8_t pwmChannelNumber;
    uint8_t PinFunSel;
} pwmChannelConfig_st;


//  PORT ID, PWM ID, Pin function

static const pwmChannelConfig_st PinMap_PWM[] = {
    {P1_18, PWM1_1, 2},
    {P1_20, PWM1_2, 2},
    {P1_21, PWM1_3, 2},
    {P1_23, PWM1_4, 2},
    {P1_24, PWM1_5, 2},
    {P1_26, PWM1_6, 2},
    {P2_0 , PWM1_1, 1},
    {P2_1 , PWM1_2, 1},
    {P2_2 , PWM1_3, 1},
    {P2_3 , PWM1_4, 1},
    {P2_4 , PWM1_5, 1},
    {P2_5 , PWM1_6, 1},
    {P3_25, PWM1_2, 3},
    {P3_26, PWM1_3, 3},
};



// Initialise this module
extern void AnalogOutInit()
{
	// Nothing to do yet
}

// Convert a float in 0..1 to unsigned integer in 0..N
static inline uint32_t ConvertRange(float f, uint32_t top)
    pre(0.0 <= ulValue; ulValue <= 1.0)
    post(result <= top)
{
	return lround(f * (float)top);
}

/*
// AnalogWrite to a DAC pin
// Return true if successful, false if we need to fall back to digitalWrite
static bool AnalogWriteDac(const PinDescription& pinDesc, float ulValue)
    pre(0.0 <= ulValue; ulValue <= 1.0)
    pre((pinDesc.ulPinAttribute & PIN_ATTR_DAC) != 0)
{

#warning Implement DAC function (If Needed)
    return false;/// TODO:: Implement this function
}
*/


const uint8_t numPwmChannels = 6;
static bool PWMEnabled = false;
static uint16_t PWMValue[numPwmChannels] = {0};
static uint16_t PWMFreq = 0;
static unsigned int pwm_clock_mhz;

void PWM_Set( Pin pin, uint32_t dutyCycle )
{
    //find entry in array
    uint8_t i = 0;
    
    for(i=0;i<size(PinMap_PWM); i++){
        if(PinMap_PWM[i].pinNumber == pin){

            GPIO_PinFunction(pin, PinMap_PWM[i].PinFunSel); //configure pin to PWM
            switch(PinMap_PWM[i].pwmChannelNumber){
                case PWM1_1:
                    LPC_PWM1->MR1 = dutyCycle;
                    LPC_PWM1->LER|= LER1_EN;
                    LPC_PWM1->PCR|= PWMENA1;

                    break;
                case PWM1_2:
                    LPC_PWM1->MR2 = dutyCycle;
                    LPC_PWM1->LER|= LER2_EN;
                    LPC_PWM1->PCR|= PWMENA2;

                    break;
                case PWM1_3:
                    LPC_PWM1->MR3 = dutyCycle;
                    LPC_PWM1->LER|= LER3_EN;
                    LPC_PWM1->PCR|= PWMENA3;

                    break;
                case PWM1_4:
                    LPC_PWM1->MR4 = dutyCycle;
                    LPC_PWM1->LER|= LER4_EN;
                    LPC_PWM1->PCR|= PWMENA4;

                    break;
                case PWM1_5:
                    LPC_PWM1->MR5 = dutyCycle;
                    LPC_PWM1->LER|= LER5_EN;
                    LPC_PWM1->PCR|= PWMENA5;

                    break;
                case PWM1_6:
                    LPC_PWM1->MR6 = dutyCycle;
                    LPC_PWM1->LER|= LER6_EN;
                    LPC_PWM1->PCR|= PWMENA6;

                    break;
                    
            }
        }
        
    }
}




// AnalogWrite to a PWM pin
// Return true if successful, false if we need to fall back to digitalWrite
static bool AnalogWritePwm(const PinDescription& pinDesc, float ulValue, uint16_t freq, Pin pin)
    pre(0.0 <= ulValue; ulValue <= 1.0)
    pre((pinDesc.ulPinAttribute & PIN_ATTR_PWM) != 0)
{
    
    const uint32_t chan = pinDesc.ulPWMChannel;
    if (freq == 0)
    {
        return false;
    }

    
    if (PWMFreq != freq) // A change of Freq is requested. !!!This changes the Freq for ALL PWM channels!!!
    {
        

        
        if (!PWMEnabled)
        {
            //Power on
            LPC_SC->PCONP |= 1 << 6;

            //pclk set to 1/4 in system_LPC17xx.c
            LPC_PWM1->PR = 0; // no pre-scale
            
            LPC_PWM1->IR = 0;// Disable all interrupt flags for PWM
            //NOTE::: Manual states LPC pwm pins all share the same period!

            LPC_PWM1->MCR = 1 << 1; // Single PWMMode -> reset TC on match 0
            pwm_clock_mhz = SystemCoreClock / 4000000;
            
            PWMEnabled = true;
        }

        LPC_PWM1->TCR = TCR_RESET; //set reset

        uint32_t ticks = pwm_clock_mhz * (1000000/freq);
        LPC_PWM1->MR0 = ticks; //Set the period (for ALL channels)
        LPC_PWM1->LER|= LER0_EN;
        
        // enable counter and pwm, clear reset
        LPC_PWM1->TCR = TCR_CNT_EN | TCR_PWM_EN;

        //TODO:: if frequency changes while PWM is running their duty cycles will be wrong
        //       update all running ones.... this is not an issue in RRF as we arent changing PWM freq for HW PWM

        
        //remember the PWM freq
        PWMFreq = freq;
    }

    //setup the PWM channel
    uint32_t v = (uint32_t)((float)(LPC_PWM1->MR0) * ulValue); //calculate duty cycle

    //SD:: this is workaround from MBED (not tested but assume its required)
    // workaround for PWM1[1] - Never make it equal MR0, else we get 1 cycle dropout
    if (v == LPC_PWM1->MR0) {
        v++;
    }

    PWM_Set(pin, v); //set the duty cycle
    PWMValue[chan] = ulValue;// remember the pwm value

    return true;
}


constexpr uint8_t numTimerChannels = 3;
static uint8_t  outputUsed[numTimerChannels] = {0}; // bitmask of slots enabled on each Timer Channel
static uint8_t  timerInitialised = 0; // bitmask of timers that have been initialised


Pin Timer1PWMPins[MaxTimerEntries] = {NoPin, NoPin, NoPin};
Pin Timer2PWMPins[MaxTimerEntries] = {NoPin, NoPin, NoPin};
Pin Timer3PWMPins[MaxTimerEntries] = {NoPin, NoPin, NoPin};


struct TimerPwmStruct{
    LPC_TIM_TypeDef *timer;
    const Pin* timerPins;
    uint16_t frequency;
};


static TimerPwmStruct TimerPWMs[numTimerChannels] = {
    {LPC_TIM1, Timer1PWMPins, 10}, //chan 0
    {LPC_TIM2, Timer2PWMPins, 50}, //chan 1
    {LPC_TIM3, Timer3PWMPins, 250} //chan 2
};

uint32_t pinsOnATimer[5] = {0}; // 5 Ports

//Initialse a timer
static inline void initTimer(uint8_t chan)
{
    
    //INIT TIMER

    //TODO:: what priority to run int at?
    
    if(TimerPWMs[chan].timer == LPC_TIM1){
            LPC_SC->PCONP |= ((uint32_t)1<<SBIT_PCTIM1); // Ensure the Power bit is set for the Timer1
            NVIC_SetPriority(TIMER1_IRQn, 8); //Timer Priority
            NVIC_EnableIRQ(TIMER1_IRQn);
            TimerPWMs[chan].timer->PR   =  getPrescalarForUs(PCLK_TIMER1); // Prescalar for 1us... every 1us TC is incremented

    }
    else  if(TimerPWMs[chan].timer == LPC_TIM2){
            LPC_SC->PCONP |= ((uint32_t)1<<SBIT_PCTIM2); // Ensure the Power bit is set for the Timer2
            NVIC_SetPriority(TIMER2_IRQn, 8); //Timer Priority
            NVIC_EnableIRQ(TIMER2_IRQn);

            TimerPWMs[chan].timer->PR   =  getPrescalarForUs(PCLK_TIMER2); // Prescalar for 1us... every 1us TC is incremented

    }
     else if(TimerPWMs[chan].timer == LPC_TIM3){
            LPC_SC->PCONP |= ((uint32_t)1<<SBIT_PCTIM3); // Ensure the Power bit is set for the Timer3
            NVIC_SetPriority(TIMER3_IRQn, 8); //Timer Priority
            NVIC_EnableIRQ(TIMER3_IRQn);

            TimerPWMs[chan].timer->PR   =  getPrescalarForUs(PCLK_TIMER3); // Prescalar for 1us... every 1us TC is incremented
         

    }

    TimerPWMs[chan].timer->MR0 = 1000000/TimerPWMs[chan].frequency; //The PWM Period in us
    TimerPWMs[chan].timer->TC  = 0x00;  // Reset the Timer Count to 0
    TimerPWMs[chan].timer->MCR = ((1<<SBIT_MR0I) | (1<<SBIT_MR0R));     // Int on MR0 match and Reset Timer on MR0 match
    TimerPWMs[chan].timer->TCR  = (1 <<SBIT_CNTEN); //Start Timer

    timerInitialised |= (1 << chan);
    
}

void ConfigureTimerForPWM(uint8_t timerChannel, uint16_t frequency){
    
    if(timerChannel >=0 && timerChannel < numTimerChannels )
    {
        //save the Frequency
        TimerPWMs[timerChannel].frequency = frequency;
        
        //Update the pinsOnATimer array to indicate which pins are configured for Timer Usage
        for(size_t i=0; i<MaxTimerEntries; i++){
            Pin pin = TimerPWMs[timerChannel].timerPins[i];
            if(pin != NoPin){
                const uint8_t port = (pin >> 5);
                if(port <= 4){
                    pinsOnATimer[port] |= 1 << (pin & 0x1f);
                }
            }
        }
        
        
    }
}

static bool AnalogWriteTimer(float ulValue, uint16_t freq, Pin pin)
pre(0.0 <= ulValue; ulValue <= 1.0)
{
    
    uint8_t timerChannel = 0;
    uint8_t slot = 0;

    //find the pin (this pin has already been checked if it's on a timer)
    for(size_t i=0; i<numTimerChannels; i++)
    {
        for(size_t j=0; j<MaxTimerEntries; j++)
        {
            if(TimerPWMs[i].timerPins[j] == pin)
            {
                timerChannel = i;
                slot = (1 << j); //Timer slot bitmask. TimerPWM_Slot1 -> TimerPWM_Slot3
                goto found;
            }
        }
    }
found:
    
    if(slot == 0) return false; // failed to find slot
    
    //Default now will be to run at the timer frequency and not reject it as the user has specifically put the pin in the Timer Array
    //if(TimerPWMs[timerChannel].frequency != freq) return false; // requested a timer pin but not at the freq the timer is set for
    
    if(TimerPWMs[timerChannel].timer == nullptr ) return false;

    //once timers are turned on, we dont turn off again. We only turn them on when first requested.
    if( !(timerInitialised & (1 << timerChannel))  ){
        initTimer(timerChannel);
    }

    uint32_t onTime = 0; //default to off
    if(freq > 0)
    {
        onTime = (uint32_t)((float)(1000000/TimerPWMs[timerChannel].frequency) * ulValue); //if the requested freq was not 0, but we use the fixed TimerFreq
    }
    if(onTime >= TimerPWMs[timerChannel].timer->MR0)
    {
        onTime = TimerPWMs[timerChannel].timer->MR0+1; // set above MR0 (prevents triggering an int and briefly turning off before reset)
    }
        

    switch(slot)
    {
        case TimerPWM_Slot1:
            if((outputUsed[timerChannel] & TimerPWM_Slot1) && TimerPWMs[timerChannel].timer->MR1 == onTime)
            {
                //already running
            }
            else
            {
                TimerPWMs[timerChannel].timer->MR1 = onTime; //The LOW time in us
                if(onTime == 0)
                {
                    TimerPWMs[timerChannel].timer->MCR &= ~(1<<SBIT_MR1I);//  No Int on MR1 Match
                }
                else
                {
                    TimerPWMs[timerChannel].timer->MCR |= (1<<SBIT_MR1I);//  Int on MR1 Match
                }
                outputUsed[timerChannel] |= TimerPWM_Slot1;
            }
            break;
            
        case TimerPWM_Slot2:
            if((outputUsed[timerChannel] & TimerPWM_Slot2) && TimerPWMs[timerChannel].timer->MR2 == onTime)
            {
                //already running
            }
            else
            {
                TimerPWMs[timerChannel].timer->MR2 = onTime; //The LOW time in us
                if(onTime == 0)
                {
                    TimerPWMs[timerChannel].timer->MCR &= ~(1<<SBIT_MR2I);//  No Int on MR2 Match
                }
                else
                {
                    TimerPWMs[timerChannel].timer->MCR |= (1<<SBIT_MR2I);     //  Int on MR2 Match
                }
                outputUsed[timerChannel] |= TimerPWM_Slot2;
            }
            break;

        case TimerPWM_Slot3:
            if((outputUsed[timerChannel] & TimerPWM_Slot3) && TimerPWMs[timerChannel].timer->MR3 == onTime)
            {
                //already running
            }
            else
            {
                TimerPWMs[timerChannel].timer->MR3 = onTime; //The LOW time in us
                if(onTime == 0)
                {
                    TimerPWMs[timerChannel].timer->MCR &= ~(1<<SBIT_MR3I);//  No Int on MR3 Match
                }
                else
                {
                    TimerPWMs[timerChannel].timer->MCR |= (1<<SBIT_MR3I);     //  Int on MR3 Match
                }
                outputUsed[timerChannel] |= TimerPWM_Slot3;

            }
            break;
    }

    return true;
}



static inline void timerFunctionResetPeriod(uint8_t timerChannel)
{
    if( outputUsed[timerChannel] == 0 ) return; // nothing to do

    
    //Determine which pins need to be set High for the start of the PWM Period
    if( outputUsed[timerChannel] & TimerPWM_Slot1 ) // if output pin is enabled
    {
        if(TimerPWMs[timerChannel].timer->MR1 > 0){
            pinMode(TimerPWMs[timerChannel].timerPins[0], OUTPUT_HIGH ); //go HIGH if we have a OnTime Set
        } else {
            pinMode(TimerPWMs[timerChannel].timerPins[0], OUTPUT_LOW ); //else keep next cycle off
        }
    }
    
    if( outputUsed[timerChannel] & TimerPWM_Slot2 ) // if output pin is enabled
    {
        if(TimerPWMs[timerChannel].timer->MR2 > 0){
            pinMode(TimerPWMs[timerChannel].timerPins[1], OUTPUT_HIGH );
        } else {
            pinMode(TimerPWMs[timerChannel].timerPins[1], OUTPUT_LOW );
        }
    }
    
    if( outputUsed[timerChannel] & TimerPWM_Slot3 ) // if output pin is enabled
    {
        if(TimerPWMs[timerChannel].timer->MR3 > 0){

            pinMode(TimerPWMs[timerChannel].timerPins[2], OUTPUT_HIGH );
        } else {
            pinMode(TimerPWMs[timerChannel].timerPins[2], OUTPUT_LOW );
        }
    }

}

static inline void timerFunction(uint8_t timerChannel)
{
    
    UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
    {
        LPC_TIM_TypeDef *timer = TimerPWMs[timerChannel].timer;
    
        uint32_t regval = timer->IR;

        //When this int is called, we check if MR1-3 triggered int, if so, the output needs to go LOW
        
        if (regval & (1 << SBIT_MRI1_IFM)) //Interrupt flag for match channel 1.
        {
            timer->IR |= (1<<SBIT_MRI1_IFM); //clear interrupt on MR1 (setting bit will clear int)
            if(outputUsed[timerChannel] & TimerPWM_Slot1)
            {
                pinMode(TimerPWMs[timerChannel].timerPins[0], OUTPUT_LOW );
            }
        }

        if (regval & (1 << SBIT_MRI2_IFM)) //Interrupt flag for match channel 2.
        {
            timer->IR |= (1<<SBIT_MRI2_IFM); //clear interrupt on MR2 (setting bit will clear int)
            if(outputUsed[timerChannel] & TimerPWM_Slot2)
            {
                pinMode(TimerPWMs[timerChannel].timerPins[1], OUTPUT_LOW );
            }
        }

        if (regval & (1 << SBIT_MRI3_IFM)) //Interrupt flag for match channel 3.
        {
            timer->IR |= (1<<SBIT_MRI3_IFM); //clear interrupt on MR3 (setting bit will clear int)
            if(outputUsed[timerChannel] & TimerPWM_Slot3)
            {
                pinMode(TimerPWMs[timerChannel].timerPins[2], OUTPUT_LOW );
            }
        }
    
        //MR0 is the Period, we need to reset all the channels to the start of the period
        if (regval & (1 << SBIT_MRI0_IFM)) //Interrupt flag for match channel 0 (
        {
            // TC will be reset... set all connected pins high for start of the cycle
            timer->IR |= (1<<SBIT_MRI0_IFM); //clear interrupt on MR0 (setting bit will clear int)
            timerFunctionResetPeriod(timerChannel); //start the cycle HIGH, MR1-3 will set the "on" times for each channel.
        }
    }
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
    
}

// Interrupt Handlers for Timers 1-3
extern "C" void TIMER1_IRQHandler(void)
{
    timerFunction(0);
}
extern "C" void TIMER2_IRQHandler(void)
{
    timerFunction(1);
}
extern "C" void TIMER3_IRQHandler(void)
{
    timerFunction(2);
}




// Analog write to DAC, PWM, TC or plain output pin
// Setting the frequency of a TC or PWM pin to zero resets it so that the next call to AnalogOut with a non-zero frequency
// will re-initialise it. The pinMode function relies on this.
void AnalogOut(Pin pin, float ulValue, uint16_t freq)
{
	if (pin > MaxPinNumber || std::isnan(ulValue))
	{
		return;
	}

    ulValue = constrain<float>(ulValue, 0.0, 1.0);

	const PinDescription& pinDesc = g_APinDescription[pin];
	const uint32_t attr = pinDesc.ulPinAttribute;
	
    /*if ((attr & PIN_ATTR_DAC) != 0)
	{
		if (AnalogWriteDac(pinDesc, ulValue))
		{
			return;
		}
	}
	else*/

    const uint8_t port = (pin >> 5);
    const uint32_t portPinPosition = 1 << (pin & 0x1f);

    if(port <= 4 && (pinsOnATimer[port] & portPinPosition))
    {
        //pin is defined as PWM on Timer
        if (AnalogWriteTimer(ulValue, freq, pin))
        {
            return;
        }
    }
    else if ((attr & PIN_ATTR_PWM) != 0)
	{

		if (AnalogWritePwm(pinDesc, ulValue, freq, pin))
		{
			return;
		}
	}

	// Fall back to digital write
	pinMode(pin, (ulValue < 0.5) ? OUTPUT_LOW : OUTPUT_HIGH);
}



//get the frequencies of the pwm and timers to report in M122
void GetTimerInfo( LPCPWMInfo *pwmInfo )
{
    
    pwmInfo->hwPWMFreq = PWMFreq; //Hardware PWM
    //Timer PWMs
    pwmInfo->tim1Freq = TimerPWMs[0].frequency;
    pwmInfo->tim2Freq = TimerPWMs[1].frequency;
    pwmInfo->tim3Freq = TimerPWMs[2].frequency;
}

 // Return true if this pin exists and can do PWM
bool IsPwmCapable(Pin pin)
{

    const uint8_t port = (pin >> 5);
    const uint32_t portPinPosition = 1 << (pin & 0x1f);
    
    //check if pin is configured to use a timer for pwm
    if(port <= 4 && (pinsOnATimer[port] & portPinPosition)) return true;

    //check if pin is HW PWM capable
    if (pin < ARRAY_SIZE(g_APinDescription) && (g_APinDescription[pin].ulPinAttribute & PIN_ATTR_PWM) != 0) return true;
    
    return false;

    
}

bool IsServoCapable(Pin pin)
{
    //TODO:: assumes it is setup as pwm correctly. we should check if the pin is configured for timer2
    return IsPwmCapable(pin);
}


// End
