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

    if(PWMValue[chan] == ulValue && PWMFreq == freq ) return true; // already running at this PWM+freq, Nothing more to do here

    
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
        
        uint32_t ticks = pwm_clock_mhz * (1000000/freq);
        LPC_PWM1->TCR = TCR_RESET; //set reset
        LPC_PWM1->MR0 = ticks; //Set the period (for ALL channels)
        
        // set the channel latch to update value at next period start
        LPC_PWM1->LER |= 1 << 0;
        
        //Frequency of PWM channel has changed, we need to recalculate all the channels so their duty cycle remains the same
        LPC_PWM1->MR1 = (uint32_t)((float)(LPC_PWM1->MR0) * PWMValue[0]);
        LPC_PWM1->MR2 = (uint32_t)((float)(LPC_PWM1->MR0) * PWMValue[1]);
        LPC_PWM1->MR3 = (uint32_t)((float)(LPC_PWM1->MR0) * PWMValue[2]);
        LPC_PWM1->MR4 = (uint32_t)((float)(LPC_PWM1->MR0) * PWMValue[3]);
        LPC_PWM1->MR5 = (uint32_t)((float)(LPC_PWM1->MR0) * PWMValue[4]);
        LPC_PWM1->MR6 = (uint32_t)((float)(LPC_PWM1->MR0) * PWMValue[5]);

        
        // enable counter and pwm, clear reset
        LPC_PWM1->TCR = TCR_CNT_EN | TCR_PWM_EN;

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

static uint8_t  outputUsed[numTimerChannels] = {0};
static uint8_t  timerInitialised = 0;



//Externs Defined in pins_lpc in RRF
extern const Pin Timer1PWMPins[3];
extern const Pin Timer2PWMPins[3];
extern const Pin Timer3PWMPins[3];
extern const uint16_t Timer1PWMFrequency;
extern const uint16_t Timer2PWMFrequency;
extern const uint16_t Timer3PWMFrequency;
extern const uint8_t TimerPWMPinsArray[MaxPinNumber]; //defined in pins_lpc lookup array to see which pins we defined can use TimerPWM


struct TimerPwmStruct{
    LPC_TIM_TypeDef *timer;
    const Pin* timerPins;
    uint16_t frequency;
};


static const TimerPwmStruct TimerPWMs[numTimerChannels] = {
    {LPC_TIM1, Timer1PWMPins, Timer1PWMFrequency}, //chan 0
    {LPC_TIM2, Timer2PWMPins, Timer2PWMFrequency}, //chan 1
    {LPC_TIM3, Timer3PWMPins, Timer3PWMFrequency}  //chan 2
};


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
            NVIC_SetPriority(TIMER2_IRQn, 9); //Timer Priority
            NVIC_EnableIRQ(TIMER2_IRQn);

            TimerPWMs[chan].timer->PR   =  getPrescalarForUs(PCLK_TIMER2); // Prescalar for 1us... every 1us TC is incremented

    }
     else if(TimerPWMs[chan].timer == LPC_TIM3){
            LPC_SC->PCONP |= ((uint32_t)1<<SBIT_PCTIM3); // Ensure the Power bit is set for the Timer3
            NVIC_SetPriority(TIMER3_IRQn, 10); //Timer Priority
            NVIC_EnableIRQ(TIMER3_IRQn);

            TimerPWMs[chan].timer->PR   =  getPrescalarForUs(PCLK_TIMER3); // Prescalar for 1us... every 1us TC is incremented
         

    }

    TimerPWMs[chan].timer->MR0 = 1000000/TimerPWMs[chan].frequency; //The PWM Period in us
    TimerPWMs[chan].timer->TC  = 0x00;  // Reset the Timer Count to 0
    TimerPWMs[chan].timer->MCR = (1<<SBIT_MR0I) | (1<<SBIT_MR0R);     // Int on MR0 match and Reset Timer on MR0 match
    TimerPWMs[chan].timer->TCR  = (1 <<SBIT_CNTEN); //Start Timer

    timerInitialised |= (1 << chan);
    
}


static bool AnalogWriteTimer(float ulValue, uint16_t freq, Pin pin)
pre(0.0 <= ulValue; ulValue <= 1.0)
{

    if(freq == 0) return false;
    
    uint8_t timerChannel = 0;
    

    switch(TimerPWMPinsArray[pin] & 0x0F) //timer number is on lower bits
    {
        case TimerPWM_1:
            timerChannel = 0;
            break;
        case TimerPWM_2:
            timerChannel = 1;
            break;
        case TimerPWM_3:
            timerChannel = 2;
            break;
        default:
            return false;
    }
    

    if(TimerPWMs[timerChannel].frequency != freq) return false; // requested a timer pin but not at the freq the timer is set for
    if(TimerPWMs[timerChannel].timer == nullptr ) return false;

    //once timers are turned on, we dont turn off again


    if( !(timerInitialised & (1 << timerChannel))  ){
        initTimer(timerChannel);
    }

    
//    if(freq != timerFrequency[chan] ){
//        initTimer(timer, freq);
//        timerFrequency[chan] = freq;
//
//    }


    
    uint32_t onTime = (1000000/freq) * ulValue;
    uint32_t offTime;
    
    if(TimerPWMs[timerChannel].timer->MR0 - onTime > 0) offTime = TimerPWMs[timerChannel].timer->MR0 - onTime;
    else offTime = TimerPWMs[timerChannel].timer->MR0 + 1; //set above MR0 (aka period) ... will not trigger int when off
    
        
    if(onTime == 0)
    {
        //pin off
        
//        if(outputUsed[chan] == 0){
//            //no outputs enabled, nothing to do
//        }
//        else
//        {
//
//            switch(TimerPWMPinsArray[pin] & 0xF0) // slot number is on upper bits
//            {
//                case TimerPWM_Slot1:
//
//                    timer->MCR &= ~(1<<SBIT_MR1I);     // disbaled Int on MR1 Match
//                    outputUsed[chan] &= ~TimerPWM_Slot1;
//
//                    break;
//
//                case TimerPWM_Slot2:
//                    timer->MCR &= ~(1<<SBIT_MR2I);     // disbaled Int on MR2 Match
//                    outputUsed[chan] &= ~TimerPWM_Slot2;
//                    break;
//
//                case TimerPWM_Slot3:
//                    timer->MCR &= ~(1<<SBIT_MR3I);     // disbaled Int on MR3 Match
//                    outputUsed[chan] &= ~TimerPWM_Slot3;
//                    break;
//            }
//
//            pinMode(pin, OUTPUT_LOW );
//        }
    }
    else
    {
        switch(TimerPWMPinsArray[pin] & 0xF0) // slot number is on upper bits
        {
            case TimerPWM_Slot1:
                TimerPWMs[timerChannel].timer->MR1 = offTime; //The LOW time in us
                TimerPWMs[timerChannel].timer->MCR |= (1<<SBIT_MR1I);     //  Int on MR1 Match
                outputUsed[timerChannel] |= TimerPWM_Slot1;
                break;
                
            case TimerPWM_Slot2:
                TimerPWMs[timerChannel].timer->MR2 = offTime; //The LOW time in us
                TimerPWMs[timerChannel].timer->MCR |= (1<<SBIT_MR2I);     //  Int on MR2 Match
                outputUsed[timerChannel] |= TimerPWM_Slot2;
                break;

            case TimerPWM_Slot3:
                TimerPWMs[timerChannel].timer->MR3 = offTime; //The LOW time in us
                TimerPWMs[timerChannel].timer->MCR |= (1<<SBIT_MR3I);     //  Int on MR3 Match
                outputUsed[timerChannel] |= TimerPWM_Slot3;
                break;
        }



    }

//    if(outputUsed[chan] == 0)
//    {
//        timer->TCR  &= ~(1 <<SBIT_CNTEN); //Stop Timer
//        timer->TC  = 0x00;  // Restart the Timer Count
//    }
//    else
//    {
//        timer->TCR  = (1 <<SBIT_CNTEN); //Start Timer
//    }

    return true;
}



static inline void timerFunctionResetPeriod(uint8_t timerChannel)
{
    if( outputUsed[timerChannel] == 0 ) return; // nothing to do

    
    if( outputUsed[timerChannel] & TimerPWM_Slot1 ) // if output pin is enabled
    {
        pinMode(TimerPWMs[timerChannel].timerPins[0], OUTPUT_LOW );
    }
    if( outputUsed[timerChannel] & TimerPWM_Slot2 ) // if output pin is enabled
    {
        pinMode(TimerPWMs[timerChannel].timerPins[1], OUTPUT_LOW );
    }
    if( outputUsed[timerChannel] & TimerPWM_Slot3 ) // if output pin is enabled
    {
        pinMode(TimerPWMs[timerChannel].timerPins[2], OUTPUT_LOW );
    }

}

static inline void timerFunction(uint8_t timerChannel)
{
    
    LPC_TIM_TypeDef *timer = TimerPWMs[timerChannel].timer;
    
    uint32_t regval = timer->IR;
    //find which Match Register triggered the interrupt
    if (regval & (1 << SBIT_MRI0_IFM)) //Interrupt flag for match channel 0 (
    {
        // TC will be reset... set all connected pins high for start of the cycle
        timer->IR |= (1<<SBIT_MRI0_IFM); //clear interrupt on MR0 (setting bit will clear int)
        timerFunctionResetPeriod(timerChannel); //start the cycle LOW, MR1-3 will set the "off" time.
        
    }

    if (regval & (1 << SBIT_MRI1_IFM)) //Interrupt flag for match channel 1.
    {
        timer->IR |= (1<<SBIT_MRI1_IFM); //clear interrupt on MR1 (setting bit will clear int)
        if(outputUsed[timerChannel] & TimerPWM_Slot1)
        {
            pinMode(TimerPWMs[timerChannel].timerPins[0], OUTPUT_HIGH );
        }
    }

    if (regval & (1 << SBIT_MRI2_IFM)) //Interrupt flag for match channel 2.
    {
        timer->IR |= (1<<SBIT_MRI2_IFM); //clear interrupt on MR2 (setting bit will clear int)
        if(outputUsed[timerChannel] & TimerPWM_Slot2)
        {
            pinMode(TimerPWMs[timerChannel].timerPins[1], OUTPUT_HIGH );
        }
    }

    if (regval & (1 << SBIT_MRI3_IFM)) //Interrupt flag for match channel 3.
    {
        timer->IR |= (1<<SBIT_MRI3_IFM); //clear interrupt on MR3 (setting bit will clear int)
        if(outputUsed[timerChannel] & TimerPWM_Slot3)
        {
            pinMode(TimerPWMs[timerChannel].timerPins[2], OUTPUT_HIGH );
        }
    }
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
    
    if (TimerPWMPinsArray[pin] != 0)
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
void GetTimerInfo( uint16_t freqs[4] ){
    freqs[0] = PWMFreq; //Hardware PWM
    //Timer PWMs
    for(int i=0; i<numTimerChannels; i++){
        if( (timerInitialised & (1<<i)) )
        {
            freqs[i+1] = TimerPWMs[i].frequency;
        }
        else
        {
            freqs[i+1] = 0;
        }
        
    }
    
    
}



// End
