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



extern "C" void debugPrintf(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));

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


static bool PWMEnabled = false;
Pin UsedHardwarePWMChannel[NumPwmChannels] = {NoPin, NoPin, NoPin, NoPin, NoPin, NoPin};
uint16_t HardwarePWMFrequency = 250;
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
    

    //if (PWMFreq != freq) // A change of Freq is requested. !!!This changes the Freq for ALL PWM channels!!!
    if(!PWMEnabled)
    {
        //Enable and Setup HW PWM

        //Power on
        LPC_SC->PCONP |= 1 << 6;

        //pclk set to 1/4 in system_LPC17xx.c
        LPC_PWM1->PR = 0; // no pre-scale
        
        LPC_PWM1->IR = 0;// Disable all interrupt flags for PWM
        //NOTE::: Manual states LPC pwm pins all share the same period!

        LPC_PWM1->MCR = 1 << 1; // Single PWMMode -> reset TC on match 0
        pwm_clock_mhz = SystemCoreClock / 4000000;

        LPC_PWM1->TCR = TCR_RESET; //set reset

        uint32_t ticks = pwm_clock_mhz * (1000000/HardwarePWMFrequency);
        LPC_PWM1->MR0 = ticks; //Set the period (for ALL channels)
        LPC_PWM1->LER|= LER0_EN;
        
        // enable counter and pwm, clear reset
        LPC_PWM1->TCR = TCR_CNT_EN | TCR_PWM_EN;

        PWMEnabled = true;
    }

    if(freq != HardwarePWMFrequency) return false;

    
    //setup the PWM channel
    uint32_t v = (uint32_t)((float)(LPC_PWM1->MR0) * ulValue); //calculate duty cycle

    //SD:: this is workaround from MBED
    // workaround for PWM1[1] - Never make it equal MR0, else we get 1 cycle dropout
    if (v == LPC_PWM1->MR0) {
        v++;
    }

    PWM_Set(pin, v); //set the duty cycle for the pin

    return true;
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
	
    
    //is the pin configured as a Servo?
    if (AnalogWriteServo(ulValue, freq, pin))
    {
        return;
    }

    
    //Try HW PWM
//    if ((attr & PIN_ATTR_PWM) != 0)
//    {
//        if (AnalogWritePwm(pinDesc, ulValue, freq, pin))
//        {
//            return;
//        }
//    }

    //is the pin configured as a Software PWM?
    if (AnalogWriteSoftwarePWM(ulValue, freq, pin))
    {
        return;
    }
    
    
	// Fall back to digital write
	pinMode(pin, (ulValue < 0.5) ? OUTPUT_LOW : OUTPUT_HIGH);
}


 // Return true if this pin exists and can do PWM
bool IsPwmCapable(Pin pin)
{
    if (pin > MaxPinNumber ) return false;
    return CanDoSoftwarePWM(pin);
}

bool ConfigurePinForPWM(Pin pin, bool outputHigh)
{
    return ConfigurePinForSoftwarePWM(pin);
}

void ReleasePWMPin(Pin pin)
{
    ReleaseSoftwarePWMPin(pin);
}

//bool ConfigurePinForPWM(Pin pin, uint16_t frequency)
//{
//    const uint8_t port = (pin >> 5);
//    if(pin == NoPin || frequency == 0 || port > 4) return false;
//
//    //Check for HW Pin
//    for(uint8_t i=0; i<size(PinMap_PWM); i++)
//    {
//        if(PinMap_PWM[i].pinNumber == pin)
//        {
//            //the pin is HWPWM, check HWPWM is running at the requested frequency
//            if(HardwarePWMFrequency == frequency)
//            {
//
//                if(UsedHardwarePWMChannel[PinMap_PWM[i].pwmChannelNumber] == pin)
//                {
//                    //pin already configured
//                    return true;
//                }
//                //some PWM pins share the same pwm channel, so check another pin on same channel is not in use
//                if(UsedHardwarePWMChannel[PinMap_PWM[i].pwmChannelNumber] == NoPin){
//                    UsedHardwarePWMChannel[PinMap_PWM[i].pwmChannelNumber] = pin; //Pin will use HW PWM
//                    return true;
//                }
//            }
//            else
//            {
//                break;
//            }
//        }
//    }
//    return false; //Pin unable to do PWM
//}

// End
