
//Author: sdavi

#include "AnalogOut.h"
#include "chip.h"

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


//PCR Bits 2:6
#define PWMSEL2            (1 << 2)
#define PWMSEL3            (1 << 3)
#define PWMSEL4            (1 << 4)
#define PWMSEL5            (1 << 5)
#define PWMSEL6            (1 << 6)

//PCR Bits 9:14
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


/*------------- Pulse-Width Modulation (PWM) ---------------------------------*/
//from lpc17xx.h
typedef struct
{
    __IO uint32_t IR;
    __IO uint32_t TCR;
    __IO uint32_t TC;
    __IO uint32_t PR;
    __IO uint32_t PC;
    __IO uint32_t MCR;
    __IO uint32_t MR0;
    __IO uint32_t MR1;
    __IO uint32_t MR2;
    __IO uint32_t MR3;
    __IO uint32_t CCR;
    __I  uint32_t CR0;
    __I  uint32_t CR1;
    __I  uint32_t CR2;
    __I  uint32_t CR3;
    uint32_t RESERVED0;
    __IO uint32_t MR4;
    __IO uint32_t MR5;
    __IO uint32_t MR6;
    __IO uint32_t PCR;
    __IO uint32_t LER;
    uint32_t RESERVED1[7];
    __IO uint32_t CTCR;
} LPC_PWM_TypeDef;
#define LPC_PWM1              ((LPC_PWM_TypeDef       *) LPC_PWM1_BASE     )


extern "C" void debugPrintf(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));

template<class T, size_t N>
constexpr size_t size(T (&)[N]) { return N; }


typedef struct
{
    gpioPins_et pinNumber;
    uint8_t pwmChannelNumber;
    uint8_t PinFunSel;
} pwmChannelConfig_st;


//  Pin, PWM Channel, PinSel
static const pwmChannelConfig_st PinMap_PWM[] =
{
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



static bool PWMEnabled = false;
Pin UsedHardwarePWMChannel[NumPwmChannels] = {NoPin, NoPin, NoPin, NoPin, NoPin, NoPin};

void PWM_Set( Pin pin, uint32_t dutyCycle )
{
    //find entry in array
    for(uint8_t i=0;i<size(PinMap_PWM); i++)
    {
        if(PinMap_PWM[i].pinNumber == pin)
        {
            GPIO_PinFunction(pin, PinMap_PWM[i].PinFunSel); //configure pin to PWM
            switch(PinMap_PWM[i].pwmChannelNumber)
            {
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


void PWM_Clear( Pin pin)
{
    for(uint8_t i=0;i<size(PinMap_PWM); i++)
    {
        if(PinMap_PWM[i].pinNumber == pin)
        {
            //Disable the PWM Outout
            switch(PinMap_PWM[i].pwmChannelNumber)
            {
                case PWM1_1:
                    LPC_PWM1->PCR &= ~PWMENA1;
                    break;
                case PWM1_2:
                    LPC_PWM1->PCR &= ~PWMENA2;
                    break;
                case PWM1_3:
                    LPC_PWM1->PCR &= ~PWMENA3;
                    break;
                case PWM1_4:
                    LPC_PWM1->PCR &= ~PWMENA4;
                    break;
                case PWM1_5:
                    LPC_PWM1->PCR &= ~PWMENA5;
                    break;
                case PWM1_6:
                    LPC_PWM1->PCR &= ~PWMENA6;
                    break;
            }
        }
    }
}



// AnalogWrite to a PWM pin
// Return true if successful, false if we need to fall back to digitalWrite
bool AnalogWriteHWPWM(const PinDescription& pinDesc, float ulValue, uint16_t freq, Pin pin)
    pre(0.0 <= ulValue; ulValue <= 1.0)
    pre((pinDesc.ulPinAttribute & PIN_ATTR_PWM) != 0)
{

    //Is this pin configured for HW PWM ?
    if(pinDesc.ulPWMChannel == NOT_ON_PWM) return false;
    
    
    const uint8_t pwmChannel = (uint8_t) pinDesc.ulPWMChannel;
    if(UsedHardwarePWMChannel[pwmChannel] == NoPin) return false; //Pin PWM capable, but not configured as PWM
    
    if(freq != HardwarePWMFrequency) return false; //HW PWM not configured at the requested frequency

    
    if(!PWMEnabled)
    {
        //NOTE: Manual states LPC pwm pins all share the same period!
        //NOTE: pclk set to 1/4 in system_LPC17xx.c

        //Enable and Setup HW PWM
        Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_PWM1); //enable power and clocking
        
        LPC_PWM1->PR = 0; // no pre-scale
        LPC_PWM1->IR = 0;// Disable all interrupt flags for PWM
        LPC_PWM1->MCR = 1 << 1; // Single PWMMode -> reset TC on match 0

        LPC_PWM1->TCR = TCR_RESET; //set reset

        const unsigned int pwm_clock_mhz = SystemCoreClock / 4000000;
        uint32_t ticks = pwm_clock_mhz * (1000000/HardwarePWMFrequency);
        LPC_PWM1->MR0 = ticks; //Set the period (for ALL channels)
        LPC_PWM1->LER|= LER0_EN;
        
        // enable counter and pwm, clear reset
        LPC_PWM1->TCR = TCR_CNT_EN | TCR_PWM_EN;

        PWMEnabled = true;
    }

    uint32_t v = (uint32_t)((float)(LPC_PWM1->MR0) * ulValue); //calculate duty cycle
    if (v == LPC_PWM1->MR0) v++; //ensure not equal to MR0

    PWM_Set(pin, v); //set the duty cycle for the pin

    return true;
}

bool ReleaseHWPWMPin(Pin pin)
{
    for(uint8_t i=0; i<NumPwmChannels; i++)
    {
        if(UsedHardwarePWMChannel[i] == pin)
        {
            //stop PWM
            PWM_Clear(pin);
            UsedHardwarePWMChannel[i] = NoPin;
            return true;
        }
    }
    return false;
}

bool CanDoHWPWM(Pin pin)
{
    for(uint8_t i=0; i<size(PinMap_PWM); i++)
    {
        if(PinMap_PWM[i].pinNumber == pin)
        {
            //some PWM pins share the same pwm channel, so check another pin on same channel is not in use
            if(UsedHardwarePWMChannel[PinMap_PWM[i].pwmChannelNumber] == NoPin  //channel unused
               || UsedHardwarePWMChannel[PinMap_PWM[i].pwmChannelNumber])       //or this pin is already configured as PWM
            {
                return true;
            }
        }
    }
    return false;
}


bool ConfigurePinForHWPWM(Pin pin, bool outputHigh)
{
    if(pin == NoPin || pin > MaxPinNumber) return false;

    //Check for HW Pin
    for(uint8_t i=0; i<size(PinMap_PWM); i++)
    {
        if(PinMap_PWM[i].pinNumber == pin)
        {
            if(UsedHardwarePWMChannel[PinMap_PWM[i].pwmChannelNumber] == pin)
            {
                return true; //pin already configured
            }
            //some PWM pins share the same pwm channel, so check another pin on same channel is not in use
            if(UsedHardwarePWMChannel[PinMap_PWM[i].pwmChannelNumber] == NoPin)
            {
                UsedHardwarePWMChannel[PinMap_PWM[i].pwmChannelNumber] = pin; //Pin will use HW PWM
                return true;
            }
        }
    }
    return false; //Pin unable to do HW PWM
}

// End
