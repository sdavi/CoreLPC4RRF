/*
 * AnalogInput.cpp
 *
 *  Created on: 2 Apr 2016
 *      Author: David
 
 * SD: Modified to run on LPC
 *   : - Uses the "Burst" mode of the LPC to continously sample each of the selected ADC channels once enabled
 *   : - Resolution is 12bit
 */
#include <lpc17xx.h>

#include "AnalogIn.h"

#include "adc.h"

const unsigned int numChannels = 8; //8 channels on LPC1768
static uint32_t activeChannels = 0;


const adcChannelConfig_st AdcConfig[numChannels]=
{
    { P0_23, PINSEL_FUNC_1}, /* AD0[0] is on P0.23 second alternative function */
    { P0_24, PINSEL_FUNC_1}, /* AD0[1] is on P0.24 second alternative function */
    { P0_25, PINSEL_FUNC_1}, /* AD0[2] is on P0.25 second alternative function */
    { P0_26, PINSEL_FUNC_1}, /* AD0[3] is on P0.26 second alternative function */
    { P1_30, PINSEL_FUNC_3}, /* AD0[4] is on P1.30 third alternative function */
    { P1_31, PINSEL_FUNC_3}, /* AD0[5] is on P1.31 third alternative function */
    { P0_3,  PINSEL_FUNC_2}, /* AD0[6] is on P0.3  third alternative function */
    { P0_2,  PINSEL_FUNC_2}  /* AD0[7] is on P0.2  third alternative function */
};


// Module initialisation
void AnalogInInit()
{
    ADC_Init();
    LPC_ADC->ADINTEN = 0x00000000; //Disable interupts for ADC done for each channel, and more importantly, ensure ADGINTEN (bit 8) is 0 for BURST Mode
}

// Enable or disable a channel.
void AnalogInEnableChannel(AnalogChannelNumber channel, bool enable)
{
	if (channel != NO_ADC && (unsigned int)channel < numChannels)
	{
		if (enable == true)
		{
			activeChannels |= (0x01 << channel);
            //set the pin mode for ADC
            GPIO_PinFunction(AdcConfig[channel].pinNumber,AdcConfig[channel].PinFunSel);
            //set the channels to sample (bits 0-7 of ADCR)
            LPC_ADC->ADCR  = (LPC_ADC->ADCR  & 0xFFFFFF00) | (activeChannels & 0x000000FF );
		}
		else
		{
			activeChannels &= ~(1u << channel);
            LPC_ADC->ADCR  = (LPC_ADC->ADCR  & 0xFFFFFF00) | (activeChannels & 0x000000FF );
            
		}
	}
}


// Read the most recent 12-bit result from a channel
uint16_t AnalogInReadChannel(AnalogChannelNumber channel)
{
    
	if ((unsigned int)channel < numChannels)
	{
        uint32_t val = 0;
        switch(channel){
            case 0: val = LPC_ADC->ADDR0; break;
            case 1: val = LPC_ADC->ADDR1; break;
            case 2: val = LPC_ADC->ADDR2; break;
            case 3: val = LPC_ADC->ADDR3; break;
            case 4: val = LPC_ADC->ADDR4; break;
            case 5: val = LPC_ADC->ADDR5; break;
            case 6: val = LPC_ADC->ADDR6; break;
            case 7: val = LPC_ADC->ADDR7; break;
            default:
                break;
        }
    
        return (val >> 4) & 0xFFF; //shift result into place and mask out other bits
    }
	return 0;
}


// Start converting the enabled channels
void AnalogInStartConversion(uint32_t channels)
{
    if(util_GetBitStatus(LPC_ADC->ADCR, SBIT_BURST) == 0 ){ // conversion wont start until BURST is set
        //Enable Burst, this will continiously sample all enabled channels until this is set to 0
        //Latest result for each channel is available in ADDRx
        util_BitSet(LPC_ADC->ADCR,SBIT_BURST);
    }
}


// Convert an  pin number to the corresponding ADC channel number
AnalogChannelNumber PinToAdcChannel(uint32_t pin)
{
    return g_APinDescription[pin].ulADCChannelNumber;
}


// End
