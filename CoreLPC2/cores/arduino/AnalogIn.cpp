/*
 * AnalogInput.cpp
 *
 *  Created on: 2 Apr 2016
 *      Author: David
 
 * SD: Modified to run on LPC
 *   : - Uses the "Burst" mode of the LPC to continously sample each of the selected ADC channels once enabled
 *   : - Resolution is 12bit
 */
#include "AnalogIn.h"
#include "chip.h"

static ADC_CLOCK_SETUP_T ADCSetup;
const unsigned int numChannels = 8; //8 channels on LPC1768
static uint32_t activeChannels = 0;

typedef struct
{
    gpioPins_et pinNumber;
    uint8_t PinFunSel;
} adcChannelConfig_st;

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
    Chip_ADC_Init(LPC_ADC, &ADCSetup);
    Chip_ADC_SetBurstCmd(LPC_ADC, ENABLE);
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
            //set the channels to sample (bits 0-7 of CR)
            LPC_ADC->CR  = (LPC_ADC->CR  & 0xFFFFFF00) | (activeChannels & 0x000000FF );
		}
		else
		{
			activeChannels &= ~(1u << channel);
            LPC_ADC->CR  = (LPC_ADC->CR  & 0xFFFFFF00) | (activeChannels & 0x000000FF );
            
		}
	}
}


// Read the most recent 12-bit result from a channel
uint16_t AnalogInReadChannel(AnalogChannelNumber channel)
{
    uint16_t val = 0;
    Chip_ADC_ReadValue(LPC_ADC, (uint8_t)channel, &val);
    return val;
}


// Start converting the enabled channels
void AnalogInStartConversion(uint32_t channels)
{
}


// Convert an  pin number to the corresponding ADC channel number
AnalogChannelNumber PinToAdcChannel(uint32_t pin)
{
    return g_APinDescription[pin].ulADCChannelNumber;
}


// End
