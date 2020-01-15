/*
* ADCPreFilter.cpp
*
*  Created on: 15 Jan 2020
*      Author: sdavi
*/
 
#include "ADCPreFilter.h"

#include <algorithm>

ADCPreFilter::ADCPreFilter(const uint8_t numChannels, const uint8_t numSamples): samplesArray(nullptr), numberSamples(numSamples), numberChannels(numChannels)
{
    //create the array
    samplesArray = new uint16_t*[numberChannels];
    for(uint8_t i = 0; i < numberChannels; ++i)
    {
        samplesArray[i] = new uint16_t[numberSamples];
        for(int s=0; s<numberSamples; s++)
        {
            samplesArray[i][s] = 0;
        }
    }
    median_buffer = new uint16_t[numberSamples];
}

void ADCPreFilter::appendADCSampleFromISR(uint8_t channel, uint16_t value)
{
    //Function from Smoothieware adc.cpp
    // Shuffle down and add new value to the end
    if(channel < numberChannels)
    {
        memmove(&samplesArray[channel][0], &samplesArray[channel][1], sizeof(uint16_t)*numberSamples - sizeof(uint16_t));
        samplesArray[channel][numberSamples - 1] = value; // the 12 bit ADC reading
    }
}

//Get the latest filtered value for the channel
uint16_t ADCPreFilter::Read(uint8_t channel)
{
    //Function from Smoothieware adc.cpp
    NVIC_DisableIRQ(ADC_IRQn);
    memcpy(median_buffer, samplesArray[channel], sizeof(uint16_t)*numberSamples);
    NVIC_EnableIRQ(ADC_IRQn);

    // sort the 8 readings and return the average of the middle 4
    std::sort(median_buffer, median_buffer + numberSamples);
    int sum = 0;
    for (int i = numberSamples / 4; i < (numberSamples - (numberSamples / 4)); ++i)
    {
        sum += median_buffer[i];
    }
    return sum / (numberSamples / 2);
}

// End
