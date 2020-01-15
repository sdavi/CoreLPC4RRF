/*
 * ADCPreFilter.h
 *
 *  Created on: 15 Jan 2020
 *      Author: sdavi
 */

#ifndef ADCPREFILTER_H_
#define ADCPREFILTER_H_

#ifdef __cplusplus

#include "Core.h"

class ADCPreFilter
{
    
public:
    ADCPreFilter(uint8_t numChannels, uint8_t numSamples);
    void appendADCSampleFromISR(uint8_t channel, uint16_t value);
    uint16_t Read(uint8_t channel);
    
private:
    uint16_t **samplesArray; //will be populated as [numChannels][numSamples]
    uint8_t numberSamples;
    uint8_t numberChannels;
    uint16_t *median_buffer;
    
    
};


#endif //end __cplusplus

#endif /* ADCPREFILTER_H_ */
