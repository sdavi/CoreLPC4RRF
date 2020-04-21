/*
 * ADCPreFilter.h
 *
 *  Created on: 15 Jan 2020
 *      Author: sdavi
 */

#ifndef ADCPREFILTER_H_
#define ADCPREFILTER_H_

#include "Core.h"

bool ADCPreFilterInit(const uint8_t numSamples, const uint32_t sampleRateHz) noexcept;
uint16_t ADCPreFilterRead(uint8_t channel) noexcept;

#endif /* ADCPREFILTER_H_ */
