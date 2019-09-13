/*
 * AnalogInput.h
 *
 *  Created on: 2 Apr 2016
 *      Author: David
 */

#ifndef ANALOGIN_H_
#define ANALOGIN_H_

#ifdef __cplusplus

#include "Core.h"



// Module initialisation
void AnalogInInit();

// Enable or disable a channel. Use AnalogCheckReady to make sure the ADC is ready before calling this.
void AnalogInEnableChannel(AnalogChannelNumber channel, bool enable);

// Read the most recent 12-bit result from a channel
uint16_t AnalogInReadChannel(AnalogChannelNumber channel);
static inline void AnalogInFinaliseConversion() { }

 static constexpr unsigned int AdcBits = 12;

// Start converting the enabled channels, to include the specified ones. Disabled channels are ignored.
void AnalogInStartConversion(uint32_t channels = 0xFFFFFFFF);

// Convert a pin number to an AnalogIn channel
extern AnalogChannelNumber PinToAdcChannel(uint32_t pin);


#endif

#endif /* CORES_ARDUINO_ANALOGIN_H_ */
