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

#ifndef ANALOGOUT_H
#define ANALOGOUT_H

// Initialise this module
void AnalogOutInit();

/*
 * \brief Writes an analog value (PWM wave) to a pin.
 *
 * \param ulPin
 * \param ulValue, will be constrained to be within 0.0 to 1.0 within this function
 * \param freq (optional)
 */

struct LPCPWMInfo
{
    uint16_t hwPWMFreq;
    uint16_t tim1Freq;
    uint16_t tim2Freq;
    uint16_t tim3Freq;
};

void AnalogOut(Pin pin, float ulValue, uint16_t freq = 1000);
void GetTimerInfo( LPCPWMInfo *pwmInfo );
void ConfigureTimerForPWM(uint8_t timerChannel, uint16_t frequency);
bool IsPwmCapable(Pin pin);
bool IsServoCapable(Pin pin);
bool ConfigurePinForPWM(Pin pin, uint16_t frequency);



//Timer PWM
#define TimerPWM_Slot1 (0x01)
#define TimerPWM_Slot2 (0x02)
#define TimerPWM_Slot3 (0x04)


static const size_t MaxTimerEntries = 3; //MR0 for the Frequency and MR1-3 for the Timer PWM
static const uint8_t NumPwmChannels = 6;


extern uint32_t pinsOnATimer[5]; // 5 Ports
extern Pin Timer1PWMPins[MaxTimerEntries];
extern Pin Timer2PWMPins[MaxTimerEntries];
extern Pin Timer3PWMPins[MaxTimerEntries];
extern Pin UsedHardwarePWMChannel[NumPwmChannels];
extern uint16_t HardwarePWMFrequency;



#endif // ANALOGOUT_H
