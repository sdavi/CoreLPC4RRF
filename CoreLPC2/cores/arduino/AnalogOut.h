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
extern void AnalogOutInit();

/*
 * \brief Writes an analog value (PWM wave) to a pin.
 *
 * \param ulPin
 * \param ulValue, will be constrained to be within 0.0 to 1.0 within this function
 * \param freq (optional)
 */
extern void AnalogOut(Pin pin, float ulValue, uint16_t freq = 1000);
extern void GetTimerInfo( uint16_t freqs[4] );
extern void ConfigureTimerForPWM(uint8_t timerChannel, uint16_t frequency);
extern bool IsPwmCapable(Pin pin);
extern bool IsServoCapable(Pin pin);


//Timer PWM
#define TimerPWM_Slot1 (0x01)
#define TimerPWM_Slot2 (0x02)
#define TimerPWM_Slot3 (0x04)


static const size_t MaxTimerEntries = 3; //MR0 for the Frequency and MR1-3 for the Timer PWM

extern uint32_t pinsOnATimer[5]; // 5 Ports
extern Pin Timer1PWMPins[MaxTimerEntries];
extern Pin Timer2PWMPins[MaxTimerEntries];
extern Pin Timer3PWMPins[MaxTimerEntries];




#endif // ANALOGOUT_H
