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

#ifndef _VARIANT_ARDUINO_DUE_X_
#define _VARIANT_ARDUINO_DUE_X_

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "Core.h"
#include "gpio.h"


/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/




/** Frequency of the board main oscillator */
//#define VARIANT_MAINOSC		12000000  // 12MHz

/** Master clock frequency */
//#define VARIANT_MCK			100000000
#define F_CPU   (SystemCoreClock)


#ifdef __cplusplus



#if defined (  __GNUC__  ) /* GCC CS3 */
#    include "syscalls.h" /** RedHat Newlib minimal stub */
#endif




void ConfigurePin(const PinDescription& pinDesc);
#endif


/**
 * Libc porting layers
 */


//i2c for DigitalPots
#define SDA P0_0
#define SCL P0_1

#define MaxPinNumber 160 // 5*32


#endif /* _VARIANT_ARDUINO_DUE_X_ */

