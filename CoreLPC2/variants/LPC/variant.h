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

#ifndef _VARIANT_LPC
#define _VARIANT_LPC

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "Core.h"
#include "gpio.h"

#ifdef __GNUC__  /* GCC CS3 */
#    include "syscalls.h" /** RedHat Newlib minimal stub */
#endif


#ifdef __cplusplus


//i2c for DigitalPots
constexpr Pin SDA = P0_0;
constexpr Pin SCL = P0_1;

void ConfigurePin(const PinDescription& pinDesc);

#endif //end __cplusplus

#endif /* _VARIANT_LPC */

