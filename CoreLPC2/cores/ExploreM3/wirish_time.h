/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

/**
 * @file wirish/include/wirish/wirish_time.h
 * @brief Timing and delay functions.
 */

#ifndef _WIRISH_WIRISH_TIME_H_
#define _WIRISH_WIRISH_TIME_H_

#include "stdutils.h"
#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif


/**
 * Delay for at least the given number of milliseconds.
 *
 * Interrupts, etc. may cause the actual number of milliseconds to
 * exceed ms.  However, this function will return no less than ms
 * milliseconds from the time it is called.
 *
 * @param ms the number of milliseconds to delay.
 * @see delayMicroseconds()
 */
// This has been renamed from delay to coreDelay so that RTOS-based applications can use a different definition of delay()

void coreDelay(uint32_t ms) noexcept;

/**
 * Delay for at least the given number of microseconds.
 *
 * Interrupts, etc. may cause the actual number of microseconds to
 * exceed us.  However, this function will return no less than us
 * microseconds from the time it is called.
 *
 * @param us the number of microseconds to delay.
 * @see delay()
 */
void delayMicroseconds(uint32_t us) noexcept;

#ifdef __cplusplus
}
#endif


#endif
