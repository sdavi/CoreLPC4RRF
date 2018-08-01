/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2011 LeafLabs, LLC.
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
 * @file   wirish/boards/maple_mini/include/board/board.h
 * @author Marti Bolivar <mbolivar@leaflabs.com>
 * @brief  Explore M3 board header.
 *
 * See wirish/boards/maple/include/board/board.h for more information
 * on these definitions.
 */
 
 #ifdef __cplusplus
extern "C" {
#endif

#ifndef _BOARD_EXPLORE_M3_H_
#define _BOARD_EXPLORE_M3_H_

#include "stdutils.h"
#include "gpio.h"

#define BOARD_MAX_GPIO_PINS  42
#define C_MaxAdcChannels_U8  8u


#define EXPLORE_M3_RESET_STRING "r$T^^3"
extern const uint8_t PIN_MAP[BOARD_MAX_GPIO_PINS];


//rearm doesnt have LEDS1-4
#if defined(__SMOOTHIEBOARD__) || defined(__AZTEEGX5MINI__)
#define LED1 P1_18
#define LED2 P1_19
#define LED3 P1_20
#define LED4 P1_21
#endif
    
//Smoothieboard doesnt have the Play LED installed, so lets use LED1.
#if defined(__SMOOTHIEBOARD__)
# define LED_PLAY LED1
#else
# define LED_PLAY P4_28
#endif
    

/************************************
             ADC pins
************************************/             
static unsigned const int A0 = P0_23;
static unsigned const int A1 = P0_24;
static unsigned const int A2 = P0_25;
static unsigned const int A3 = P0_26;
static unsigned const int A4 = P1_30;
static unsigned const int A5 = P1_31;
static unsigned const int A6 = P0_3;
static unsigned const int A7 = P0_2;
    
    
    
    
#define ANALOG_ZERO  A0
#define ANALOG_MAX   A7
/**********End of ADC pins********/





//External SPI
#define SPI0_MOSI  P0_18
#define SPI0_MISO  P0_17
#define SPI0_SCK   P0_15
//#define SPI0_SSEL  P0_16
    
    
//Internal SDCard
#define SPI1_MOSI  P0_9
#define SPI1_MISO  P0_8
#define SPI1_SCK   P0_7
//#define SPI1_SSEL  P0_6

    

#endif


#ifdef __cplusplus
}
#endif
