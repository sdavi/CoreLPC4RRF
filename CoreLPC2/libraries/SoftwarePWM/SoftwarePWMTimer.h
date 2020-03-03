/* mbed Microcontroller Library
* Copyright (c) 2006-2013 ARM Limited
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

//SD:: Code from mbed us_ticker_api merged into class for Software PWM Timer

#ifndef SOFTWAREPWMTIMER_H
#define SOFTWAREPWMTIMER_H

#include "Core.h"
#include "chip.h"

class SoftwarePWM; //Fwd decl

typedef struct ticker_event_s
{
    uint32_t timestamp;
    SoftwarePWM *PWM;
    struct ticker_event_s *next;
} ticker_event_t;



class SoftwarePWMTimer
{
    
    
public:
    SoftwarePWMTimer();
    
    void us_ticker_insert_event(ticker_event_t *obj, unsigned int timestamp, SoftwarePWM *softPWMObject);
    void us_ticker_remove_event(ticker_event_t *obj);
    void Interrupt();

    uint32_t us_ticker_read();

private:
    void us_ticker_set_interrupt(ticker_event_t *obj);
    void us_ticker_disable_interrupt(void);
    void us_ticker_clear_interrupt(void);
    
    ticker_event_t *head;
};


extern SoftwarePWMTimer softwarePWMTimer;

#endif
