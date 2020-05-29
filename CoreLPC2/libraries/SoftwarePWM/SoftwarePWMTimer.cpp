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

//SD:: Code adapted from mbed us_ticker_api.c merged into class for Software PWM Timer

#include "SoftwarePWMTimer.h"
#include "SoftwarePWM.h"


SoftwarePWMTimer softwarePWMTimer;

SoftwarePWMTimer::SoftwarePWMTimer() noexcept
{
    head = nullptr;

    //Setup RIT
    Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_RIT); //enable power and clocking
    LPC_RITIMER->MASK = 0;
    LPC_RITIMER->COUNTER = 0;
    LPC_RITIMER->CTRL = RIT_CTRL_INT | RIT_CTRL_ENBR | RIT_CTRL_TEN;
    ticks_per_us = Chip_Clock_GetPeripheralClockRate(SYSCTL_PCLK_RIT)/1000000;
}

inline void SoftwarePWMTimer::ticker_set_interrupt(ticker_event_t *obj, bool inInterrupt) noexcept
{
    const irqflags_t flags = cpu_irq_save();

    
    LPC_RITIMER->CTRL &= ~RIT_CTRL_TEN; //Stop the timer
        
    //Check to make sure the timestamp is in the future
    if((int)(obj->timestamp - TickerRead()) <= 0)
    {
#ifdef LPC_DEBUG
        obj->PWM->IncrementLateCount(); // scheduled late
#endif
        if(inInterrupt)
        {
            //called from the ticker interrupt, schedule a little later to allow lower priority ints to run
            obj->timestamp = TickerRead() + TicksPerMicrosecond();
        }
        else
        {
            obj->timestamp = TickerRead() + 1; //next timer tick
        }
    }

    ticker_clear_interrupt();
    LPC_RITIMER->COMPVAL = obj->timestamp; //Set timer compare value
    NVIC_EnableIRQ(RITIMER_IRQn);

    LPC_RITIMER->CTRL |= RIT_CTRL_TEN; //Enable the timer
    
    cpu_irq_restore(flags);
}

inline void SoftwarePWMTimer::ticker_disable_interrupt(void) noexcept
{
    NVIC_DisableIRQ(RITIMER_IRQn);
}

inline void SoftwarePWMTimer::ticker_clear_interrupt(void) noexcept
{
    LPC_RITIMER->CTRL |= RIT_CTRL_INT; // Clear Interrupt
}

extern "C" void RIT_IRQHandler(void)  noexcept __attribute__ ((hot));
void RIT_IRQHandler(void) noexcept
{
    softwarePWMTimer.Interrupt();
}

void SoftwarePWMTimer::Interrupt() noexcept
{
    
    ticker_clear_interrupt();

    /* Go through all the pending TimerEvents */
    while (1)
    {
        if (head == NULL)
        {
            // There are no more TimerEvents left, so disable matches.
            ticker_disable_interrupt();
            return;
        }

        if ((int)(head->timestamp - TickerRead()) <= 0)
        {
            // This event was in the past:
            //      point to the following one and execute its handler
            ticker_event_t *p = head;
            head = head->next;

            p->PWM->Interrupt(); // NOTE: the handler can set new events
        }
        else
        {
            // This event and the following ones in the list are in the future:
            //      set it as next interrupt and return
            ticker_set_interrupt(head, true);
            return;
        }
    }
}



void SoftwarePWMTimer::ScheduleEventInMicroseconds(ticker_event_t *obj, uint32_t microseconds, SoftwarePWM *softPWMObject) noexcept
{
    //convert microseconds into timer ticks
    uint32_t timestamp = TickerRead() + microseconds * TicksPerMicrosecond();
    ticker_insert_event(obj, timestamp, softPWMObject);
}


void SoftwarePWMTimer::ticker_insert_event(ticker_event_t *obj, uint32_t timestamp, SoftwarePWM *softPWMObject) noexcept
{
    /* disable interrupts for the duration of the function */
    const irqflags_t flags = cpu_irq_save();

    // initialise our data
    obj->timestamp = timestamp;
    obj->PWM = softPWMObject;

    /* Go through the list until we either reach the end, or find
       an element this should come before (which is possibly the
       head). */
    ticker_event_t *prev = NULL, *p = head;
    while (p != NULL)
    {
        /* check if we come before p */
        if ((int)(timestamp - p->timestamp) <= 0)
        {
            break;
        }
        /* go to the next element */
        prev = p;
        p = p->next;
    }
    /* if prev is NULL we're at the head */
    if (prev == NULL)
    {
        head = obj;
        ticker_set_interrupt(obj);
    }
    else
    {
        prev->next = obj;
    }
    /* if we're at the end p will be NULL, which is correct */
    obj->next = p;

    cpu_irq_restore(flags);
    
}

void SoftwarePWMTimer::RemoveEvent(ticker_event_t *obj) noexcept
{
    const irqflags_t flags = cpu_irq_save();

    // remove this object from the list
    if (head == obj)
    {
        // first in the list, so just drop me
        head = obj->next;
        if (obj->next != NULL)
        {
            ticker_set_interrupt(head);
        }
    }
    else
    {
        // find the object before me, then drop me
        ticker_event_t* p = head;
        while (p != NULL)
        {
            if (p->next == obj)
            {
                p->next = obj->next;
                break;
            }
            p = p->next;
        }
    }

    cpu_irq_restore(flags);
}
