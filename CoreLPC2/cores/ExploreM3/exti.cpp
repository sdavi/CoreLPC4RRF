/***************************************************************************************************
                                   ExploreEmbedded    
*****************************************************************************************************
 * File:   extintr.c
 * Version: 16.0
 * Author: ExploreEmbedded
 * Website: http://www.exploreembedded.com/wiki
 * Description: File contains the functions for configuring and using the LPC1768 External Interrupts.


The libraries have been tested on ExploreEmbedded development boards. We strongly believe that the 
library works on any of development boards for respective controllers. However, ExploreEmbedded 
disclaims any kind of hardware failure resulting out of usage of libraries, directly or indirectly.
Files may be subject to change without prior notice. The revision history contains the information 
related to updates. 


GNU GENERAL PUBLIC LICENSE: 
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>. 

Errors and omissions should be reported to codelibraries@exploreembedded.com
 
 SD: Modified to handle GPIO interrupts and support RRF
 
***************************************************************************************************/
#include "lpc17xx.h"
#include "exti.h"
#include "stdutils.h"
#include "gpio.h"
#include "board.h"
#include "core.h"

/**************************************************************************************************
                      EINTconfiguration table(Do not alter uncless required)
***************************************************************************************************/
eintConfig_t EintConfigTable[EINT_MAX] = 
{  /* userFunPtr  IRQ Name   EINT Pin */
        { NULL,      EINT0_IRQn,  P2_10, },
        { NULL,      EINT1_IRQn,  P2_11, },
        { NULL,      EINT2_IRQn,  P2_12, },
        { NULL,      EINT3_IRQn,  P2_13, },
};
/*************************************************************************************************/


typedef void (*interruptCB)(CallbackParameter);

struct InterruptCallback
{
    interruptCB func;
    void *param;
    InterruptCallback() : func(nullptr) { }

};


struct CallbackEntry{
    bool enabled;
    uint8_t portNumber;
    uint8_t portPinNumber;
    InterruptCallback intCallback;
};

constexpr uint8_t maxCallbacks = 3;

//support only 3 callbacks
static CallbackEntry callbacks[maxCallbacks] __attribute__((section ("AHBSRAM0")));


void InitialiseCallbacks()
{
    for(uint8_t i=0; i< maxCallbacks; i++)
    {
        callbacks[i].enabled = false;
    }
}


inline int8_t findEntry(uint8_t port, uint8_t pinNumber)
{
    for(uint8_t i=0; i< maxCallbacks; i++)
    {
        if(callbacks[i].portNumber == port && callbacks[i].portPinNumber == pinNumber )
        {
            return i;
        }
    }
    return -1; // not found
}

//takes 256 Bytes each
//static InterruptCallback callbacksGPIO0[32] __attribute__ ((section ("AHBSRAM0"))); // put callbacks array in AHB0
//static InterruptCallback callbacksGPIO2[32] __attribute__ ((section ("AHBSRAM0")));


//Function from WInterrupts from RRF.

// Get the number of the highest bit that is set in a 32-bit word. Returns 0 if no bit set (same as if lowest bit is set).
// This needs to be fast. Hopefully the ARM conditional instructions will be used to advantage here.
static unsigned int GetHighestBit(uint32_t bits)
{
    unsigned int bitNum = (bits >= 0x00010000) ? 16 : 0;
    if ((bits >> bitNum) >= 0x0100u)
    {
        bitNum += 8;
    }
    if ((bits >> bitNum) >= 0x0010u)
    {
        bitNum += 4;
    }
    if ((bits >> bitNum) >= 0x0004u)
    {
        bitNum += 2;
    }
    if ((bits >> bitNum) >= 0x0002u)
    {
        bitNum += 1;
    }
    return bitNum;
}

//SD: added new functions to handle Interrupt in on GPIO
bool attachInterrupt(uint32_t pin, void (*callback)(CallbackParameter), enum InterruptMode mode, void *param)
{
    static bool init_callbacks = false;
    if(init_callbacks == false)
    {
        InitialiseCallbacks();
        init_callbacks = true;
    }
    
    int8_t freeEntry = -1;
    for(uint8_t i=0; i< maxCallbacks; i++)
    {
        if(callbacks[i].enabled == false)
        {
            freeEntry = i;
            break;
        }
    }
    
    if( freeEntry == -1 ) return false;
    
    
    
    
    //Port 0 and Port 2 can provide a single interrupt for any combination of port pins.
    //GPIO INTS call EINT3 handler!!

    uint8_t portNumber;
    uint8_t var_pinNumber_u8;
    
    portNumber =  (pin>>5);  //Divide the pin number by 32 go get the PORT number
    var_pinNumber_u8  =   pin & 0x1f;  //lower 5-bits contains the bit number of a 32bit port


    // Set callback function and parameter

    
    //ensure pin is on Port 0 or Port 2
    //Each port pin can be programmed to generate an interrupt on a rising edge, a falling edge, or both.
    if(portNumber ==0 || portNumber == 2 )
    {
        
        const irqflags_t flags = cpu_irq_save();

     
        NVIC_EnableIRQ(EINT3_IRQn); // GPIO interrupts on P0 and P2 trigger EINT3 handler

//        if(portNumber == 0)
//        {
//            callbacksGPIO0[var_pinNumber_u8].func = callback;
//            callbacksGPIO0[var_pinNumber_u8].param = param;
//        }
//        else if(portNumber == 2)
//        {
//            callbacksGPIO2[var_pinNumber_u8].func = callback;
//            callbacksGPIO2[var_pinNumber_u8].param = param;
//        }

        
        callbacks[freeEntry].enabled = true;
        callbacks[freeEntry].portNumber = portNumber;
        callbacks[freeEntry].portPinNumber = var_pinNumber_u8;
        callbacks[freeEntry].intCallback.func = callback;
        callbacks[freeEntry].intCallback.param = param;


        
        
        
        switch(mode)
        {
            case INTERRUPT_MODE_LOW:
                //LOW level int not implemented in GPIO
                break;
                
            case INTERRUPT_MODE_HIGH:
                //HIGH level int not implemented in GPIO
                break;
                
            case INTERRUPT_MODE_FALLING:
                if(portNumber == 0){
                    util_BitSet(LPC_GPIOINT->IO0IntEnF, var_pinNumber_u8); // set Falling Interrupt bit for pin
                    util_BitClear(LPC_GPIOINT->IO0IntEnR, var_pinNumber_u8); //ensure Rising disabled
                }
                if(portNumber == 2){
                    util_BitSet(LPC_GPIOINT->IO2IntEnF, var_pinNumber_u8);
                    util_BitClear(LPC_GPIOINT->IO2IntEnR, var_pinNumber_u8); //ensure Rising disabled
                }

                break;
                
            case INTERRUPT_MODE_RISING:
                if(portNumber == 0) {
                    util_BitSet(LPC_GPIOINT->IO0IntEnR, var_pinNumber_u8);
                    util_BitClear(LPC_GPIOINT->IO0IntEnF, var_pinNumber_u8); // ensure Falling disabled
                }
                if(portNumber == 2) {
                    util_BitSet(LPC_GPIOINT->IO2IntEnR, var_pinNumber_u8);
                    util_BitClear(LPC_GPIOINT->IO2IntEnF, var_pinNumber_u8); // ensure Falling disabled
                }

                break;
            case INTERRUPT_MODE_CHANGE:
                //Rising and Falling
                if(portNumber == 0){
                    util_BitSet(LPC_GPIOINT->IO0IntEnF, var_pinNumber_u8); //Falling
                    util_BitSet(LPC_GPIOINT->IO0IntEnR, var_pinNumber_u8); //Rising
                }
                if(portNumber == 2){
                    util_BitSet(LPC_GPIOINT->IO2IntEnF, var_pinNumber_u8); //Falling
                    util_BitSet(LPC_GPIOINT->IO2IntEnR, var_pinNumber_u8); //Rising
                }


                break;
            default:
                break;
                
        }

        cpu_irq_restore(flags);
        

        
    } else {
        
        return false; // no interrupts avail on this pin
    }

    
    return true;
}


void detachInterrupt(uint32_t pin){
 
    uint8_t portNumber;
    uint8_t var_pinNumber_u8;
    
    portNumber =  (pin>>5);  //Divide the pin number by 32 go get the PORT number
    var_pinNumber_u8  =   pin & 0x1f;  //lower 5-bits contains the bit number of a 32bit port

    //clear Rise and Fall
    if(portNumber == 0){
        util_BitClear(LPC_GPIOINT->IO0IntEnF, var_pinNumber_u8); //Falling
        util_BitClear(LPC_GPIOINT->IO0IntEnR, var_pinNumber_u8); //Rising
    }
    if(portNumber == 2){
        util_BitClear(LPC_GPIOINT->IO2IntEnF, var_pinNumber_u8); //Falling
        util_BitClear(LPC_GPIOINT->IO2IntEnR, var_pinNumber_u8); //Rising
    }

    
    int8_t val = findEntry(portNumber, var_pinNumber_u8);
    if(val != -1) callbacks[val].enabled = false;
    
    
}

/** \brief  Get IPSR Register
 
 This function returns the content of the IPSR Register.
 
 \return               IPSR Register value
 */
__attribute__( ( always_inline ) ) static inline uint32_t __get_IPSR(void)
{
    uint32_t result;
    
    __ASM volatile ("MRS %0, ipsr" : "=r" (result) );
    return(result);
}


// Return true if we are in any interrupt service routine
bool inInterrupt()
{
    //return (__get_IPSR() & 0x01FF) != 0;
    
    //bits 0:8 are the ISR_NUMBER
    //bits 9:31 reserved
    
    return (__get_IPSR() & 0xFF) != 0;
}






/***************************************************************************************************
                            EINTx ISR's
*****************************************************************************************************
 desc: The four ISR's will be called independently whenever the interrupt is detected on EITx pins.

       The ISR will clear the Interrupt bit as it is being served.(Setting the bit will clear the Corresponding ISR Bit)
       If the user CallBack Function is configured then it will be called. 

*****************************************************************************************************/
extern "C" void EINT0_IRQHandler(void)
{
    util_BitSet(LPC_SC->EXTINT, EINT0);  /* Clear Interrupt Flag */
    if(EintConfigTable[EINT0].userFunction != NULL)
    {
        EintConfigTable[EINT0].userFunction();
    }
}


extern "C" void EINT1_IRQHandler(void)
{
    util_BitSet(LPC_SC->EXTINT, EINT1);  /* Clear Interrupt Flag */
    if(EintConfigTable[EINT1].userFunction != NULL)
    {
        EintConfigTable[EINT1].userFunction();
    }
}


extern "C" void EINT2_IRQHandler(void)
{
    util_BitSet(LPC_SC->EXTINT, EINT2);  /* Clear Interrupt Flag */
    if(EintConfigTable[EINT2].userFunction != NULL)
    {
        EintConfigTable[EINT2].userFunction();
    }
}


extern "C" void EINT3_IRQHandler(void)
{
    //We assume we arent also using EINT3 (external interrupt function), but only the GPIO interrupts which share the same interrupt
    
    //Look for Rising And Falling interrupt for both ports
    //Since we dont need to do anything different for rise/fall, we treat the same
    uint32_t isr0 = LPC_GPIOINT->IO0IntStatR | LPC_GPIOINT->IO0IntStatF; // get all pins rise and fall which triggered int
    uint32_t isr2 = LPC_GPIOINT->IO2IntStatR | LPC_GPIOINT->IO2IntStatF; // get all pins rise and fall which triggered int

    //port 0
    while (isr0 != 0)
    {
        const unsigned int pos0 = GetHighestBit(isr0);
        LPC_GPIOINT->IO0IntClr |= (1 << pos0); // clear the status
        
        int8_t ret = findEntry(0, pos0);
        if(ret != -1 && callbacks[ret].intCallback.func != NULL)
        {
            callbacks[ret].intCallback.func(callbacks[ret].intCallback.param);

        }
//        if (callbacksGPIO0[pos0].func != nullptr)
//        {
//            callbacksGPIO0[pos0].func(callbacksGPIO0[pos0].param);
//        }
        isr0 &= ~(1u << pos0);
    }
    //port 2
    while (isr2 != 0)
    {
        const unsigned int pos2 = GetHighestBit(isr2);
        LPC_GPIOINT->IO2IntClr |= (1 << pos2); // clear the status

        int8_t ret = findEntry(2, pos2);
        if(ret != -1 && callbacks[ret].intCallback.func != NULL)
        {
            callbacks[ret].intCallback.func(callbacks[ret].intCallback.param);
        }
//        if (callbacksGPIO2[pos2].func != nullptr)
//        {
//            callbacksGPIO2[pos2].func(callbacksGPIO2[pos2].param);
//        }
        isr0 &= ~(1u << pos2);
    }

    
    
}

/*************************************************************************************************
                                    END of  ISR's 
**************************************************************************************************/
