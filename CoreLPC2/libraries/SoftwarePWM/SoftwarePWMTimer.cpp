//Author: sdavi

//This class uses the HardwarePWM as a general timer to generate software PWM for up to 7 channels
//This allows using any available GPIO pin to generate PWM and each pin can be running at different frequencies

#include "AnalogOut.h"
#include "SoftwarePWMTimer.h"
#include "SoftwarePWM.h"
#include "chip.h"
#include "pwm_176x.h"

static_assert( (MaxNumberSoftwarePWMPins < 8), "Max number of software PWM using HWPWM Timer is 7");

SoftwarePWMTimer softwarePWMTimer;

volatile uint32_t* const MRxMap[] = {&LPC_PWM1->MR0, &LPC_PWM1->MR1, &LPC_PWM1->MR2, &LPC_PWM1->MR3, &LPC_PWM1->MR4, &LPC_PWM1->MR5, &LPC_PWM1->MR6};

SoftwarePWMTimer::SoftwarePWMTimer() noexcept
{
    Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_PWM1); //enable power and clocking
    LPC_PWM1->TCR = PWM_TCR_RESET; //set reset
    LPC_PWM1->PR = Chip_Clock_GetPeripheralClockRate(SYSCTL_PCLK_PWM1)/1000000 - 1; //Setup prescaler so every timer tick is 1us
    LPC_PWM1->PCR = 0; //ensure all pwm outputs are disabled
    LPC_PWM1->MCR = 0; //clear all match controls
    LPC_PWM1->CTCR = 0; //Timer Mode
    LPC_PWM1->TCR = PWM_TCR_CNT_EN; // Enable the counter but keep PWM mode disabled
    
    NVIC_EnableIRQ(PWM1_IRQn);
}

__attribute__((always_inline)) static inline void UpdateChannel(SoftwarePWM * const sChannel, const uint8_t channelIndex) noexcept
{
    uint32_t nextEvent;
    if(sChannel->updateState(&nextEvent))
    {
        //schedule next event
        const irqflags_t flags = cpu_irq_save();//make sure we dont get interrupted while setting the next event
        *MRxMap[channelIndex] = LPC_PWM1->TC + nextEvent;
        cpu_irq_restore(flags);
    }
    else
    {
        //disable int for this channel
        LPC_PWM1->MCR &= ~(1u << (channelIndex*3) );
    }
}

void SoftwarePWMTimer::EnableChannel(SoftwarePWM *sChannel) noexcept
{
    for(uint8_t i=0; i<MaxNumberSoftwarePWMPins; i++)
    {
        if(softwarePWMEntries[i] == sChannel)
        {
            LPC_PWM1->MCR |= (1u << (i*3) ); //enable the interrupt
            UpdateChannel(sChannel, i);
            return;
        }
    }
}

void SoftwarePWMTimer::DisableChannel(SoftwarePWM *sChannel) noexcept
{
    for(uint8_t i=0; i<MaxNumberSoftwarePWMPins; i++)
    {
        if(softwarePWMEntries[i] == sChannel)
        {
            LPC_PWM1->MCR &= ~(1u << (i*3) ); //disable the interrupt
            return;
        }
    }
}

extern "C" void PWM1_IRQHandler(void) noexcept __attribute__ ((hot));
void PWM1_IRQHandler(void) noexcept
{
    uint32_t flags = LPC_PWM1->IR;
    LPC_PWM1->IR = flags;   //clear interrupts that we are going to service

    flags = flags & 0x70F; // get only the MR0-7 interrupt flags MR0-3 (bits 0:4) MR4-6 (bits 8:10)
    
    while(flags != 0)
    {
        unsigned int indx = LowestSetBitNumber(flags);
        flags &= ~(1u << indx);

        if(indx > 4) indx = indx - 4; //index if MR4-6
        
        SoftwarePWM * const channel = softwarePWMEntries[indx];
        if(channel != nullptr)
        {
            UpdateChannel(channel, indx);
        }
    }
}


