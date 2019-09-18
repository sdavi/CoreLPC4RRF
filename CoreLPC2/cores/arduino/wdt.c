

//SD: Watchdog functions for LPC to work with RRF

#include "chip.h"
#include "wdt.h"
#include "stdutils.h"
#include "interrupt_lpc.h"


#ifdef __cplusplus
extern "C" {
#endif
/* LPC
 -Programmable 32-bit timer with internal pre-scaler.
 -Selectable time period from (TWDCLK x 256 x 4) to (TWDCLK x 2^32 x 4) in multiples of TWDCLK x 4.
 */

    
#define WDEN_SBIT 0
#define WDRESET_SBIT 1
    
    
    
void wdt_init(uint32_t s_counter){

    //From Manual
    //1. Set the Watchdog timer constant reload value in WDTC register.
    //2. Setup the Watchdog timer operating mode in WDMOD register.
    //3. Enable the Watchdog by writing 0xAA followed by 0x55 to the WDFEED register.
    //4. The Watchdog should be fed again before the Watchdog counter underflows to prevent reset/interrupt.
    
    LPC_WWDT->CLKSEL = 0x1;                // Set CLK src to PCLK (PCLK_WDT is default to CCLK/4)
    uint32_t clk = SystemCoreClock / 16;    // WDT has a fixed /4 prescaler, PCLK default is /4
    LPC_WWDT->TC = s_counter * clk;

    /*Manual says: The WDTC register determines the time-out value. Every time a feed sequence occurs
     the WDTC content is reloaded in to the Watchdog timer. It’s a 32-bit register with 8 LSB
     set to 1 on reset. Writing values below 0xFF will cause 0x0000 00FF to be loaded to the
     WDTC. Thus the minimum time-out interval is TWDCLK x 256 x 4.
     */
     
    LPC_WWDT->MOD = (1<<WDEN_SBIT);//Enable Watchdog 

    NVIC_EnableIRQ(WDT_IRQn);
    NVIC_SetPriority(WDT_IRQn, 1);

    wdt_restart(WDT); //feed the watchdog
}

void wdt_restart(uint8_t wdt) //compat with RRF wdt not used, but maintain compat with RRF
{
    //Feed the hungry hungry watchdog
    //0xAA followed by 0x55 to WDFEED reloads the Watchdog timer with the value contained in WDTC
    irqflags_t flags = cpu_irq_save();

    LPC_WWDT->FEED = 0xAA;
    LPC_WWDT->FEED = 0x55;
    cpu_irq_restore(flags);
}

    
#ifdef __cplusplus
}
#endif

