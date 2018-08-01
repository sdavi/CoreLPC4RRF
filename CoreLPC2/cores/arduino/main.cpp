
#define ARDUINO_MAIN
#include "Core.h"
#include "gpio.h"

#include "lpc17xx.h"
#include "stdutils.h"

#include "USBDevice/USB.h"
#include "USBDevice/USBSerial/USBSerial.h"
#include "usb_serial.h"

#include <FreeRTOS.h>

extern "C" void UrgentInit();
extern "C" void __libc_init_array(void);
extern "C" void AppMain();



SerialUSB Serial; // our wrapper object to provide Serial over USB

#include <FreeRTOS.h>
#include "task.h"

#include "board.h"


//RTOS Heap (size set in FreeRTOS config header)
__attribute__ ((used,section("AHBSRAM0"))) uint8_t ucHeap[configTOTAL_HEAP_SIZE];




extern "C" void Board_Init(void);
int main( void )
{

    SystemInit();
    //SystemCoreClockUpdate(); // is called from system init
    
    //Board_Init();
    
    
    extern uint8_t __AHB0_block_start;
    extern uint8_t __AHB0_dyn_start;
    //extern uint8_t __AHB0_end;

    
    // zero the data sections in AHB0 SRAM
    memset(&__AHB0_block_start, 0, &__AHB0_dyn_start - &__AHB0_block_start);

    UrgentInit();

    __libc_init_array();
    init(); // Defined in variant.cpp

    
    SysTick_Init(); // also inits the LEDs for systick/vApplicationTickHook
   
//Dont start systick fo RTOS
    //SysTick_Start();
    //for (;;)
    // {
        //loop();
    //}
    
    
    AppMain();

    return 0;
}
