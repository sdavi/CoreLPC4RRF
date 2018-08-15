/*
  Copyright (c) 2014 Arduino.  All right reserved.

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

#include <stdlib.h>
#include <FreeRTOS.h>


//extern uint8_t ucHeap[configTOTAL_HEAP_SIZE];

//Heap5... only dynamic memory allocated in AHBRAM is done via FreeRTOS HEAP
#define AHB_RAM_START 0x2007C000


//SD:: This assumed Heap4.c is used in FreeRTOS
//SD:: Add a backup to new to allocate memory in RTOS heap if insufficient memory in main SRAM



void *operator new(size_t size)
{
    void *ptr = malloc(size);
    
    if( ptr == nullptr )
    {
        //failed to allocate in main RAM, backup option, lets try AHB Ram
        ptr = pvPortMalloc(size);
    }
    
    return ptr;
}

void *operator new[](size_t size)
{
    void *ptr = malloc(size);
    
    if( ptr == nullptr )
    {
        //failed to allocate in main RAM, backup option, lets try AHB Ram
        ptr = pvPortMalloc(size);
    }
    
    return ptr;
}



void operator delete(void * ptr)
{

    //check the memory address to see if its within ucHeap array to know if we are using free or pvPortFree
    if( (uint32_t) ptr >= AHB_RAM_START && (uint32_t) ptr < AHB_RAM_START+(32*1024) ){
        vPortFree(ptr);
    } else {
        free(ptr);
    }
    
}

void operator delete(void *ptr , std::size_t)
{
    //check the memory address to see if its within ucHeap array to know if we are using free or pvPortFree
    if( (uint32_t) ptr >= AHB_RAM_START && (uint32_t) ptr < AHB_RAM_START+(32*1024) ){
        vPortFree(ptr);
    } else {
        free(ptr);
    }
}

void operator delete[](void * ptr)
{
    //check the memory address to see if its within ucHeap array to know if we are using free or pvPortFree
    if( (uint32_t) ptr >= AHB_RAM_START && (uint32_t) ptr < AHB_RAM_START+(32*1024) ){
        vPortFree(ptr);
    } else {
        free(ptr);
    }
}




