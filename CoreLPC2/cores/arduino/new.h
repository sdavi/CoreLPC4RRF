#ifndef new_h
#define new_h

#include <FreeRTOS.h>

typedef uint8_t AHBType;
static const AHBType AHB0 = 0;

//new to use FreeRTOS Heap (located in AHB)
inline void* operator new(size_t nbytes, AHBType ahb)
{
    return pvPortMalloc(nbytes);
}


#endif
