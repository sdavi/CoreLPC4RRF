//Author: sdavi

#include "AnalogOut.h"
#include "SoftwarePWM.h"

extern "C" void debugPrintf(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));


SoftwarePWM* softwarePWMEntries[MaxNumberSoftwarePWMPins] = {};
static uint32_t pinsOnSoftPWM[5] = {0}; //5 ports;


bool CanDoSoftwarePWM(Pin pin) noexcept
{
    (void)pin;
    
    //SoftwarePWM can be on any pin, the only restriction if the Max number we allow
    uint8_t count = 0;
    for(size_t i=0; i<MaxNumberSoftwarePWMPins; i++)
    {
        if(softwarePWMEntries[i] == nullptr) count++;
    }

    return (count <= MaxNumberSoftwarePWMPins);
    
}

bool ConfigurePinForSoftwarePWM(Pin pin) noexcept
{
    const uint8_t port = (pin >> 5);
    const uint32_t portPinPosition = 1 << (pin & 0x1f);

    
    if( (pinsOnSoftPWM[port] & portPinPosition) )
    {
        //Pin is already configured as a SoftwarePWM ;
        return true;
    }

    //Find a free Slot
    for(size_t i=0; i<MaxNumberSoftwarePWMPins; i++)
    {
        if(softwarePWMEntries[i] == nullptr)
        {
            softwarePWMEntries[i] = new SoftwarePWM(pin);
            pinsOnSoftPWM[port] |= (portPinPosition);
            //debugPrintf("[AnalogOutSoftwarePWM] ConfigurePinForSoftwarePWM %d.%d\n", (pin >> 5), (pin & 0x1f));
            return true;
        }
    }

    return false;
}

void ReleaseSoftwarePWMPin(Pin pin) noexcept
{
    const uint8_t port = (pin >> 5);
    const uint32_t portPinPosition = 1 << (pin & 0x1f);

    if( !(pinsOnSoftPWM[port] & portPinPosition))
    {
        return;// pin not condifigured as a Software PWM
    }

    //find the pin
    for(size_t i=0; i<MaxNumberSoftwarePWMPins; i++)
    {
        if(softwarePWMEntries[i] != nullptr && softwarePWMEntries[i]->GetPin() == pin)
        {
            //remove PWM for this PIN as frequency is zero
            softwarePWMEntries[i]->Disable();
            delete(softwarePWMEntries[i]);
            softwarePWMEntries[i] = nullptr;
            pinsOnSoftPWM[i] &= ~(portPinPosition);
            return;
        }
    }

    
}

bool AnalogWriteSoftwarePWM(float ulValue, uint16_t freq, Pin pin) noexcept
{
    //Is the pin configured as softwarePWM?
    
    //debugPrintf("[AnalogOutSoftwarePWM] Write -  %d.%d %f %d\n", (pin >> 5), (pin & 0x1f), ulValue, freq);

    const uint8_t port = (pin >> 5);
    const uint32_t portPinPosition = 1 << (pin & 0x1f);
    
    if( !(pinsOnSoftPWM[port] & portPinPosition))
    {
        return false;// pin not condifigured as a Software PWM
    }
    
    
    //already configured to be softwarePWM
    int slot = -1;

    //find the pin
    for(size_t i=0; i<MaxNumberSoftwarePWMPins; i++)
    {
        if(softwarePWMEntries[i] != nullptr && softwarePWMEntries[i]->GetPin() == pin)
        {
            slot = i;
            break;
        }
    }
    
    if(slot == -1)
    {
        pinsOnSoftPWM[port] &= ~(portPinPosition);
        return false;
    }
    
    if(softwarePWMEntries[slot]->GetFrequency() != freq)
    {
        //Frequency has changed
        softwarePWMEntries[slot]->Disable();
        softwarePWMEntries[slot]->SetFrequency(freq);
        softwarePWMEntries[slot]->SetDutyCycle(ulValue);
        softwarePWMEntries[slot]->Enable();
    }
    else
    {
        //Frequency unchanged, update Duty Cycle
        softwarePWMEntries[slot]->SetDutyCycle(ulValue);
    }
    
    if( !softwarePWMEntries[slot]->IsRunning() )
    {
        softwarePWMEntries[slot]->Enable(); // enable if not running
    }
    else
    {
        softwarePWMEntries[slot]->Check(); // check the PWM is still functioning
    }
    
    return true;
}



