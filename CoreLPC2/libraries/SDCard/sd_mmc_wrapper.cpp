//SD: Wrapper to work with RRF

//Wraps RRF "Slot 0" to SDCard on SSP1 (internal SDCard on smoothie)
//wraps RRF "Slot 1" to SDCard on SSP0


#include "sd_mmc_wrapper.h"
#include "SDCard.h"


SDCard *_ffs[_DRIVES]; //also used by FatFS

//writeProtect pins and ChipSelect Pins for the SDCards
void sd_mmc_init(Pin const wpPins[_DRIVES],Pin const spiCsPins[_DRIVES]){

    if(spiCsPins != nullptr)
    {
        _ffs[0] = new SDCard(1, spiCsPins[0]);//RRF Slot0 = internal card on SSP1
        _ffs[1] = new SDCard(0, spiCsPins[1]);//RRF Slot1 = External card on SSP0
    }
}

//SD:: new function to allow to reinit to support flexible configuration
void sd_mmc_reinit_slot(uint8_t slot, Pin csPin, uint32_t spiFrequency)
{
    if(slot < _DRIVES)
    {
        _ffs[slot]->ReInit(csPin, spiFrequency);
    }
}


void sd_mmc_unmount(uint8_t slot)
{

}


sd_mmc_err_t sd_mmc_check(uint8_t slot){
    
    if(slot < _DRIVES && _ffs[slot]->disk_initialize() == 0)
    {
        return SD_MMC_OK;
    }
    else
    {
        return SD_MMC_ERR_UNUSABLE;
    }
}


uint32_t sd_mmc_get_capacity(uint8_t slot){
    
    if(slot < _DRIVES)
    {
        uint64_t s = _ffs[slot]->disk_sectors();
        uint32_t b = _ffs[slot]->disk_blocksize();
        
        s = (s/1024)*b;
        
        return (uint32_t) s; //return in kB
    }
    return 0;
}


card_type_t sd_mmc_get_type(uint8_t slot)
{
    return CARD_TYPE_SD;
}

uint32_t sd_mmc_get_interface_speed(uint8_t slot)
{
    if(slot < _DRIVES)
    {
    // Get the speed of the SPI SD card interface for reporting purposes, in bytes/sec
        return _ffs[slot]->interface_speed()/8;
    }
    
    return 0;
}

