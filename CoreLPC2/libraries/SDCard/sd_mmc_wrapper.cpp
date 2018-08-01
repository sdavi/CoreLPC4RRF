//SD: Wrapper to work with RRF

//Wraps RRF "Slot 0" to SDCard on SSP1 (internal SDCard on smoothie)
//wraps RRF "Slot 1" to SDCard on SSP0


#include "sd_mmc_wrapper.h"
#include "SDCard.h"


SDCard *_ffs[_DRIVES];// __attribute__ ((section ("AHBSRAM0")));    //


//writeProtect pins and ChipSelect Pints for the SDCards
void sd_mmc_init(Pin const wpPins[],Pin const spiCsPins[]){
    
    //TODO:: this assumes there are 2 entries in the arrays
    
    _ffs[0] = new /*(AHB0)*/ SDCard(1, spiCsPins[0]);//RRF Slot0 = internal card on SSP1
    _ffs[1] = new /*(AHB0)*/ SDCard(0, spiCsPins[1]);//RRF Slot1 = External card on SSP0

}


void sd_mmc_unmount(uint8_t slot){
    //TODO:: do we need to do anything here ?
}


sd_mmc_err_t sd_mmc_check(uint8_t slot){
    
    if(_ffs[slot]->disk_initialize() == 0){
        return SD_MMC_OK;
    } else {
        return SD_MMC_ERR_UNUSABLE;
    }
}


uint32_t sd_mmc_get_capacity(uint8_t slot){
    
    uint64_t s = _ffs[slot]->disk_sectors();
    uint32_t b = _ffs[slot]->disk_blocksize();
    
    s = (s/1024)*b;
    
    return (uint32_t) s; //return in kB
}


card_type_t sd_mmc_get_type(uint8_t slot){
    return CARD_TYPE_SD;
}

uint32_t sd_mmc_get_interface_speed(uint8_t slot){
    //currently unsupported. TODO::
    return 0;
    
    
    
}

