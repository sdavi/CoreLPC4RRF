
#ifndef WDT_H
#define WDT_H


#ifdef __cplusplus
extern "C" {
#endif

#define WDT 1
    
    
void wdt_init(uint32_t s_counter);
//void wdt_restart();
void wdt_restart(uint8_t wdt); //compat with RRF
    
#ifdef __cplusplus
}
#endif

#endif /* WDT_H */
