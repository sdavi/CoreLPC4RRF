
#ifndef WDT_H
#define WDT_H


#ifdef __cplusplus
extern "C" {
#endif

    
void wdt_init(uint32_t s_counter);
void wdt_restart();

#ifdef __cplusplus
}
#endif

#endif /* WDT_H */
