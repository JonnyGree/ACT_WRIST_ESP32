#ifndef tmp32_h
#define tmp32

#include "HardwareConfig.h"
#include "Parameters.h"
    
    #define TMP32_DEBUG (0)

    
    // Update Temperature when exceed deadband (Parameter). 
    // Used for regulate the fan. Store data in C°
    extern float  LastTemperature;             // Degree"

    // Function   
    void   tmp32_init();
    void   ReadTemperature();
    void   SetFan(uint32_t  _FanSpeed);

    void tmp32_read( void * parameters );
     
#endif
