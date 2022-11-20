#ifndef tmp32_h
#define tmp32

#include "Parameters.h"

    #define TEMP_DEBUG_ON  (0)
    
    extern float TempAverage, MemTempC;
    extern byte TempHasChanged,FanSpeed;
    
    float ReadTemperature();

    void tmp32_init();
     
#endif
