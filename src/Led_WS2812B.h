/*
  Created by A. Sommacal, April 26, 2020.
  Released into the public domain.
*/
#ifndef Led_WS2812B_h
#define Led_WS2812B_h

#include "HardwareConfig.h"
#include "Parameters.h"

    #define LED_DEBUG  (0)    

    void led_init();
    void led_task(void *parameter);

#endif