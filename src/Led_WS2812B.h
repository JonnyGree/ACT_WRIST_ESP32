/*
  Created by A. Sommacal, April 26, 2020.
  Released into the public domain.
*/
#ifndef Led_WS2812B_h
#define Led_WS2812B_h

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "Parameters.h"
#include <FastLED.h>


    #define LED_DEBUG_ON  (0)    
    #define LED_FLASH_DELAYVAL 300  // Time (in milliseconds) to pause between pixels
    #define LED_TASK_DELAYVAL 50  // Time (in milliseconds) to pause between pixels    


    void led_init(void);

#endif