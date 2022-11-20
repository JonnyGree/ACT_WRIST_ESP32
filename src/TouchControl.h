/*
  Created by A. Sommacal, April 26, 2020.
  Released into the public domain.
*/
#ifndef TouchControl_h
#define TouchControl_h

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "driver/touch_pad.h"
#include "soc/rtc_periph.h"
#include "C:\Users\somma\.platformio\packages\framework-arduinoespressif32\tools\sdk\esp32c3\include\soc\include\soc\sens_periph.h"

#include "Parameters.h"
#include "StepperControl.h"


#define TOUCH_FIRST_PAD   (2)
#define TOUCH_THRESH_NO_USE   (0)
#define TOUCH_THRESH_PERCENT  (75)
#define TOUCHPAD_FILTER_TOUCH_PERIOD (5)
#define TOUCH_FILTER_MODE_EN  (0)

#define TOUCH_DEBUG_ON  (1)

#define TOUCH_STEP_INC  (5)

    void tp_init(void);
    extern int tp_speed;

#endif