#include "Hall_Homing.h"

bool Homing_enable = false;

  void IRAM_ATTR HallDetect() {
    //
    #if HALL_DEBUG_ON
        Serial.print("Hall State ");Serial.println(digitalRead(HALL_PIN));
    #endif
    if (digitalRead(HALL_PIN) == true){

    }
}

static void Homing_sequence(void *pvParameter)
{

    while (1) {
        if(Homing_enable){}
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void hall_init(void){

    pinMode(HALL_PIN, INPUT_PULLUP );
    attachInterrupt(HALL_PIN, HallDetect, CHANGE);

    xTaskCreatePinnedToCore(&Homing_sequence, "touch_pad_read_task", 2048, NULL, 5, NULL,0);
}
