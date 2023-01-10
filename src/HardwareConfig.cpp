#include <Arduino.h>
#include "HardwareConfig.h"

void initHardware(){


    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);
    pinMode(SLEEP_PIN, OUTPUT);
    pinMode(RESET_PIN, OUTPUT);
    pinMode(M0_PIN, OUTPUT);
    pinMode(M1_PIN, OUTPUT);
    pinMode(M2_PIN, OUTPUT);

    digitalWrite(STEP_PIN, LOW);
    digitalWrite(DIR_PIN, LOW);
    digitalWrite(ENABLE_PIN, HIGH);
    digitalWrite(SLEEP_PIN, HIGH);
    digitalWrite(RESET_PIN, HIGH);
    digitalWrite(M0_PIN, LOW);
    digitalWrite(M1_PIN, LOW);
    digitalWrite(M2_PIN, LOW);

    //Init hall 
    pinMode(HALL_PIN, INPUT_PULLUP );

    //TMP32 temperature sensor
    //set the resolution to 12 bits (0-4096)
    analogReadResolution(12);

    //init fan - attach to PWM channel 
    ledcAttachPin(FAN_PIN, FAN_PWM_CHANNEL);
    ledcSetup(FAN_PWM_CHANNEL, FAN_PWM_FREQ, FAN_PWM_RESOLUTION);
    ledcWrite(FAN_PWM_CHANNEL, 0);
 
}
