#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "WIFI_Main.h"
#include "Parameters.h"
#include "StepperControl.h"
#include "tmp32.h"
#include "Hall_Homing.h"
#include "TouchControl.h"
#include "Led_WS2812B.h"


//Watchdog timer
const int wdtTimeout = 3000;  //time in ms to trigger the watchdog
hw_timer_t *timer0 = NULL;
void IRAM_ATTR resetModule() {
      ets_printf("reboot\n");
      esp_restart();
}



//----------------------------SETUP------------------------//

void setup() {
  setCpuFrequencyMhz(80); //Set CPU clock to 80MHz fo example

  //setup WIFI
  WIFI_init();


  //Set Pin
  initpin();
  // Initialize Serial Monitor
  Serial.begin(115200);
  //setup motor 
  motor_init();

  //setup led
  led_init();
  //setup tmp32 & fan
  tmp32_init();
  //setup hall sensor and homing sequence
  hall_init();
  //setup touchpad
  tp_init();

 

  //Register watchdog timer
  timer0 = timerBegin(TIMER_WATCHDOG, 80, true);     //timer 0, div 80
  timerAttachInterrupt(timer0, &resetModule, true);  //attach callback
  timerAlarmWrite(timer0, wdtTimeout * 1000, false); //set time in us
  //timerAlarmEnable(timer0);


}

void loop() { 
  timerWrite(timer0, 0); //reset timer (feed watchdog)   

      if(motor.Execute){

        if(motor.mode == motor_mode::MOVE_ABS ){
            MoveAbs(motor.Position);          
        }
        else if(motor.mode == motor_mode::MOVE_REL ){
            MoveRel(motor.Position);           
        }
        else if (motor.mode == motor_mode::JOG && abs(motor.Jog_vel) >10 ){
            Serial.println("Chiamo Jog ");
            //Jog() ;
        }
        else if (motor.mode == motor_mode::SET_POS){
          SetCurrentPos(motor.Position);
        }
        else if (motor.mode == motor_mode::SET_SPEED){
          SetParameter(motor.vel, motor.acc, motor.dec);
        }
        else if (motor.mode == motor_mode::TURN_OFF){
          SavePosToEEPR();
        }
        motor.Execute = false;
    }
    if (motor.microsteps != microsteps && cN == 0)
    {
        SetMicroSteps(motor.microsteps);
    }
}
