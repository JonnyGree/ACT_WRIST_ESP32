
#include <Arduino.h>
//#include <esp_now.h>

#include "HardwareConfig.h"
#include "Parameters.h"
#include "tmp32.h"

// #include <WiFi.h>
// #include "WIFI_Main.h"

// #include "StepperControl.h"
// #include "tmp32.h"
// #include "Hall_Homing.h"
// #include "TouchControl.h"
 #include "Led_WS2812B.h"


//Watchdog timer
// const int wdtTimeout = 3000;  //time in ms to trigger the watchdog
// hw_timer_t *timer0 = NULL;
// void IRAM_ATTR resetModule() {
//       ets_printf("reboot\n");
//       esp_restart();
// }

//Globals
static TimerHandle_t Timer_5S = NULL;

// CALLBACKS

//Called when the timer expires
void Timer_5S_Callback(TimerHandle_t xTimer){
  Serial.println("Timer Expired");
}  


//----------------------------SETUP------------------------//

void setup() {

  Timer_5S =  xTimerCreate( "Timer_5S",                           // Text name for the task.  Helps debugging only.  Not used by FreeRTOS.
                                  5000 / portTICK_PERIOD_MS,      // The period of the timer in ticks.
                                  pdTRUE,                         // This is an auto-reload timer.
                                  ( void * ) 0,                   // A variable incremented by the software timer's callback function
                                  Timer_5S_Callback);               // The function to execute when the timer expires.
  //setCpuFrequencyMhz(80); //Set CPU clock to 80MHz fo example
  initHardware();

  // Initialize Serial Monitor
  Serial.begin(115200);

  // //setup WIFI
  // WIFI_init();


  // //Set Pin
  // initpin();

  // //setup motor 
  // motor_init();

  // //setup led
   led_init();
  //setup tmp32 & fan
  tmp32_init();
  // //setup hall sensor and homing sequence
  // hall_init();
  // //setup touchpad
  // tp_init();

 

  // //Register watchdog timer
  // timer0 = timerBegin(TIMER_WATCHDOG, 80, true);     //timer 0, div 80
  // timerAttachInterrupt(timer0, &resetModule, true);  //attach callback
  // timerAlarmWrite(timer0, wdtTimeout * 1000, false); //set time in us
  // //timerAlarmEnable(timer0);


}

void loop() { 
  // timerWrite(timer0, 0); //reset timer (feed watchdog)   

  //     if(motor.Execute){

  //       if(motor.mode == motor_mode::MOVE_ABS ){
  //           MoveAbs(motor.Position);          
  //       }
  //       else if(motor.mode == motor_mode::MOVE_REL ){
  //           MoveRel(motor.Position);           
  //       }
  //       else if (motor.mode == motor_mode::JOG && abs(motor.Jog_vel) >10 ){
  //           Serial.println("Chiamo Jog ");
  //           //Jog() ;
  //       }
  //       else if (motor.mode == motor_mode::SET_POS){
  //         SetCurrentPos(motor.Position);
  //       }
  //       else if (motor.mode == motor_mode::SET_SPEED){
  //         SetParameter(motor.vel, motor.acc, motor.dec);
  //       }
  //       else if (motor.mode == motor_mode::TURN_OFF){
  //         SavePosToEEPR();
  //       }
  //       motor.Execute = false;
  //   }
  //   if (motor.microsteps != microsteps && cN == 0)
  //   {
  //       SetMicroSteps(motor.microsteps);
  //   }



}
