
#include <Arduino.h>
//#include <esp_now.h>

#include "HardwareConfig.h"
#include "Parameters.h"
#include "StepperControl.h"

  hw_timer_t * timer3 = NULL;  
  
  volatile long  timerCounter = 0;
  volatile long  OB_1ms = 0;
  volatile long  OB_10ms = 0;
  volatile long  OB_100ms = 0;
  volatile long  OB_1s = 0;
  volatile long  OB_5s = 0;

//  }

  void IRAM_ATTR onTimer3() {
  timerCounter++;

    REG_WRITE(GPIO_OUT_W1TC_REG, BIT4);  
       if (motorStart){                                 // Ts = 1 / F
        CurSpeed = CurSpeed + Af;                       //[steps /Ts] [steps /Ts^2]
        CurPos = CurPos + CurSpeed;                     //[steps   ] [steps /Ts]
            if (CurPos - Nint >= 1) {
                  REG_WRITE(GPIO_OUT_W1TS_REG, BIT4);
                  if (dir)  cN++; 
                  else  cN--; 
                  Nint++;
            }
        if (Nint > Na && Nint <= N-Nd) Af = 0;  //speed is constant
        if (Nint > N-Nd) Af = Df;               //begin of deceleration             
        if (Nint >= N || CurSpeed < 0.0) { 
            motorStart = false;
            moveDone = true;
        }
    
      }

}

//----------------------------SETUP------------------------//

void setup() {
  //setCpuFrequencyMhz(80); //Set CPU clock to 80MHz fo example
  initHardware();

  // Initialize Serial Monitor
  Serial.begin(115200);

  //setup motor 
  motor_init();

  timer3 = timerBegin(3, timir_divider, true);  // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 8 -> 100 ns = 0,1 us, countUp
  timerAttachInterrupt(timer3, &onTimer3, true); // edge (not level) triggered
  timerAlarmWrite(timer3, timir_max_count, true); //200 * 0,1 us = 20 us, autoreload true
  timerAlarmEnable(timer3);

  data.mode = MODE_READY;

}

float target = 90.0;

void loop() {
  if(timerCounter > 100){ 
    timerCounter = 0;
    
    OB_1ms++;
    OB_10ms++;
    OB_100ms++;
    OB_1s++;
    OB_5s++;

    if(OB_1ms>=1){
      OB_1ms = 0;
    } 

    if(OB_10ms>=10){
      OB_10ms = 0;
      MotorMonitor();
    }

    if(OB_100ms>=100){
      OB_100ms = 0;
    }

    if(OB_1s>=1000){
      OB_1s = 0;
    }

    if(OB_5s>=5000){
      OB_5s = 0;
      PrintData();
      if  (!motorStart){
        MoveRel(target);
      }
    }

  } 
}  


 
