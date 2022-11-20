/*
  StepperControl.cpp - Library for control stepper motor 
  using acceleration, deceleration and vmax on ESP32.
  Created by A. Sommacal, March 31, 2020.
  Released into the public domain.
*/
#include "StepperControl.h"


    hw_timer_t * timer2 = NULL;
    portMUX_TYPE timerMux2 = portMUX_INITIALIZER_UNLOCKED;
    volatile SemaphoreHandle_t timerSemaphore;

    float VelMaxRpm = 800.0, AccMaxRpm = 600.0, DecMaxRpm = 600.0;
    float VelMaxRads, AccMaxRads, DecMaxRads ;
    int microsteps;
    float  K, F = 20000; //50000
    float vM, aM, dM, m;
    float vs = 0.0,vv ;
    float sad;
    float T,Tad, ta,tv,td;
    volatile float alfa, Af, Vsf, Df; //Af[steps/Ts^2],Vsf[steps/Ts]
    volatile float CurSpeed, CurPos; //
    volatile long N, Na, Nv, Nd, Nad, cN, Nint;
    volatile boolean dir, MotStop = true;

    

// interrupt procedure, every 1/10000 [sec]=1/10[ms] program calls this procedure
void IRAM_ATTR onTimer2() {
  //Critical Code here
  digitalWrite(STEP_PIN, LOW);  
  portENTER_CRITICAL_ISR(&timerMux2);
       if (!MotStop){ 
    
        CurSpeed = CurSpeed + Af;
        CurPos = CurPos + CurSpeed;
            if (CurPos - Nint >= 1) {
                  digitalWrite(STEP_PIN, HIGH);
                  if (dir)  cN++; 
                  else  cN--; 
                  Nint++;
            }
        if (Nint > Na && Nint <= N-Nd) Af = 0; //speed is constant
        if (Nint > N-Nd) Af = Df; //begin of deceleration             
        
    }
  // Give a semaphore that we can check in the loop  
  portEXIT_CRITICAL_ISR(&timerMux2);  
  if (Nint >= N ) MotorOFF(); 
  xSemaphoreGiveFromISR(timerSemaphore, NULL); 
}


void motor_init()
{ 
  Serial.print("MotorSetup");Serial.println(xPortGetCoreID());
 

  VelMaxRads = abs(VelMaxRpm)*TWO_PI/60; 
  AccMaxRads = abs(AccMaxRpm)*TWO_PI/60; 
  DecMaxRads = abs(DecMaxRpm)*TWO_PI/60;
  m = AccMaxRads/DecMaxRads;
 
    if (!EEPROM.begin(1000)) {
    Serial.println("Failed to initialise EEPROM");
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }
  
  cN = EEPROM.readLong(0);
  microsteps = EEPROM.readFloat(sizeof(long));
   if (DEBUG_ON){
        Serial.print("Cn read from EEPROM:"); Serial.println(cN);
        Serial.print("microsteps read from EEPROM:"); Serial.println(microsteps);
     }
  if (!(microsteps == 1 || microsteps == 2 || microsteps == 4 || microsteps == 8 || microsteps == 16 || microsteps == 32 )) {
    microsteps = 1;
    cN = 0;
    SavePosToEEPR();
    Serial.println("Setting microsteps to 1 ,need to set zero ");
    Serial.print("K Is equal to :"); Serial.println(200.0*(float)microsteps);
  }
  K = 200.0*(float)microsteps;
  alfa = TWO_PI/K;
  
  SetParameter(10,10,10);

  motor.microsteps = microsteps;

  motor.Position=cN*360.0/K;

  timerSemaphore = xSemaphoreCreateBinary();
  
  //xTaskCreatePinnedToCore(&motor_main, "motor_main_task", 4096, NULL, 5, NULL,1);

//  timer0 = timerBegin(0, 16, true);  // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 16 -> 200 ns = 0,2 us, countUp
//  timerAttachInterrupt(timer0, &onTimer0, true); // edge (not level) triggered
//  timerAlarmWrite(timer0, 100, true); //100 * 0,2 us = 20 us, autoreload true
//  timerAlarmEnable(timer0);
}

void SetCurrentPos(float pos){
  cN = long(pos*K/360.0);
   if (DEBUG_ON){
        Serial.print("Set Cn to:"); Serial.println(cN);
     }
}

void SavePosToEEPR(){
  EEPROM.writeLong(0,cN);
  EEPROM.writeFloat(sizeof(long),microsteps);
  EEPROM.commit();
     if (DEBUG_ON){
        Serial.print("Cn write to EERPROM:"); Serial.println(cN);
        Serial.print("microsteps write to EEPROM:"); Serial.println(microsteps);
     }
}


void SetMicroSteps(float mic){
  
  if(cN != 0) {
      return;
  }
  motor.microsteps = mic;
  microsteps = mic;
  K = 200.0*(float)microsteps;
  alfa = TWO_PI/K;
    
  if (microsteps==1){
     digitalWrite(M0_PIN, LOW);  digitalWrite(M1_PIN, LOW);  digitalWrite(M2_PIN, LOW);
  }
  if (microsteps==2){
     digitalWrite(M0_PIN, HIGH);  digitalWrite(M1_PIN, LOW);  digitalWrite(M2_PIN, LOW);
  }
  if (microsteps==4){
     digitalWrite(M0_PIN, LOW);  digitalWrite(M1_PIN, HIGH);  digitalWrite(M2_PIN, LOW);
  }
  if (microsteps==8){
     digitalWrite(M0_PIN, HIGH);  digitalWrite(M1_PIN, HIGH);  digitalWrite(M2_PIN, LOW);
  }
  if (microsteps==16){
     digitalWrite(M0_PIN, LOW);  digitalWrite(M1_PIN, LOW);  digitalWrite(M2_PIN, HIGH);
  }
  if (microsteps==32){
     digitalWrite(M0_PIN, HIGH);  digitalWrite(M1_PIN, HIGH);  digitalWrite(M2_PIN, HIGH);
  }
  if (DEBUG_ON){
        Serial.print("Setting microsteps to "); Serial.println(microsteps);
     }
  
  SetParameter(10, 10, 10);
}

void SetParameter(float _v, float _a, float _d){
        motor.vel = _v;;
        motor.acc = _a;
        motor.dec = _d;
      vM = VelMaxRads*_v*(float)microsteps/100.0;
      aM = AccMaxRads*_a*(float)microsteps/100.0;
      dM = DecMaxRads*_d*(float)microsteps/100.0;
      m = aM/dM;   
      if (DEBUG_ON){
        Serial.print("Set Vm to: "); Serial.println(vM);
        Serial.print("Set Am to: "); Serial.println(aM);
        Serial.print("Set Dm to: "); Serial.println(dM);
     }   
}      

void MoveAbs(float _s){
  if (!MotStop || act_state == EMERGENCY) return;
    PlanTrajectory( _s);
}

void MoveRel(float _s){
  if (!MotStop || act_state == EMERGENCY ) return;
    PlanTrajectory( _s + cN*360.0/K);
}

void MotorON(){  
    digitalWrite(ENABLE_PIN, LOW);
    if (dir) digitalWrite(DIR_PIN, HIGH);
    else digitalWrite(DIR_PIN, LOW);
  act_mode = MODE_OPERATION_ENABLED;   
  MotStop=false;
  timer2 = timerBegin(2, 8, true);  // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 8 -> 100 ns = 0,1 us, countUp
  timerAttachInterrupt(timer2, &onTimer2, true); // edge (not level) triggered
  timerAlarmWrite(timer2, 500, true); //500 * 0,1 us = 50 us, autoreload true
  timerAlarmEnable(timer2);
  
}
void MotorOFF(){
   //timerStop(timer2);
   timerEnd(timer2);
   timer2 = NULL;  
   MotStop=true;
  digitalWrite(STEP_PIN, LOW); 
  digitalWrite(ENABLE_PIN, HIGH);
  act_mode = MODE_READY;
  if (DEBUG_ON){  
        PrintData();
  }
         //leds[0] = CRGB::Green; 
         //uncomment cause CRASH
         //FastLED.show();
}  

  
//trajectory planning procedure
void PlanTrajectory(float _s){ 
    
 N = long(_s*K/360.0) - cN ; //difference from desired and current position [steps]
 if (N == 0) {Nint = -1; return;} //
    if (N < 0) dir = false;
    else dir = true;    

        //Compute trajectory
        N = abs(N); float s_ = N*alfa;
        T = sqrt( pow( (vs*(1+m)/aM),2)+2*s_*(1+m)/aM )-vs*(1+m)/aM; //N = round(s/alfa);
        ta = T/(1+m); vv = vs + aM*ta; //td = (T-ta); tv = 0;

        if (vv <= vM){//TRIANGULAR PROFILE
              Na = int(N/(1+m)); Nd = N - Na; Nv = 0;
              td = T - ta, tv=0;
              }
        else{ //TRAPEZOIDAL PROFILE
              vv = vM; ta = (vM-vs)/aM, Tad = (1+m)*ta, td = Tad - ta; 
              sad = vs*Tad+aM*pow(Tad,2)/(2*(1+m));
              tv = (s_-sad)/vv, T = Tad+tv ;
              Nad = int(sad/alfa); Na = int(Nad/(m+1)); Nd = Nad - Na; Nv = N - Nad; //Nv - number of steps in the phase v=const.
            }
    //Vsf [steps/0.1ms] - the start speed, Af[steps/(0.1ms)^2]-acceleration, Df-deceleration
    Vsf = vs/alfa/F; Af = aM/alfa/F/F; Df = -Af/m;
    CurPos = 0; CurSpeed= 0; Nint=0;

    if (DEBUG_ON){
Serial.print("N = "); Serial.println(N);
Serial.print("Na = "); Serial.println(Na);
Serial.print("Nv = "); Serial.println(Nv);
Serial.print("Nd = "); Serial.println(Nd);
Serial.print("_s = "); Serial.println(_s);
Serial.print("s_ = "); Serial.println(s_*360/TWO_PI);
Serial.print("sad= "); Serial.println(sad);
Serial.print("T = "); Serial.println(T);
Serial.print("Ta = "); Serial.println(ta);
Serial.print("Tv = "); Serial.println(tv);
Serial.print("Td = "); Serial.println(td);
Serial.print("vv = "); Serial.println(vv);
Serial.print("Vsf = "); Serial.println(Vsf,6);
Serial.print("Af = "); Serial.println(Af,5);
Serial.print("Df = "); Serial.println(Df,5);
}
  //Start Motor
  MotorON();
}

//this procedure displaying current data
void  PrintData(){
Serial.print("\n..............................................\n");
Serial.print("MotStop = "); Serial.println(MotStop);
Serial.print("Nint = "); Serial.println(Nint);
Serial.print("STEPS = "); Serial.print(cN); Serial.print(" [steps]\n");
Serial.print("ANGLE = "); Serial.print(cN*360.0/K,2);Serial.print(" [deg]\n");       
Serial.print("Start. speed vs = ");Serial.print(vs,2);Serial.print("[rad/sec]\n");
Serial.print("Max. speed vM = ");Serial.print(vM,2);Serial.print("[rad/sec]\n");
Serial.print("Max. accler. aM = ");Serial.print(aM,2);Serial.print("[rad/sec^2]\n");
Serial.print("VelMaxRmp = ");Serial.println(VelMaxRpm);
Serial.print("AccMaxRmp = ");Serial.println(AccMaxRpm);
Serial.print("DecMaxRmp = ");Serial.println(DecMaxRpm);
Serial.print("Ratio of a/d m = ");Serial.print(m,2); Serial.print("\n");
Serial.print("K = ");Serial.print(K,2); Serial.print("\n");
Serial.print("..............................................\n");
}

