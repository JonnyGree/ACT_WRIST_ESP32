/*
  StepperControl.cpp - Library for control stepper motor 
  using acceleration, deceleration and vmax on ESP32.
  Created by A. Sommacal, March 31, 2020.
  Released into the public domain.
*/
#include "StepperControl.h"



  //Globals sync variables
  //volatile will let known the compiler the variable value may change outside the current task, like inside an ISR
  volatile boolean dir, motorStart = false, moveReady = false, moveDone = false, jogReady = false;                        



  static float VelMaxRads = MOTOR_VEL_MAX_RPM*TWO_PI/60, AccMaxRads = MOTOR_ACC_MAX_RPM*TWO_PI/60, DecMaxRads = MOTOR_DEC_MAX_RPM*TWO_PI/60 ;
  int microsteps;
  float  K, F = 100000; //50000
  float vM, aM, dM, m;
  float JogAccMax, JogAcc, JogSpeed;
  int cyclecounter;
  float vs = 0.0,vv ;
  float sad;
  float T,Tad, ta,tv,td;
  volatile float  alfa, Af, Vsf, Df; //Af[steps/Ts^2],Vsf[steps/Ts]
  volatile float  CurSpeed, CurPos; //

  // N difference betweek current and target position in step
  volatile long  N, Na, Nv, Nd, Nad, cN, Nint;




    

// // interrupt procedure, every 1/10000 [sec]=1/10[ms] program calls this procedure
// void IRAM_ATTR onTimer3() {
//   //Critical Code here
//   REG_WRITE(GPIO_OUT_W1TC_REG, BIT4);  
//        if (motorStart){   
//         CurSpeed = CurSpeed + Af;
//         CurPos = CurPos + CurSpeed;
//             if (CurPos - Nint >= 1) {
//                   REG_WRITE(GPIO_OUT_W1TS_REG, BIT4);
//                   if (dir)  cN++; 
//                   else  cN--; 
//                   Nint++;
//             }
//         if (Nint > Na && Nint <= N-Nd) Af = 0;  //speed is constant
//         if (Nint > N-Nd) Af = Df;               //begin of deceleration             
//         if (Nint >= N || CurSpeed < 0.0) { 
//             motorStart = false;
//             moveDone = true;
//         }
    
//       }

    //   if (!JogStop){ 
    
    //     CurSpeed = CurSpeed + JogAcc;
    //     CurPos = CurPos + CurSpeed;
    //         if (CurPos - Nint >= 1) {
    //               digitalWrite(STEP_PIN, HIGH);
    //               if (dir)  cN++; 
    //               else  cN--; 
    //               Nint++;
    //         }
    //     JogSpeed = abs(data.Jog_vel) / 10.0 ;
    //     if (CurSpeed < (JogSpeed-1.0)) {
    //       if (JogSpeed > 4.0) JogAcc = 5*JogAccMax; //Accelerate
    //       else JogAcc = JogAccMax; //Accelerate
    //     }

    //     else if ((CurSpeed > JogSpeed +1.0) || data.Jog_vel == 0.0) JogAcc = -8.0*JogAccMax; //Decelerate         
    //     else  JogAcc = 0; //Constant Speed
    //     if (CurSpeed < 0.0) MotorOFF();

    // }
     
// }


void motor_init()
{ 
  Serial.print("MotorSetup on core");Serial.println(xPortGetCoreID());
 
  m = AccMaxRads/DecMaxRads;
 
    if (!EEPROM.begin(1000)) {
    Serial.println("Failed to initialise EEPROM");
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }
  
  cN = EEPROM.readLong(0);
  microsteps = EEPROM.readFloat(sizeof(long));
   #if MOTOR_DEBUG_ON
        Serial.print("Cn read from EEPROM:"); Serial.println(cN);
        Serial.print("microsteps read from EEPROM:"); Serial.println(microsteps);
   #endif
  if (!(microsteps == 1 || microsteps == 2 || microsteps == 4 || microsteps == 8 || microsteps == 16 || microsteps == 32 )) {
    microsteps = 1;
    cN = 0;
    SavePosToEEPR();
    Serial.println("Setting microsteps to 1 ,need to set zero ");
    Serial.print("K Is equal to :"); Serial.println(MOTOR_STEPS_PER_REV*(float)microsteps);
  }

  SetMicroSteps(microsteps);
  
  SetParameter(10,10,10);

  data.Position=cN*360.0/K;

}

void SetCurrentPos(float pos){
  cN = long(pos*K/360.0);
    #if MOTOR_DEBUG_ON
        Serial.print("Set Cn to:"); Serial.println(cN);
    #endif
}

void SavePosToEEPR(){
  EEPROM.writeLong(0,cN);
  EEPROM.writeFloat(sizeof(long),microsteps);
  EEPROM.commit();
     #if MOTOR_DEBUG_ON
        Serial.print("Cn write to EERPROM:"); Serial.println(cN);
        Serial.print("microsteps write to EEPROM:"); Serial.println(microsteps);
    #endif
}


void SetMicroSteps(int mic){
  
  if(cN != 0) {
      return;
  }
  data.microsteps = mic;
  microsteps = mic;
  K = MOTOR_STEPS_PER_REV*(float)microsteps;
  alfa = TWO_PI/K;
    
  if (microsteps==1){     digitalWrite(M0_PIN, LOW);  digitalWrite(M1_PIN, LOW);  digitalWrite(M2_PIN, LOW);  }
  if (microsteps==2){     digitalWrite(M0_PIN, HIGH); digitalWrite(M1_PIN, LOW);  digitalWrite(M2_PIN, LOW);  }
  if (microsteps==4){     digitalWrite(M0_PIN, LOW);  digitalWrite(M1_PIN, HIGH); digitalWrite(M2_PIN, LOW);  }
  if (microsteps==8){     digitalWrite(M0_PIN, HIGH); digitalWrite(M1_PIN, HIGH); digitalWrite(M2_PIN, LOW);  }
  if (microsteps==16){    digitalWrite(M0_PIN, LOW);  digitalWrite(M1_PIN, LOW);  digitalWrite(M2_PIN, HIGH); }
  if (microsteps==32){    digitalWrite(M0_PIN, HIGH); digitalWrite(M1_PIN, HIGH); digitalWrite(M2_PIN, HIGH); }
    #if MOTOR_DEBUG_ON
        Serial.print("Setting microsteps to "); Serial.println(microsteps);
    #endif
}

void SetParameter(float _v, float _a, float _d){
      data.vel = _v;;
      data.acc = _a;
      data.dec = _d;
      vM = VelMaxRads*_v/100.0;
      aM = AccMaxRads*_a/100.0;
      dM = DecMaxRads*_d/100.0;
      m = aM/dM;   

      JogAccMax = AccMaxRads*0.1/alfa/F/F;

      #if MOTOR_DEBUG_ON
        Serial.print("Set Vm to: "); Serial.println(vM);
        Serial.print("Set Am to: "); Serial.println(aM);
        Serial.print("Set Dm to: "); Serial.println(dM);
     #endif  

}      

void MoveAbs(float _s){
  if (data.mode != MODE_READY  || data.state == ALARM) return;
    PlanTrajectory( _s*MOTOR_REDUCTOR_RATIO);
}

void MoveRel(float _s){
  if (data.mode != MODE_READY  || data.state == ALARM) return;
    Serial.println("Call move rel");
    PlanTrajectory( _s + float(cN)*360.0/K);
}

void Jog(){

  if (data.mode != MODE_READY || data.state == ALARM) return;
  dir = (data.Jog_vel > 0.0) ;
  data.mode = MODE_JOG;   
  moveReady=true;
  jogReady=false;
  CurPos = 0.0; CurSpeed= 0.0; Nint=0;
  cyclecounter = 0;
  JogAcc =JogAccMax;

}


void MotorMonitor(){
  if  (moveDone){
    digitalWrite(STEP_PIN, LOW); 
    digitalWrite(ENABLE_PIN, HIGH);
    data.mode = MODE_READY;
    data.command = IDLE;
    moveDone = false;
  }
}  
//trajectory planning procedure
void PlanTrajectory(float _s){ 
 Serial.println("Call plan trajectory");   
 N = long(_s*K/360.0) - cN ;        //difference from desired and current position [steps]
 Serial.println(_s); 
 Serial.println(K); 
 if (N == 0) {Nint = -1; return;}   //
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

  #if MOTOR_DEBUG_ON
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
  #endif
  //Start Motor
  moveDone=false;
  jogReady=false; 
  data.mode = MODE_OPERATION_ENABLED;
  digitalWrite(ENABLE_PIN, LOW);
  if (dir) digitalWrite(DIR_PIN, HIGH);
  else digitalWrite(DIR_PIN, LOW); 
  motorStart = true;
}

//this procedure displaying current data
  void  PrintData(){
    Serial.print("\n..............................................\n");
    Serial.print("STEPS = "); Serial.print(cN); Serial.print(" [steps]\n");
    Serial.print("ANGLE = "); Serial.print(cN*360.0/K,2);Serial.print(" [deg]\n");  
    Serial.print("POS = "); Serial.print(CurPos);Serial.print(" [step]\n"); 
    Serial.print("SPEED = "); Serial.print(CurSpeed);Serial.print(" [step/s]\n");      
    Serial.print("Max. speed vM = ");Serial.print(vM,2);Serial.print("[rad/sec]\n");
    Serial.print("Max. accler. aM = ");Serial.print(aM,2);Serial.print("[rad/sec^2]\n");
    Serial.print("Ratio of a/d m = ");Serial.print(m,2); Serial.print("\n");
    Serial.print("K = ");Serial.print(K,2); Serial.print("\n");
    Serial.print("..............................................\n");
  }
