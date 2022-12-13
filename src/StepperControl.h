/*
  StepperControl.h - Library for control stepper motor 
  using acceleration, deceleration and vmax on ESP32.
  Created by David A. Sommacal, March 31, 2020.
  Released into the public domain.
*/
    #ifndef StepperControl_h
    #define StepperControl_h
    #include "Parameters.h"
    #include "EEPROM.h"
    
    #define MOTOR_DEBUG_ON (1)
    #define DIOLAZZARO    (0)

    //Maximum parameter
    #define MOTOR_VEL_MAX_RPM (300.0)
    #define MOTOR_ACC_MAX_RPM (300.0)
    #define MOTOR_DEC_MAX_RPM (300.0)

    //Motor step per revolution
    #define MOTOR_STEPS_PER_REV (200.0)
    #define MOTOR_INTERR_F      (20000.0) //50000

  //step per revolution, step pin, dir pin, microstepping
    //0 full step, 1 half, 2 1/4,3 1/8, 4 1/16, 5 1/32 
    void motor_init();
    void SetParameter(float Vmax, float Amax, float Dmax);    //% of Max
    void MoveAbs(float _s );
    void MoveRel(float _s);
    void IRAM_ATTR onTimer0();
    void MotorON();
    void MotorOFF();  
    void SetMicroSteps(float mic);
    void PrintData();  
    void PlanTrajectory(float _s);
    void SetCurrentPos(float pos);
    void SavePosToEEPR();
    
 
    //max v 700 acc 600   
    extern float  K, F;
    extern float VelMaxRpm, AccMaxRpm, DecMaxRpm;
    extern float VelMaxRads, AccMaxRads, DecMaxRads ;
    extern int microsteps;
    extern float vM, aM, dM, m;
    extern float vs,vv;
    extern float T,Tad, ta,tv,td;
    extern float sad;   
    extern volatile float alfa, Af, Vsf, Df; //Af[steps/Ts^2],Vsf[steps/Ts]
    extern volatile float CurSpeed, CurPos; //
    extern volatile long N, Na, Nv, Nd, Nad, cN, Nint;
    extern volatile boolean dir, MotStop;

#endif
