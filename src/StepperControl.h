/*
  StepperControl.h - Library for control stepper motor 
  using acceleration, deceleration and vmax on ESP32.
  Created by David A. Sommacal, March 31, 2020.
  Released into the public domain.
*/
    #ifndef StepperControl_h
    #define StepperControl_h
    #include "HardwareConfig.h"
    #include "Parameters.h"
    
    #define MOTOR_DEBUG_ON (1)

  //step per revolution, step pin, dir pin, microstepping
    //0 full step, 1 half, 2 1/4,3 1/8, 4 1/16, 5 1/32 
    void motor_init();
    void SetParameter(float Vmax, float Amax, float Dmax);    //% of Max
    void MoveAbs(float _s );
    void MoveRel(float _s);
    void Jog();

    void MotorMonitor();
    void SetMicroSteps(int mic);
    void PrintData();  
    void PlanTrajectory(float _s);
    void SetCurrentPos(float pos);
    void SavePosToEEPR();
    
      //Globals sync variables
  //volatile will let known the compiler the variable value may change outside the current task, like inside an ISR
    extern volatile boolean dir, motorStart, moveReady, moveDone, jogReady ;     
 
    extern float  K, F;
    extern int microsteps;
    extern float vM, aM, dM, m;
    extern float vs,vv;
    extern float T,Tad, ta,tv,td;
    extern float sad;   
    extern volatile float alfa, Af, Vsf, Df; //Af[steps/Ts^2],Vsf[steps/Ts]
    extern volatile float CurSpeed, CurPos; //
    extern volatile long N, Na, Nv, Nd, Nad, cN, Nint;

#endif
