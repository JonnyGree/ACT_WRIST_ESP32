/*
  StepperControl.h - Library for control stepper motor 
  using acceleration, deceleration and vmax on ESP32.
  Created by David A. Sommacal, March 31, 2020.
  Released into the public domain.

  Implementation of:
  "Generate stepper motor linear speed profile in real time, M Y Stoychitch
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

    // absolute movement [deg] 
    void MoveAbs(float _s );
    // relative movement [deg] 
    void MoveRel(float _s);
    void Jog();

    void MotorMonitor();
    void SetMicroSteps(int mic);
    void PrintData();  
    void PlanTrajectory(float _s);
    void SetCurrentPos(float pos);
    void SavePosToEEPR();

  // ------------------------------------------------ VARIABLES ------------------------------------------//    
  //Globals sync variables
  //volatile will let known the compiler the variable value may change outside the current task, like inside an ISR
    extern volatile boolean dir, motorStart, moveReady, moveDone, jogReady ;  

    // ISR Timer
    static const uint16_t timir_divider   = 80;//F = 1 000 000, tick every 0. 000 001 S
    static const uint64_t timir_max_count = 10;  

    // INPUT:
    // Speed start [rad/s], Max speed [rad/s], Max acceleration [rad/s^2], Max deceleration [rad/s^2], m= a/d
    extern float vs , vM , aM , dM ;
    // m = aM / dM
    extern float m ;
    // Conversion:
    // [steps /T^2] = [rad/T^2] *K * TWO_PI / F^2
    // [steps /T  ] = [rad/T  ] *K * TWO_PI / F
 
    // K Number of steps per revolution [steps/rev]
    extern float  K ;
    // Integral Frequency
    extern float  F ;
    // Driver microsteps. Total number of steps per rev are equal to K = microsteps * mechanical motor steps
    extern int microsteps ;


    // Time: Total time of motion, acceleration + deceleration,
    // Acceleration, constant speed, deceleration
    // [s]
    extern volatile float T, Tad, ta, tv, td;
    // N difference from desired and current position [steps] 
    // Na... Number of steps in each phase [step]
    extern volatile long  N, Na, Nv, Nd, Nad;
    extern float sad;   
    //Initialized at maximum
    extern volatile float alfa , Af , Vsf , Df ; //alfa[rad/step] Af[steps/Ts^2] Vsf[steps/Ts]
    
    // Current number of steps
    extern volatile long cN;

    // Integral manipulated value
    extern volatile long Nint;
    extern volatile float CurSpeed, CurPos; 
    // Max velocity in the trajectory
    extern volatile float vv;

  // -------------------------------------------- Algorithm ----------------------------------------
  // at the beginning we assume a triangular speed profile, acceleration + deceleration
  // total movement S = S0 +Sa + Sd = Vs*T + 0.5a*Ta^2 + aTa*Td -0.5dTd^2

  // INPUT: s, Vs, Vm, Am, K, m=a/d
  // alfa = 2PI/K, N = s/alfa, 



#endif
