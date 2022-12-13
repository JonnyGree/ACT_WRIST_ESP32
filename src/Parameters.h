/*
  StepperControl.h - Library for control stepper motor 
  using acceleration, deceleration and vmax on ESP32.
  Created by A. Sommacal, March 31, 2020.
  Released into the public domain.
*/
    #ifndef Parameters_h
    #define Parameters_h

    #include <stdio.h>
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "freertos/queue.h"

    #include "esp_system.h"
    #include "Arduino.h"

    #define ACTUATOR_NUMBER 6

    typedef enum {
    EMERGENCY = 2,       
    WARNING = 1,       
    STATUS_OK = 0,
    } actuator_current_state;
    extern actuator_current_state act_state;

    typedef enum {    
    MODE_OPERATION_ENABLED = 4,  
    MODE_JOG = 3,      
    MODE_HOMING = 2, 
    MODE_READY = 1,         
    MODE_INIT = 0,   
    } actuator_current_mode;   
    extern actuator_current_mode act_mode;

    typedef enum {    
    MOVE_ABS = 0,  
    MOVE_REL = 1,      
    JOG = 2, 
    SET_POS = 3,
    SET_SPEED=4,
    TURN_OFF = 5,  //Save current position to eeprom 
    } motor_mode;   

    //Actuator parameter 
    typedef struct motor_data {
      bool Execute;
      motor_mode mode;
      float Position;
      float vel;
      float acc;
      float dec;
      float Jog_vel;
      int microsteps;
    } motor_data;

      extern motor_data motor;

    #define TIMER_WATCHDOG (0)
    #define TIMER_WIFI     (1)
    #define TIMER_MOTOR    (3)
//---------------DEFINE PIN -----------------//
    #define STEP_PIN 4
    #define DIR_PIN 0   
    #define ENABLE_PIN 21
    #define SLEEP_PIN 16
    #define RESET_PIN 22
    #define M0_PIN 17
    #define M1_PIN 18 
    #define M2_PIN 19

    #define TMP39_PIN 39
    #define FAN_PIN 25
    #define FAN_FREQ 20000
    #define PWM_RESOLUTION 8
    #define PWM_CHANNEL 1

    #define HALL_PIN 5
    #define NUM_LEDS 1
    #define LED_PIN 26
   
    //#define I2C_SDA 12
    //#define I2C_SCL 13

   extern boolean DEBUG_ON;
   void initpin();

#endif
