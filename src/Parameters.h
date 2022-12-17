/*
  StepperControl.h - Library for control stepper motor 
  using acceleration, deceleration and vmax on ESP32.
  Created by A. Sommacal, March 31, 2020.
  Released into the public domain.
*/
    #ifndef Parameters_h
    #define Parameters_h

    #include "Arduino.h"
    #include <stdio.h>
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "freertos/queue.h"
    #include "esp_system.h"

    // undefine stdlib's abs if encountered
    //avoid problem using float
    #ifdef abs
    #undef abs
    #endif

    #define abs(x) ((x)>0?(x):-(x))

    //------------------ PARAMETERS -------------------------//
    #define ACTUATOR_NUMBER 6

    //Deadband for regulate fan according to temperature.
    #define TEMPERATURE_DEADBAND 2.0    // C°

    //Led parameter
    #define NUM_LEDS            1
    #define LED_FLASH_DELAYVAL  300   // Time (in milliseconds) for led blinking
    #define LED_TASK_DELAYVAL   50    // Time (in milliseconds) to pause led scan cycle
                
    #define MAX_BRIGHTNESS 250   

    //------------------ PROCESS DATA -------------------------//
    //State of Actuator (How is it doing doing ?)
    typedef enum {
    ALARM = 2,       
    WARNING = 1,       
    STATUS_OK = 0,
    } actuator_state;

    //Mode of Actuator (What is it doing doing ?)
    typedef enum {    
    MODE_OPERATION_ENABLED = 4,   //Remote operation, Mov ABS or Mov Rel
    MODE_JOG = 3,      
    MODE_HOMING = 2,              //Homing sequence using hall effect sensor enabled
    MODE_READY = 1,         
    MODE_INIT = 0,            
    } actuator_mode;   

    //Motor Command
    typedef enum {    
    MOVE_ABS  = 0,  
    MOVE_REL  = 1,      
    JOG       = 2, 
    SET_POS   = 3,
    SET_SPEED = 4,
    TURN_OFF  = 5,  //Save current position to eeprom 
    IDLE      = 99, 
    } motor_command;  

    typedef enum{
      WHITE     = 0,
      YELLOW    = 1,
      RED       = 2,
      GREEN     = 3,
      VIOLET    = 4,
      BLUE      = 5,
      BLACK     = 10,
      COMMAND_DONE = 255,
    } led_color; 

    //Actuator data - global DUT 
    typedef struct actuator_data {
      actuator_state  state     = STATUS_OK;
      actuator_mode   mode      = MODE_INIT;  
      motor_command   command   = IDLE;
      led_color       ledColor  = BLACK;
      bool            Execute;
      float           Position;
      float           vel;
      float           acc;
      float           dec;
      float           Jog_vel;
      int             microsteps;
      uint32_t        fanSpeed = 0;      // 0-100
      float           temperature = 0.0;
    } actuator_data;
    extern actuator_data data;


#endif
