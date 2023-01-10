#ifndef HardwareConfig_h
#define HardwareConfig_h   

//--------------- RESOURCE ALLOCATION -----------------//    

    //--------- TIMER ----------//
    #define TIMER_WATCHDOG 0    // + LEDC
    #define TIMER_WIFI     1
    //Timer 2 is FREE
    #define TIMER_MOTOR    3

    //--------- PWM CHANNEL ----------//
    /*
    ** LEDC Chan to Group/Channel/Timer Mapping
    ** ledc: 0  => Group: 0, Channel: 0, Timer: 0
    */
    // use first channel of 16 channels (started from zero)
    #define FAN_PWM_CHANNEL 0

//--------------- HARDWARE PARAMETER -----------------//
    // use 5000 Hz as a LEDC base frequency
    #define FAN_PWM_FREQ        30000
    // bit precission for LEDC timer
    #define FAN_PWM_RESOLUTION  8


//--------------- DEFINE PIN -----------------//
    //MOTOR DRIVER DRV8825 
    //Max current is manually set using potentiometer
    #define STEP_PIN        4
    #define DIR_PIN         0   
    #define ENABLE_PIN      21
    #define SLEEP_PIN       16
    #define RESET_PIN       22
    #define M0_PIN          17
    #define M1_PIN          18 
    #define M2_PIN          19

    //TMP36 Temperature Sensor TO-92 TMP36GT9Z
    //From Datasheet: Offset Voltage 0.5 (V)  
    //Output Voltage Scaling (mV/°C) 10
    //Output Voltage at 25°C (mV) 750
    #define TMP39_PIN       39
    //Mosfet Output PWM pin for controlling the FAN 
    #define FAN_PIN         25

    //HALL Effect sensor UTC(Unisonic Tech) SK1816G-G03-K
    #define HALL_PIN        5

    //LED WS2812B
    #define LED_PIN         26
   
    //#define I2C_SDA       12
    //#define I2C_SCL       13
    
    // JTAG PIN
    //#define TDO       15
    //#define TMS       14
    //#define TCK       13
    //#define TDI       12

//--------------- FUNCTION -----------------//
    void initHardware();
#endif