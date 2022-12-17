#include "TouchControl.h"

int tp_speed;

static uint32_t s_pad_init_val[TOUCH_PAD_MAX];
uint32_t s_pad_actual_val[TOUCH_PAD_MAX];

/*
  Read and save values sensed at all available touch pads.
  Do not touch any pads when this routine
  is running (on application start).
 */
static void tp_read_init_value(void){
    uint16_t touch_value;
    for (int i = TOUCH_FIRST_PAD; i < TOUCH_PAD_MAX; i++) {
        //read filtered value
        #if TOUCH_FILTER_MODE_EN
                touch_pad_read_filtered(touch_pad_t(i), &touch_value);
        #else
                touch_pad_read(touch_pad_t(i), &touch_value);
        #endif
        s_pad_init_val[i] = touch_value;
        #if TOUCH_DEBUG_ON
        printf("test init: touch pad [%d] val is %d", i, touch_value);
        printf("\n");
        #endif        
    }
}

void tp_press_home_time(void){
     uint16_t _delay = 0;
     uint16_t home_value = 0;
     bool home_1_sec_on = false;
     while (home_value < s_pad_init_val[2] * TOUCH_THRESH_PERCENT / 100) {
            
            vTaskDelay(10 / portTICK_PERIOD_MS);
            _delay++;
            if (_delay >100 && !home_1_sec_on){
                home_1_sec_on = true;
                #if TOUCH_DEBUG_ON
                printf("button 2 press 1 sec");
                printf("\n");
                #endif 
            }
            if (_delay >200){
                #if TOUCH_DEBUG_ON
                printf("button 2 press 2 sec");
                printf("\n");
                #endif
                return;

            }
        touch_pad_read(touch_pad_t(2), &home_value);
     }
}

void tp_check_state(void){

static int_fast8_t tp_actual_state;
static int_fast8_t tp_prev_state;
static bool tp_home_touched, tp_istouch;

    if (s_pad_actual_val[2] && !tp_home_touched){
        tp_home_touched = true;
        tp_press_home_time(); 
    } 
    else if(!s_pad_actual_val[2]) tp_home_touched = false;

         if (s_pad_actual_val[3] || (s_pad_actual_val[3] && s_pad_actual_val[9])){ tp_actual_state = 3; tp_istouch=true; }
    else if (s_pad_actual_val[4] || (s_pad_actual_val[4] && s_pad_actual_val[3])){ tp_actual_state = 4; tp_istouch=true; }
    else if (s_pad_actual_val[5] || (s_pad_actual_val[5] && s_pad_actual_val[4])){ tp_actual_state = 5; tp_istouch=true; }
    else if (s_pad_actual_val[6] || (s_pad_actual_val[6] && s_pad_actual_val[5])){ tp_actual_state = 6; tp_istouch=true; }
    else if (s_pad_actual_val[7] || (s_pad_actual_val[7] && s_pad_actual_val[6])){ tp_actual_state = 7; tp_istouch=true; }
    else if (s_pad_actual_val[8] || (s_pad_actual_val[8] && s_pad_actual_val[7])){ tp_actual_state = 8; tp_istouch=true; }
    else if (s_pad_actual_val[9] || (s_pad_actual_val[9] && s_pad_actual_val[8])){ tp_actual_state = 9; tp_istouch=true; }
    else if (!s_pad_actual_val[2]) tp_istouch=false;

    if (tp_actual_state != tp_prev_state){
        #if TOUCH_DEBUG_ON
            printf("State is:%d ,", tp_actual_state);  
            printf("\n");                 
        #endif

        if (tp_actual_state>3 && tp_actual_state<9 ){
            if (tp_actual_state > tp_prev_state) tp_speed += TOUCH_STEP_INC;       
            else tp_speed -= TOUCH_STEP_INC;            
        }
        else if ((tp_actual_state==3) && (tp_prev_state==9)) tp_speed += TOUCH_STEP_INC;
        else if ((tp_actual_state==9) && (tp_prev_state==3)) tp_speed -= TOUCH_STEP_INC;

        tp_prev_state = tp_actual_state;
        
            #if TOUCH_DEBUG_ON
                printf("Speed is :%d ,", tp_speed);  
                printf("\n");                 
            #endif
    }

    if (!tp_istouch && tp_speed != 0){
        if (tp_speed>0) tp_speed -= TOUCH_STEP_INC;
        else  tp_speed += TOUCH_STEP_INC;
        #if TOUCH_DEBUG_ON
            printf("Speed is :%d ,", tp_speed);  
            printf("\n");                 
        #endif
    }

    motor.Jog_vel = tp_speed;

    if(abs(tp_speed)>10 && motor.mode != motor_mode::JOG){
        motor.mode = motor_mode::JOG;
        motor.Execute = true;
    }

}


static void tp_read_task(void *pvParameter)
{

    while (1) {
        #if TOUCH_DEBUG_ON
        static int show_message;
        static bool printline = false;
        #endif

            //filter mode, disable touch interrupt
            for (int i = TOUCH_FIRST_PAD; i < TOUCH_PAD_MAX; i++) {
                uint16_t value = 0;
                    #if TOUCH_FILTER_MODE_EN
                    touch_pad_read_filtered(touch_pad_t(i), &value);
                    #else
                    touch_pad_read(touch_pad_t(i), &value);
                    #endif
                if (value < s_pad_init_val[i] * TOUCH_THRESH_PERCENT / 100) {
                    s_pad_actual_val[i] = 1;              
                    #if TOUCH_DEBUG_ON
                    //printf("T%d ,", i);                   
                    //printline = true;
                    show_message = 1;
                    #endif
                   
                } 
                else s_pad_actual_val[i] = 0;               
            }
        tp_check_state();

        #if TOUCH_DEBUG_ON
        
        if(printline){
            printline = false;
            printf("\n");
        }
        // If no pad is touched, every ...seconds, show a message
        // that application is running
        if (show_message++ % 500 == 0) {
            printf("Waiting for any pad being touched...");
            printf("\n");           
        }           
         #endif
         vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}




/*
 * Before reading touch pad, we need to initialize the RTC IO.
 */
static void tp_touch_pad_init(void)
{
    for (int i = TOUCH_FIRST_PAD; i < TOUCH_PAD_MAX; i++) {
        //init RTC IO and mode for touch pad.
        touch_pad_config(touch_pad_t(i), TOUCH_THRESH_NO_USE);
    }
}

void tp_init(void)
{
    // Initialize touch pad peripheral, it will start a timer to run a filter
    #if TOUCH_DEBUG_ON
        printf("Initializing touch pad");
        printf("\n");
    #endif
    touch_pad_init();
    // If use interrupt trigger mode, should set touch sensor FSM mode at 'TOUCH_FSM_MODE_TIMER'.
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_SW);
    // Set reference voltage for charging/discharging
    // For most usage scenarios, we recommend using the following combination:
    // the high reference valtage will be 2.7V - 1V = 1.7V, The low reference voltage will be 0.5V.
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);
    // Init touch pad IO
    tp_touch_pad_init();
    // Initialize and start a software filter to detect slight change of capacitance.
    #if TOUCH_FILTER_MODE_EN
    touch_pad_filter_start(TOUCHPAD_FILTER_TOUCH_PERIOD);
    #endif
    // Set thresh hold
    tp_read_init_value();
    // Start a task to show what pads have been touched on core 0 (first core)
    xTaskCreatePinnedToCore(&tp_read_task, "touch_pad_read_task", 2048, NULL, 5, NULL,0);
}