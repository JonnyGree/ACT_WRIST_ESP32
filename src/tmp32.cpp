#include "tmp32.h"

float TempAverage=0.0;
float MemTempC = 0.0;
byte TempHasChanged = 0;
byte FanSpeed = 3;

float ReadTemperature(){ 
  float voltage = analogRead(TMP39_PIN) / 1024.0;
  float temperatureC = (voltage - 0.46) * 100 ;  //converting from 10 mv per degree wit 500 mV offset 
  return temperatureC;
}

void SetFan(byte _FanSpeed){
  #if TEMP_DEBUG_ON 
    Serial.print("Setting Speed to "); Serial.println(_FanSpeed);
  #endif
  switch (_FanSpeed) {            
            case 3: {
                ledcWrite(PWM_CHANNEL, 250);
                break;
              }
            case 2: {
                ledcWrite(PWM_CHANNEL, 200);
                break;
              }
            case 1: {
                ledcWrite(PWM_CHANNEL, 160);
                break;
              }
            case 0: {
                ledcWrite(PWM_CHANNEL, 0);
                break;
              }
           default:
              {}
              break;
  }
}

void CheckTemperature(){

   float SumTemperature =0.0;      
   for (int i= 0; i <4; i++) {
       SumTemperature += ReadTemperature(); 
       vTaskDelay(10 / portTICK_PERIOD_MS); 
    } 
    TempAverage = SumTemperature/4.0;
      #if TEMP_DEBUG_ON                                 
      Serial.print(TempAverage); Serial.println(" degrees C");
      #endif  
 
    if (abs(TempAverage - MemTempC) >2.0){
       TempHasChanged++;

    }

 
 if (TempHasChanged>4){

      if (TempAverage > 40.0) act_state = EMERGENCY ;
 else if (TempAverage > 30.0) act_state = WARNING   ;
 else act_state = STATUS_OK;

 
    TempHasChanged=0;
    MemTempC =TempAverage;
 
    if (TempAverage > 30.0 ){
          FanSpeed = 3; SetFan(3);   
        }
    else if (TempAverage > 28.0 ){
          FanSpeed = 2; SetFan(2);
        }
    else if (TempAverage > 24.0){
          FanSpeed = 1; SetFan(1);
        }
    else if (TempAverage > 0.0 ){
          FanSpeed = 0; SetFan(0);
        }
 }

}

static void tmp32_read( void * pvParameters ){    
  #if TEMP_DEBUG_ON
      Serial.print("Task1 running on core ");
      Serial.println(xPortGetCoreID());
  #endif

  while(1){
      CheckTemperature();
      vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}


void tmp32_init(){
  //init fan
  pinMode(FAN_PIN, OUTPUT);
  ledcSetup(PWM_CHANNEL, FAN_FREQ, PWM_RESOLUTION);
  ledcAttachPin(FAN_PIN, PWM_CHANNEL);
  
  xTaskCreatePinnedToCore(
                    &tmp32_read,           /* Task function. */
                    "temp32_read_task",    /* name of task. */
                    2048,                  /* Stack size of task */
                    NULL,                  /* parameter of the task */
                    5,                     /* priority of the task */
                    NULL,                  /* Task handle to keep track of created task */
                    0);                    /* pin task to core 0 */      
}





