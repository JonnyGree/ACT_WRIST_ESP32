#include "tmp32.h"

float  LastTemperature = 0.0;             // Degree"
char buffer [80];

// Function for reading TMP32 DATA using ADC, and scaling the result
void ReadTemperature(){ 
  uint32_t value = analogRead(TMP39_PIN);
  uint32_t voltage = analogReadMilliVolts(TMP39_PIN); 
  data.temperature = (float(voltage) - 500.0)/10.0 ;  //converting from 10 mv per degree wit 500 mV offset 

    #if TMP32_DEBUG   
      sprintf(buffer,"TMP32 Reading raw value: %d MilliVolts %d Scaled value %f ", value, voltage, data.temperature);
      Serial.println(buffer);                             
    #endif 
}

// Set Fan speed /  range 0 - 100
void SetFan(uint32_t _FanSpeed){
  ledcWrite(FAN_PWM_CHANNEL, 255*_FanSpeed/100);
  data.fanSpeed = _FanSpeed;

  #if TMP32_DEBUG   
    sprintf(buffer,"Fan Speed raw: %d scaled %d ", ledcRead(FAN_PWM_CHANNEL), data.fanSpeed);  
    Serial.println(buffer);                           
  #endif 

}


void tmp32_read( void * parameters ){    
  while(1){
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    ReadTemperature();
    if (abs(data.temperature - LastTemperature) > TEMPERATURE_DEADBAND) { 
      LastTemperature =  data.temperature;
      //Low duty cycle can result in no start
      //Set full duty cycle for 1S for ensure starting of the fan
      SetFan(100);                          
      vTaskDelay(1000 / portTICK_PERIOD_MS); 

      if (LastTemperature > 32.0)       { SetFan(100); }
      else if (LastTemperature > 29.0)  { SetFan(80); }
      else if (LastTemperature > 26.0)  { SetFan(60); }
      else                              { SetFan(0); }
      }   
  }  
}


void tmp32_init(){
  //Start fan on init 
  SetFan(100);
  //Create task for read temperature and set fan accordingly
  xTaskCreatePinnedToCore(
                    tmp32_read,           /* Task function. */
                    "TEM32_READ_TASK",    /* name of task. */
                    2048,                  /* Stack size of task */
                    NULL,                  /* parameter of the task */
                    1,                     /* priority of the task */
                    NULL,                  /* Task handle to keep track of created task */
                    0);                    /* pin task to core 0 */      
}





