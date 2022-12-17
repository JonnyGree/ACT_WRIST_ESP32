#include "Led_WS2812B.h"
#include "FastLed.h"

CRGB leds[NUM_LEDS];

int led_init_flash=0 ;

void led_task(void *parameter)
{
    while (1){
    //------------------------STATE ALARM --------------------//
        if (data.state == ALARM){  
            data.ledColor = RED;      
            if (leds[0] == (CRGB)CRGB::DarkRed){
                leds[0] = CRGB::Red; FastLED.show();                
                vTaskDelay(LED_FLASH_DELAYVAL / portTICK_PERIOD_MS);
            }
            else{leds[0] = CRGB::DarkRed; FastLED.show();                
                vTaskDelay(LED_FLASH_DELAYVAL / portTICK_PERIOD_MS);
            }
        }
    //------------------------STATE WARNING --------------------//
        else if (data.state == WARNING){  
            data.ledColor = YELLOW;      
            if (leds[0] == (CRGB)CRGB::Yellow ){
                leds[0] = CRGB::LightYellow ; FastLED.show();                
                vTaskDelay(LED_FLASH_DELAYVAL / portTICK_PERIOD_MS);
            }
            else{leds[0] = CRGB::Yellow; FastLED.show();                
                vTaskDelay(LED_FLASH_DELAYVAL / portTICK_PERIOD_MS);
            }
        }
    //------------------------STATE OK --------------------//
        else if (data.state == STATUS_OK){
            if (data.mode == MODE_OPERATION_ENABLED && data.ledColor != RED ){
                leds[0] = CRGB::Red; FastLED.show(); data.ledColor = RED ;
            }
            else if (data.mode == MODE_JOG && data.ledColor != VIOLET ){
                leds[0] = CRGB::BlueViolet ; FastLED.show(); data.ledColor = VIOLET ;
            }
            else if (data.mode == MODE_READY && data.ledColor != GREEN ){
                leds[0] = CRGB::Green ; FastLED.show(); data.ledColor = GREEN ;
            }
            else if (data.mode == MODE_INIT ){
                data.ledColor = WHITE;      
                if (leds[0] == (CRGB)CRGB::White ){
                    leds[0] = CRGB::Black ; FastLED.show();                
                    vTaskDelay(LED_FLASH_DELAYVAL / portTICK_PERIOD_MS);
                }
                else{leds[0] = CRGB::White; FastLED.show();                
                vTaskDelay(LED_FLASH_DELAYVAL / portTICK_PERIOD_MS);
                led_init_flash++;
                if (led_init_flash ==5){ data.mode = MODE_READY;}   
                }
            }
            else if (data.mode == MODE_HOMING ){
                data.ledColor = BLUE;      
                if (leds[0] == (CRGB)CRGB::Blue  ){
                    leds[0] = CRGB::LightBlue  ; FastLED.show();                
                    vTaskDelay(LED_FLASH_DELAYVAL / portTICK_PERIOD_MS);
                }
                else{leds[0] = CRGB::Blue ; FastLED.show();                
                vTaskDelay(LED_FLASH_DELAYVAL / portTICK_PERIOD_MS);
                }
            }

        }
        vTaskDelay(LED_TASK_DELAYVAL / portTICK_PERIOD_MS);
        #if LED_DEBUG
            printf("act_state :%d , act_mode :%d, COLOR :%d", data.state, data.mode, data.ledColor);  
            printf("\n");
            vTaskDelay(500 / portTICK_PERIOD_MS);                 
        #endif
    }
}

void led_init(void)
{
    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);  // GRB ordering is typical
    FastLED.setBrightness(MAX_BRIGHTNESS);
    xTaskCreatePinnedToCore(&led_task, "Led_WS2812B_Task", 4096, NULL, 5, NULL, 0);
}
