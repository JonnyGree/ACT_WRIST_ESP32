#include "Led_WS2812B.h"


CRGB leds[NUM_LEDS];

/*
  set color to 
  WHITE, YELLOW, RED, GREEN, BLACK
*/
typedef enum
{
    WHITE = 0,
    YELLOW = 1,
    RED = 2,
    GREEN = 3,
    BLACK = 4,
    COMMAND_DONE = 255,
} led_state;

led_state Color = BLACK;

int led_init_flash=0 ;

void led_task(void *pvParameter)
{
    while (1){
        if (act_state == EMERGENCY){        
            if (Color == RED){
                leds[0] = CRGB::Yellow;
                FastLED.show();
                Color = YELLOW;
                vTaskDelay(LED_FLASH_DELAYVAL / portTICK_PERIOD_MS);
            }
            else{
                leds[0] = CRGB::Red;
                FastLED.show();
                Color = RED;
                vTaskDelay(LED_FLASH_DELAYVAL / portTICK_PERIOD_MS);
            }
        }
        else if (act_state == WARNING){
            if (Color == BLACK){
                leds[0] = CRGB::Yellow;
                FastLED.show();
                Color = YELLOW;
                vTaskDelay(LED_FLASH_DELAYVAL / portTICK_PERIOD_MS);
            }
            else{
                leds[0] = CRGB::Black;
                FastLED.show();
                Color = BLACK;
                vTaskDelay(LED_FLASH_DELAYVAL / portTICK_PERIOD_MS);
            }
        }
        else if (act_state == STATUS_OK){
            if (act_mode == MODE_OPERATION_ENABLED && Color != RED ){
                leds[0] = CRGB::Red;
                FastLED.show();
                Color = RED;
            }
            if (act_mode == MODE_JOG ){
                if (Color == WHITE){
                    leds[0] = CRGB::Green;
                    FastLED.show();
                    Color = GREEN;
                    vTaskDelay(LED_FLASH_DELAYVAL / portTICK_PERIOD_MS);
                }
                else{
                    leds[0] = CRGB::White;
                    FastLED.show();
                    Color = WHITE;
                    vTaskDelay(LED_FLASH_DELAYVAL / portTICK_PERIOD_MS); 
                }
            }
            if (act_mode == MODE_READY && Color != GREEN ){
                leds[0] = CRGB::Green;
                FastLED.show();
                Color = GREEN;
            }
            if (act_mode == MODE_INIT ){
                if (Color == WHITE){
                    leds[0] = CRGB::Black;
                    FastLED.show();
                    Color = BLACK;
                    vTaskDelay(LED_FLASH_DELAYVAL / portTICK_PERIOD_MS);
                }
                else{
                    leds[0] = CRGB::White;
                    FastLED.show();
                    Color = WHITE;
                    vTaskDelay(LED_FLASH_DELAYVAL / portTICK_PERIOD_MS);
                    led_init_flash++;
                    if (led_init_flash ==5)   act_mode = MODE_READY;;                    
                }
            }
            if (act_mode == MODE_HOMING ){
                if (Color == WHITE){
                    leds[0] = CRGB::Black;
                    FastLED.show();
                    Color = BLACK;
                    vTaskDelay(LED_FLASH_DELAYVAL / portTICK_PERIOD_MS);
                }
                else{
                    leds[0] = CRGB::Green;
                    FastLED.show();
                    Color = GREEN;
                    vTaskDelay(LED_FLASH_DELAYVAL / portTICK_PERIOD_MS);
                }
            }

        }
        vTaskDelay(LED_TASK_DELAYVAL / portTICK_PERIOD_MS);
        #if LED_DEBUG_ON
            printf("act_state :%d , act_mode :%d, COLOR :%d", act_state, act_mode, Color);  
            printf("\n");
            vTaskDelay(500 / portTICK_PERIOD_MS);                 
        #endif
    }
}

void led_init(void)
{
    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);  // GRB ordering is typical
    xTaskCreatePinnedToCore(&led_task, "Led_WS2812B_Task", 4096, NULL, 5, NULL, 0);
}
