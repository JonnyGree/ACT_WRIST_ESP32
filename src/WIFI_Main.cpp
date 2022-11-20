#include "WIFI_Main.h"

//Send Data Timer
const int DataTimeout = 5000;  //time in ms to trigger the watchdog
hw_timer_t *timer1 = NULL;

void IRAM_ATTR TransmittData() {
                DataOut.Position = (float)1*360.0/1;    //Motor Pulse
                DataOut.Val_1 = 1;                      //speed
                DataOut.Val_2 = 1;                      //Motor Temperature
                DataOut.Val_3 = (float)1*5.0;           //Fan Speed
                SendToMaster();
}

//Master_board Mac
uint8_t Master_board[] = {0x4C, 0x11, 0xAE, 0x74, 0xB3, 0xDC}; // chip 3 Master_board  

// Create a struct_message called myData
struct_message DataIn;
struct_message DataOut;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
     #if WIFI_DEBUG_ON
      Serial.print("\r\nLast Packet Send Status:\t");
      Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
     #endif
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&DataIn, incomingData, sizeof(DataIn));
       #if WIFI_DEBUG_ON
        Serial.print("Mode: ");Serial.println(DataIn.Mode);
        Serial.print("Pos: ");Serial.println(DataIn.Position);
        Serial.print("Val1: ");Serial.println(DataIn.Val_1);
        Serial.print("Val2: ");Serial.println(DataIn.Val_2);
        Serial.print("Val3: ");Serial.println(DataIn.Val_3);
        #endif
          switch (DataIn.Mode) {
            
            case 'A':   //Move Absolute
              {
                motor.mode = motor_mode::MOVE_ABS;
                motor.Position= DataIn.Position;               
                motor.Execute = true;
               // MoveAbs(DataIn.Position);
                break;
              }
             case 'R':  //Move relative
              {
                motor.mode = motor_mode::MOVE_REL;
                motor.Position= DataIn.Position;               
                motor.Execute = true;
                //MoveRel(DataIn.Position);
                break;
              }
              case 'S':   //Set % velocity, acc, dec --> Max 100
              {
                motor.mode = motor_mode::SET_SPEED;
                motor.vel = DataIn.Val_1;
                motor.acc = DataIn.Val_1;
                motor.dec = DataIn.Val_1;
                motor.Execute = true;
                break;
              }
              case 'M':   //Set microsteps
              {
                motor.microsteps = DataIn.Val_1;
                break;
              }
              case 'P':   //Set current position
              {
                motor.mode = motor_mode::SET_POS;
                motor.Position = DataIn.Position;
                motor.Execute = true;
                break;
              }
              case 'O':   //OFF --> save position to EEprom
              {
                motor.mode = motor_mode::TURN_OFF;
                motor.Execute = true;
                break;
              }
              case 'K' :  //Start Data Transmission
              {
                  timer1 = timerBegin(TIMER_WIFI, 80, true);           //timer 0, div 80
                  timerAttachInterrupt(timer1, &TransmittData, true);  //attach callback
                  timerAlarmWrite(timer1, DataTimeout * 1000, true);   //set time in us
                  timerAlarmEnable(timer1);
                break;
              }
              case 'L' :
              {
                 timerEnd(timer1);
                 timer1 = NULL;  
                break;
              }
            default:
              {}
              break;
          }
}


//----------------------------SETUP------------------------//
esp_now_peer_info_t peerInfoMaster;

void WIFI_init(void){
   btStop();
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  #if WIFI_DEBUG_ON 
    Serial.print("STA MAC: "); 
    Serial.println(WiFi.macAddress());
  #endif

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
    
  memcpy(peerInfoMaster.peer_addr, Master_board, 6);
  peerInfoMaster.channel = 3;  
  peerInfoMaster.encrypt = false;
  // Add peer        
  if (esp_now_add_peer(&peerInfoMaster) != ESP_OK){
    Serial.println("Failed to add peer Master");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  timer1 = timerBegin(1, 80, true);                  //timer 0, div 80
  timerAttachInterrupt(timer1, &TransmittData, true);  //attach callback
  timerAlarmWrite(timer1, DataTimeout * 1000, true); //set time in us
  //timerAlarmEnable(timer1);


Serial.println("WIFI READY");
}

  void SendToMaster(){
  esp_err_t result = esp_now_send(Master_board, (uint8_t *) &DataOut, sizeof(DataOut));   
      #if WIFI_DEBUG_ON
      if (result == ESP_OK) {
        Serial.println("Sent with success");
      }
      else {
        Serial.println("Error sending the data");
      } 
    #endif
}
