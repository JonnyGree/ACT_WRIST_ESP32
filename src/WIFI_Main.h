#ifndef WIFI_Main_h
#define WIFI_Main_h

#include <esp_now.h>
#include <WiFi.h>
#include "Parameters.h"
#include "StepperControl.h"

    #define WIFI_DEBUG_ON (1)

    //Master_board Mac
    extern uint8_t Master_board[]; // chip 3 Master_board  

    // Structure example to receive data
    // Must match the sender structure
    typedef struct struct_message {
    int Act_Numb;
    char Mode;
    float Position;
    float Val_1;
    float Val_2;
    float Val_3;
    } struct_message;

    // Create a struct_message called myData
    extern struct_message DataIn;
    extern struct_message DataOut;

    // callback when data is sent
    void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);

    // callback function that will be executed when data is received
    void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);

    extern esp_now_peer_info_t peerInfoMaster;

   void IRAM_ATTR TransmittData();

    void WIFI_init(void);
    void SendToMaster();


#endif