#include <ESP8266WiFi.h>
#include <espnow.h>
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <CrsfSerial.h>  // https://github.com/CapnBry/CRServoF/
//#include <SerialTransfer.h>
//#include <Wire.h>

#define SwSerialBaud 115200 //Baudrate in bauds

// REPLACE WITH RECEIVER MAC Address
uint8_t broadcastAddress1[] = {0xBC, 0xDD, 0xC2, 0x30, 0xC4, 0xEB};
uint8_t broadcastAddress2[] = {0x40, 0xF5 ,0x20, 0x33, 0x11, 0xA4};
//5C:CF:7F:A2:BD:D8 - sbus receiver
uint8_t broadcastAddress3[] = {0x5C, 0xCF, 0x7F, 0xA2, 0xBD, 0xD8};

// Structure example to send data
// Must match the receiver structure
typedef struct data {
   byte RCH1;
  byte RCH2;
  byte RCH3;
  byte RCH4;
  byte RCH5;
  byte RCH6;
  byte RCH7;
  byte RCH8;
} data;

// Create a struct_message called tx
data tx;

SoftwareSerial DataSerial(5, 4);
SerialTransfer DataTransfer;

unsigned long lastTime = 0;  
unsigned long timerDelay = 20;  // send readings timer - 10ms for 100hz

void esp_now_send_function() {
  // Send message via ESP-NOW
   
    //esp_now_send(broadcastAddress1, (uint8_t *) &tx, sizeof(tx));
    //esp_now_send(broadcastAddress2, (uint8_t *) &tx, sizeof(tx));
    //esp_now_send(broadcastAddress3, (uint8_t *) &tx, sizeof(tx));
    esp_now_send(NULL, (uint8_t*) &tx, sizeof(tx)); //NULL instead of MAC address to send to all peers
    Serial.print("ch2:");
    Serial.println(tx.RCH2);
}

// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
 //Serial.print("Send status:");
  if (sendStatus == 0){
    Serial.println("success");
  }
  else{
   Serial.println("fail");
  }
}

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  DataSerial.begin(SwSerialBaud);
  DataTransfer.begin(DataSerial);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
 
  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  //esp_now_add_peer(broadcastAddress1, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
  esp_now_add_peer(broadcastAddress1, ESP_NOW_ROLE_SLAVE, 255, NULL, 0);
  //esp_now_add_peer(broadcastAddress2, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
  esp_now_add_peer(broadcastAddress2, ESP_NOW_ROLE_SLAVE, 255, NULL, 0);
  //esp_now_add_peer(broadcastAddress3, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
  esp_now_add_peer(broadcastAddress3, ESP_NOW_ROLE_SLAVE, 255, NULL, 0);
  tx.RCH1 = 127;      //Throttle (channel 2) to 0
  tx.RCH2 = 0;
  tx.RCH3 = 127;
  tx.RCH4 = 127;
  tx.RCH5 = 0;
  tx.RCH6 = 0;
  tx.RCH7 = 0;
  tx.RCH8 = 0;
  
}
 
void loop() {
   if(DataTransfer.available()) {
    // use this variable to keep track of how many
    // bytes we've processed from the receive buffer
    uint16_t recSize = 0;

    recSize = DataTransfer.rxObj(tx, recSize);
  }

  if ((millis() - lastTime) > timerDelay) {
    esp_now_send_function();
    lastTime = millis();
  } 

}

