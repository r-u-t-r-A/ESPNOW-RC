#include <Arduino.h>
#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <espnow.h>

SoftwareSerial SBUS;

// Must match the sender structure
typedef struct received_data {
  byte RCH1;
  byte RCH2;
  byte RCH3;
  byte RCH4;
  byte RCH5;
  byte RCH6;
  byte RCH7;
  byte RCH8;
} received_data;

// Create a struct_message called rx
received_data rx;

#define DEBUG //define if you want debug data out on hardware serial - IMPACTS PERFORMANCE!

#define RC_CHANNEL_MIN 0
#define RC_CHANNEL_MAX 255

#define SBUS_MIN_OFFSET 173
#define SBUS_MID_OFFSET 992
#define SBUS_MAX_OFFSET 1811
#define SBUS_CHANNEL_NUMBER 16
#define SBUS_PACKET_LENGTH 25
#define SBUS_FRAME_HEADER 0x0f
#define SBUS_FRAME_FOOTER 0x00
#define SBUS_FRAME_FOOTER_V2 0x04
#define SBUS_STATE_FAILSAFE 0x08
#define SBUS_STATE_SIGNALLOSS 0x04
#define SBUS_UPDATE_RATE 15 //ms

unsigned long lastRecvTime = 0;
unsigned long TimeSinceLastPacket = 0; // You can setup AUX12 in betaflight as rssi to monitor packet time
unsigned long mseconds = 0;
uint8_t sbusPacket[SBUS_PACKET_LENGTH];
int rcChannels[SBUS_CHANNEL_NUMBER];
uint32_t sbusTime = 0;

// Callback function that will be executed when data is received
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&rx, incomingData, sizeof(rx));
  mseconds = millis();
  
  TimeSinceLastPacket = mseconds - lastRecvTime;
  lastRecvTime = millis();
  
}

void sbusPreparePacket(uint8_t packet[], int channels[], bool isSignalLoss, bool isFailsafe){

    static int output[SBUS_CHANNEL_NUMBER] = {0};

    /*
     * Map 0-255 with middle at 127 chanel values to
     * 173-1811 with middle at 992 S.BUS protocol requires
     */
    for (uint8_t i = 0; i < SBUS_CHANNEL_NUMBER; i++) {
      if (i <= 3) {
        output[i] = map(rcChannels[i], RC_CHANNEL_MIN, RC_CHANNEL_MAX, SBUS_MIN_OFFSET, SBUS_MAX_OFFSET);
      } else if (i > 3 && i < 15 ) {
        output[i] = map(rcChannels[i], RC_CHANNEL_MIN, 1, SBUS_MIN_OFFSET, SBUS_MAX_OFFSET);
      } else {
        output[i] = map(rcChannels[i], 100, 0, SBUS_MIN_OFFSET, SBUS_MAX_OFFSET);
      }
    }

    uint8_t stateByte = 0x00;
    if (isSignalLoss) {
        stateByte |= SBUS_STATE_SIGNALLOSS;
    }
    if (isFailsafe) {
        stateByte |= SBUS_STATE_FAILSAFE;
    }
    packet[0] = SBUS_FRAME_HEADER; //Header

    packet[1] = (uint8_t) (output[0] & 0x07FF);
    packet[2] = (uint8_t) ((output[0] & 0x07FF)>>8 | (output[1] & 0x07FF)<<3);
    packet[3] = (uint8_t) ((output[1] & 0x07FF)>>5 | (output[2] & 0x07FF)<<6);
    packet[4] = (uint8_t) ((output[2] & 0x07FF)>>2);
    packet[5] = (uint8_t) ((output[2] & 0x07FF)>>10 | (output[3] & 0x07FF)<<1);
    packet[6] = (uint8_t) ((output[3] & 0x07FF)>>7 | (output[4] & 0x07FF)<<4);
    packet[7] = (uint8_t) ((output[4] & 0x07FF)>>4 | (output[5] & 0x07FF)<<7);
    packet[8] = (uint8_t) ((output[5] & 0x07FF)>>1);
    packet[9] = (uint8_t) ((output[5] & 0x07FF)>>9 | (output[6] & 0x07FF)<<2);
    packet[10] = (uint8_t) ((output[6] & 0x07FF)>>6 | (output[7] & 0x07FF)<<5);
    packet[11] = (uint8_t) ((output[7] & 0x07FF)>>3);
    packet[12] = (uint8_t) ((output[8] & 0x07FF));
    packet[13] = (uint8_t) ((output[8] & 0x07FF)>>8 | (output[9] & 0x07FF)<<3);
    packet[14] = (uint8_t) ((output[9] & 0x07FF)>>5 | (output[10] & 0x07FF)<<6);  
    packet[15] = (uint8_t) ((output[10] & 0x07FF)>>2);
    packet[16] = (uint8_t) ((output[10] & 0x07FF)>>10 | (output[11] & 0x07FF)<<1);
    packet[17] = (uint8_t) ((output[11] & 0x07FF)>>7 | (output[12] & 0x07FF)<<4);
    packet[18] = (uint8_t) ((output[12] & 0x07FF)>>4 | (output[13] & 0x07FF)<<7);
    packet[19] = (uint8_t) ((output[13] & 0x07FF)>>1);
    packet[20] = (uint8_t) ((output[13] & 0x07FF)>>9 | (output[14] & 0x07FF)<<2);
    packet[21] = (uint8_t) ((output[14] & 0x07FF)>>6 | (output[15] & 0x07FF)<<5);
    packet[22] = (uint8_t) ((output[15] & 0x07FF)>>3);

    packet[23] = stateByte; //Flags byte
    packet[24] = SBUS_FRAME_FOOTER; //Footer
}

void map_val() {
  
  rcChannels[0] = rx.RCH1;
  rcChannels[1] = rx.RCH2;
  rcChannels[2] = rx.RCH3;
  rcChannels[3] = rx.RCH4;
  rcChannels[4] = rx.RCH5;
  rcChannels[5] = rx.RCH6;
  rcChannels[6] = rx.RCH7;
  rcChannels[7] = rx.RCH8;
  rcChannels[8] = 0;
  rcChannels[9] = 0;
  rcChannels[10] = 0;
  rcChannels[11] = 0;
  rcChannels[12] = 0;
  rcChannels[13] = 0;
  rcChannels[14] = 0;
  rcChannels[15] = constrain(TimeSinceLastPacket, 0, 100);

}

void setup() {
  for (uint8_t i = 0; i < SBUS_CHANNEL_NUMBER; i++) {
      rcChannels[i] = 127;
  }
  SBUS.begin(100000, SWSERIAL_8E2, 4, 5, true);
  SBUS.enableTx(true);
  SBUS.enableRx(false);
  SBUS.enableIntTx(false);
 #if defined(DEBUG)
  // Initialize Serial Monitor
  Serial.begin(115200);
  Serial.println("start up'");
 #endif
  //Serial.begin(100000, SERIAL_8E2);
  // Set device as a Wi-Fi Station

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Init ESP-NOW
  if (esp_now_init() != 0) {
  //  Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);
  #if defined(DEBUG)
    Serial.println("Setup done");
    Serial.print("ESP8266 Board MAC Address:  ");
    Serial.println(WiFi.macAddress());
  #endif
 // delay(100);

  //Serial.begin(100000, SERIAL_8E2);
}

void loop() {
  
  //Reset data if signal is lost for 1 second
  unsigned long now = millis();
  if ( now - lastRecvTime > 1000 ) {
    // signal lost?
    rx.RCH1 = 127;      //Throttle (channel 2) to 0
    rx.RCH2 = 0;
    rx.RCH3 = 127;
    rx.RCH4 = 127;
    rx.RCH5 = 0;
    rx.RCH6 = 0;
    rx.RCH7 = 0;
    rx.RCH8 = 0;
    //Go up and change the initial values if you want depending on
    //your aplications. Put 0 for throttle in case of drones so it won't
    //fly away
  } 
  map_val();

  if (now > sbusTime) {
    sbusPreparePacket(sbusPacket, rcChannels, false, false);
    SBUS.write(sbusPacket, SBUS_PACKET_LENGTH);
    #if defined(DEBUG)
      Serial.println("transmitting sbus");
      Serial.print(rcChannels[1]);
      Serial.print(" | ");
      Serial.print(rcChannels[4]);
      Serial.println("");
      Serial.print("Time since last packet:");
      Serial.println(TimeSinceLastPacket);
    #endif

    sbusTime = now + SBUS_UPDATE_RATE;
  }
}
