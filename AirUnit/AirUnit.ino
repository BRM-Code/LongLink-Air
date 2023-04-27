#include "heltec.h"
#include <Crypto.h>
#include <AESLib.h>

#define BAND       868E6 //Lora band frequency
#define UAV_ID     "u1"   //Iroha blockchain ID
#define DA         0x0016c001ff15eb23 // Set the destination address for the packet (the address of the ground station)
#define ACK_RATIO  8     // how many packets are sent before a ACK packet is expected
#define PK_FREQ    1000  // how long to wait between packets in milliseconds
#define TIMEOUT    1000  // how long to wait for a ACK packet
#define SF_MAX     10
#define SF_MIN     7
#define SF         7     //Spreading factor
#define CR         4     //Coding rate
#define MIN_RSSI   -130
#define MAX_RSSI   -60
#define ACK_NUM    7

int SendPacketCounter = 0;
long lastRecvTime = 0;  // The last time a packet was recived to check when ack has been missed
long trackSendTime = 0; // Used to measure how long a packet took to make + send and adjust the delay between packets accordingly
bool WAITING_FOR_ACK = false;

// Variables used soley for the display
byte currentSF = SF;
int last_RSSI = 0;
int totalSendPacketCounter = 0;
int recPacketCounter = 0;

AESLib aesLib;

byte aes_key[] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };
byte aes_iv[N_BLOCK] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };


struct TelemetryData {
  float latitude;
  float longitude;
  float vbatt;
  uint8_t altitude;
  uint8_t ground_speed;
  uint8_t satellites;
  uint8_t consumption;
  uint8_t rssi;
  int16_t pitch;
  int16_t roll;
  int16_t heading;
  bool arm;
  bool sat_fix;
};


String encrypt_impl(char * msg, byte iv[]) {
  int msgLen = strlen(msg);
  char encrypted[2 * msgLen] = {0};
  aesLib.encrypt64((const byte*)msg, msgLen, encrypted, aes_key, sizeof(aes_key), iv);
  return String(encrypted);
}

String decrypt_impl(char * msg, byte iv[]) {
  int msgLen = strlen(msg);
  char decrypted[msgLen] = {0};
  aesLib.decrypt64(msg, msgLen, (byte*)decrypted, aes_key, sizeof(aes_key), iv);
  return String(decrypted);
}

void setup() {
  // Setup LoRa module
  Heltec.begin(true /*DisplayEnable Enable*/, true /*Heltec.Heltec.Heltec.LoRa Disable*/, true /*Serial Enable*/, true /*PABOOST Enable*/, BAND /*long BAND*/);
  LoRa.setSpreadingFactor(SF);
  LoRa.setCodingRate4(CR);
  LoRa.setPreambleLength(8);
  LoRa.setFrequency(867500000);
  LoRa.setSignalBandwidth(125E3);
  LoRa.enableCrc();
  LoRa.setSyncWord(0x12);

  // Setup serial communication
  Serial.begin(115200);
  while (!Serial);

  // Setup display
  Heltec.display->init();
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->clear();

  // Display setup message
  Heltec.display->drawString(0, 0, "LongLink-Air Unit");
  Heltec.display->display();
}

void sendPacket(TelemetryData telemetry) {
  String json_data = String(telemetry.latitude, 4) +
                   " " + String(telemetry.longitude, 4) +
                   " " + String(telemetry.vbatt) +
                   " " + String(telemetry.altitude) +
                   " " + String(telemetry.ground_speed) +
                   " " + String(telemetry.satellites) +
                   " " + String(telemetry.consumption) +
                   " " + String(telemetry.rssi) +
                   " " + String(telemetry.pitch) +
                   " " + String(telemetry.roll) +
                   " " + String(telemetry.heading) +
                   " " + String(telemetry.arm ? 1 : 0) + 
                   String(telemetry.sat_fix ? 1 : 0) + 
                   String(SendPacketCounter);

  aesLib.set_paddingmode(paddingMode::ZeroLength); 

  // Encrypt Data
  byte enc_iv[N_BLOCK] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // iv_block gets written to, provide own fresh copy...
  String encrypted = encrypt_impl((char*)json_data.c_str(), enc_iv);

  LoRa.beginPacket();
  LoRa.write(DA);
  LoRa.print(UAV_ID);
  LoRa.print(encrypted);
  LoRa.endPacket();
}

void onReceive(int packetSize){
  // if there's no packet, return
  if (packetSize == 0){return;}
  recPacketCounter++;

  Serial.print("[<-PK] : ");
  Serial.println(LoRa.readString());
  
  Serial.print("Waited for ");
  Serial.print(millis() - lastRecvTime);
  Serial.println(" ms");

  SendPacketCounter = 0;
  WAITING_FOR_ACK = false;
  last_RSSI = LoRa.packetRssi();
  adjustLoRaParams(LoRa.packetRssi());
  sendPacketACK(last_RSSI);
  LoRa.sleep();
  delay(PK_FREQ - (millis() - lastRecvTime)*2);
  LoRa.idle();
  lastRecvTime = 0;
}

void sendPacketACK(int RSSI) {
  Serial.println("[->PK] ACK");
  LoRa.beginPacket();
  LoRa.write(DA);
  LoRa.print(UAV_ID);
  LoRa.print(" ");
  LoRa.print(ACK_NUM);
  LoRa.print(" ");
  LoRa.print(ACK_RATIO);
  LoRa.print(" ");
  LoRa.print(RSSI);
  LoRa.endPacket();
}

// For testing, these are just fixed values
TelemetryData getTelemetry(){
  TelemetryData telemetry;
  telemetry.latitude = 37.7749;
  telemetry.longitude = -122.4194;
  telemetry.vbatt = 9.0;
  telemetry.altitude = 200;
  telemetry.ground_speed = 50;
  telemetry.satellites = 8;
  telemetry.consumption = 80;
  telemetry.rssi = 50;
  telemetry.pitch = 10;
  telemetry.roll = 20;
  telemetry.heading = 30;
  telemetry.arm = true;
  telemetry.sat_fix = true;
  return telemetry;
}

void adjustLoRaParams(int rssi) {
  //Keep track of what SF used to be to see if it needs changing
  byte oldSF = currentSF;

  //Adjusting Spread Factor based on RSSI
  if(MIN_RSSI > rssi) currentSF++;
  else if(MAX_RSSI <= rssi) currentSF--;

  //Checking the SF hasn't gone out of range
  if(currentSF >= SF_MAX) currentSF = SF_MAX;
  else if (currentSF < SF_MIN) currentSF = SF_MIN;

  if (oldSF != currentSF){
    LoRa.setSpreadingFactor(currentSF);
    Serial.print("RSSI: " + String(rssi) + ", Updating SF: " + String(currentSF));
  }
}

void packetSleep(){
  Serial.println("Sleeping for " + String(PK_FREQ - trackSendTime));
    LoRa.sleep();
    delay(PK_FREQ - trackSendTime);
    LoRa.idle();
}

void loop() {
  LoRa.setSpreadingFactor(currentSF);
  if (SendPacketCounter < ACK_RATIO)
  {
    trackSendTime = millis();
    SendPacketCounter++;
    totalSendPacketCounter++;
    sendPacket(getTelemetry());
    trackSendTime = millis() - trackSendTime;
    Serial.print("[PK->] No." + String(SendPacketCounter));
    Serial.println(" took: " + String(trackSendTime) + "ms");

    if (SendPacketCounter != ACK_RATIO && trackSendTime < PK_FREQ){
      packetSleep();
    }
  }
  else if (SendPacketCounter == ACK_RATIO && !WAITING_FOR_ACK){
    Serial.println("Now waiting for ACK");
    WAITING_FOR_ACK = true;
    lastRecvTime = millis();
  }
  else if (WAITING_FOR_ACK && millis() - lastRecvTime > TIMEOUT){
    Serial.println("ACK Timeout: " + String(millis() - lastRecvTime));
    SendPacketCounter = 0;
    WAITING_FOR_ACK = false;
    currentSF++;
    if(currentSF > SF_MAX) currentSF = SF_MAX;
    LoRa.setSpreadingFactor(currentSF);
  }
  else if(WAITING_FOR_ACK) onReceive(LoRa.parsePacket());

  Heltec.display->clear();
  Heltec.display->drawString(0, 0, "LongLink-Air Unit");
  Heltec.display->drawString(0, 12, "Sent: " + String(totalSendPacketCounter));
  Heltec.display->drawString(0, 24, "Recv: " + String(recPacketCounter) + "(" + String(totalSendPacketCounter/ACK_RATIO) + ")");

  // Display LoRa parameters
  Heltec.display->drawString(0, 36, "SF: " + String(currentSF));
  Heltec.display->drawString(64, 36, "CR: " + String(CR));
  Heltec.display->drawString(64, 48, "RSSI: " + String(last_RSSI));
  Heltec.display->drawString(0, 48, "Battery: " + String(analogRead(1)));

  Heltec.display->display();
}
