#include "heltec.h"
#include <Crypto.h>
#include <AESLib.h>

#define BAND       868E6 //Lora band frequency
#define SF         7     //Spreading factor
#define CR         4     //Coding rate
#define DA         0x0016c001ff15eb23 // Set the destination address for the packet (the address of the ground station)
#define UAV_ID     "u1"   //Iroha blockchain ID
#define ACK_RATIO  8     // how many packets are sent before a ACK packet is expected
#define PK_FREQ    1000  // how long to wait between packets in milliseconds
#define TIMEOUT    10000  // how long to wait for a ACK packet

byte SendPacketCounter = 0;
byte ReceivePacketCounter = 1;
byte lastReceivePacketCounter = 0;
long lastRecvTime = 0;  // The last time a packet was recived to check when ack has been missed
long trackSendTime = 0; // Used to measure how long a packet took to make + send and adjust the delay between packets accordingly
bool WAITING_FOR_ACK = false;

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
  LoRa.enableCrc();

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
  Serial.print("Sending Telemetry Packet No.");
  Serial.println(SendPacketCounter);
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
  //Serial.print("Plaintext: ");
  //Serial.println(json_data);

  aesLib.set_paddingmode(paddingMode::ZeroLength); 

  // Encrypt Data
  byte enc_iv[N_BLOCK] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // iv_block gets written to, provide own fresh copy...
  String encrypted = encrypt_impl((char*)json_data.c_str(), enc_iv);
  //Serial.print("Base64 encoded Ciphertext: ");
  //Serial.println(encrypted);

  // // Decrypt Data
  // byte dec_iv[N_BLOCK] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // iv_block gets written to, provide own fresh copy...
  // String decrypted = decrypt_impl((char*)encrypted.c_str(), dec_iv);
  // //Serial.print("Base64-decoded Cleartext: ");
  // //Serial.println(decrypted);

  LoRa.beginPacket();
  LoRa.write(DA);
  LoRa.print(UAV_ID);
  LoRa.print(encrypted);
  LoRa.endPacket();
}

void onReceive(int packetSize){
  // if there's no packet, return
  if (packetSize == 0){return;}
  

  Serial.println("Receiving Packet!");
  adjustLoRaParams(LoRa.packetRssi());
  Serial.println(LoRa.readString());
  
  Serial.print("Waited for ");
  Serial.print(millis() - lastRecvTime);
  Serial.println(" milliseconds");

  ReceivePacketCounter++;
  SendPacketCounter = 0;
  lastRecvTime = 0;
  WAITING_FOR_ACK = false;
  sendPacketACK();
}

//Not sure if I will keep this, good for testing
void sendPacketACK() {
  Serial.println("Sending ACK Packet!");
  LoRa.beginPacket();
  LoRa.write(DA);
  LoRa.print(UAV_ID);
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
  Serial.print("RSSI: ");
  Serial.print(rssi);
  Serial.print(", Updating LoRa params: ");
  if (rssi > -70) {
    Serial.println("SF: 7, BW = 125");
    LoRa.setSpreadingFactor(7);
    LoRa.setSignalBandwidth(125E3);
  } else if (rssi > -80) {
    Serial.println("SF: 9, BW = 125");
    LoRa.setSpreadingFactor(9);
    LoRa.setSignalBandwidth(125E3);
  } else if (rssi > -90) {
    Serial.println("SF: 10, BW = 250");
    LoRa.setSpreadingFactor(10);
    LoRa.setSignalBandwidth(250E3);
  } else {
    Serial.println("SF: 12, BW = 250");
    LoRa.setSpreadingFactor(12);
    LoRa.setSignalBandwidth(250E3);
  }
}


void loop() {
  if (SendPacketCounter < ACK_RATIO)
  {
    trackSendTime = millis();
    SendPacketCounter++;
    sendPacket(getTelemetry());
    Serial.println(PK_FREQ - (millis() - trackSendTime));
    if (SendPacketCounter != 8){
      delay(PK_FREQ - (millis() - trackSendTime));
    }
    trackSendTime = 0;
  }
  else if (SendPacketCounter == ACK_RATIO && !WAITING_FOR_ACK){
    Serial.println("Now waiting for ACK");
    WAITING_FOR_ACK = true;
    lastRecvTime = millis();
  }
  else if (WAITING_FOR_ACK && millis() - lastRecvTime > TIMEOUT){
    Serial.println("ACK likely missed :(");
    SendPacketCounter = 0;
    WAITING_FOR_ACK = false;
  }
  else if(WAITING_FOR_ACK){
    onReceive(LoRa.parsePacket());
  }
}
