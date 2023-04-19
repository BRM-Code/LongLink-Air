#include "heltec.h"
#include <Crypto.h>
#include <AESLib.h>

#define BAND    868E6 //Lora band frequency
#define SF      7     //Spreading factor
#define CR      4    //Coding rate
uint64_t DEST_ADDRESS = 0x0016c001ff15eb23; // Set the destination address for the packet (the address of the ground station)
byte key[16]={0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
char UAV_ID[] = "u1"; //Iroha blockchain ID
byte data[53];
byte SendPacketCounter = 0;
byte ReceivePacketCounter = 1;
byte lastReceivePacketCounter = 0;
long lastSendTime = 0;
boolean send = false;

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
  int8_t pitch;
  int8_t roll;
  int8_t heading;
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
  char decrypted[msgLen] = {0}; // half may be enough
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
  Serial.println("Sending Telemetry Packet!");
  String json_data = " " + String(telemetry.latitude, 4) +
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
                   " " + String(telemetry.arm ? 1 : 0) + String(telemetry.sat_fix ? 1 : 0);
  Serial.print("Plaintext: ");
  Serial.println(json_data);

  aesLib.set_paddingmode(paddingMode::ZeroLength); 

  // Encrypt Data
  byte enc_iv[N_BLOCK] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // iv_block gets written to, provide own fresh copy...
  String encrypted = encrypt_impl((char*)json_data.c_str(), enc_iv);
  // sprintf(ciphertext, "%s", encrypted.c_str());
  Serial.print("Base64 encoded Ciphertext: ");
  Serial.println(encrypted);

  // Decrypt Data
  byte dec_iv[N_BLOCK] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // iv_block gets written to, provide own fresh copy...
  String decrypted = decrypt_impl((char*)encrypted.c_str(), dec_iv);
  Serial.print("Base64-decoded Cleartext: ");
  Serial.println(decrypted);

  LoRa.beginPacket();
  LoRa.write(DEST_ADDRESS);
  LoRa.print(UAV_ID);
  LoRa.print(encrypted);
  LoRa.endPacket();
}

void onReceive(int packetSize)
{
  if (packetSize == 0){
    return;   
  }        // if there's no packet, return

  Serial.println("Receiving Packet!");
  while (LoRa.available()) {
    Serial.println(LoRa.read());
  }
  ReceivePacketCounter++;
  sendPacketACK();
}

void sendPacketACK() {
  Serial.println("Sending ACK Packet!");
  LoRa.beginPacket();
  LoRa.write(DEST_ADDRESS);
  LoRa.write(UAV_ID[0]);
  LoRa.write(UAV_ID[1]);
  LoRa.endPacket();
}

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


void loop() {
  if (!send)
  {
    sendPacket(getTelemetry());
    SendPacketCounter++;
    lastSendTime = millis();
    lastReceivePacketCounter = ReceivePacketCounter;
    send = true;
  }
  else{
    onReceive(LoRa.parsePacket());
  }
}
