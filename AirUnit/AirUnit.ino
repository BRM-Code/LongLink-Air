#include "heltec.h"

// Set LoRa frequency and spreading factor
#define BAND    868E6
#define SF      9

// Set the destination address for the packet (the address of the ground station)
#define DEST_ADDRESS 0x0016c001ff15eb23

int packetCounter = 0;

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

void setup() {
  // Setup LoRa module
  Heltec.begin(true /*DisplayEnable Enable*/, true /*Heltec.Heltec.Heltec.LoRa Disable*/, true /*Serial Enable*/, true /*PABOOST Enable*/, BAND /*long BAND*/);
  LoRa.setSpreadingFactor(SF);

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
  delay(1500);
}

void sendPacket(TelemetryData telemetry) {
  LoRa.beginPacket();
  LoRa.write(DEST_ADDRESS);
  LoRa.print(String(telemetry.latitude, 4));
  LoRa.print("," + String(telemetry.longitude, 4));
  LoRa.print("," + String(telemetry.vbatt));
  LoRa.print("," + String(telemetry.altitude));
  LoRa.print("," + String(telemetry.ground_speed));
  LoRa.print("," + String(telemetry.satellites));
  LoRa.print("," + String(telemetry.consumption));
  LoRa.print("," + String(telemetry.rssi));
  LoRa.print("," + String(telemetry.pitch));
  LoRa.print("," + String(telemetry.roll));
  LoRa.print("," + String(telemetry.heading));
  LoRa.print("," + String(telemetry.arm ? 1 : 0));
  LoRa.print(telemetry.sat_fix ? 1 : 0);
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
  packetCounter += 1;

  sendPacket(getTelemetry());

  Heltec.display->clear();
  Heltec.display->drawString(0, 0, "LongLink-Air Unit");
  Heltec.display->drawString(0, 12, String(packetCounter));
  Heltec.display->display();

  delay(5000);
}
