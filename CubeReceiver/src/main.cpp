#include <Arduino.h>

#include <LoRaLib.h>

#define LORA_CS 4

struct CubeStruct {
  String name;
  uint32_t timestamp; 
  float temp;
  float humidity;
  float pressure;
};

SX1276 lora = new LoRa(LORA_CS);

uint8_t packetLen = sizeof(CubeStruct);

void setup() {
  Serial.begin(9600);

  while(!Serial);

  int state = lora.begin(915, 500, 12, 5, 0x13);

  if(state == ERR_NONE) {
        Serial.println(F("LoRa Success"));
    } else {
        Serial.print(F("LoRa Failed: "));
        Serial.println(state);
    }
}

void loop() {
  uint8_t * rxPacket;
  int state = lora.receive(rxPacket, packetLen);

  if(state == ERR_NONE) {
    CubeStruct * cubeData = (CubeStruct *)rxPacket;
    Serial.print(F("Name: "));
    Serial.println(cubeData->name);
    Serial.print(F("Timestamp: "));
    Serial.println(cubeData->timestamp);
    Serial.print(F("Temperature: "));
    Serial.println(cubeData->temp);
    Serial.print(F("Humidity: "));
    Serial.println(cubeData->humidity);
    Serial.print(F("Pressure: "));
    Serial.println(cubeData->pressure);
  } else {
    Serial.print(F("Receive Failed: "));
    Serial.println(state);
  }
}