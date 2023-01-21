#include <Arduino.h>

#include <GroundStation/TelemetryBoard.h>

#include <EmbeddedSystems/IMU.h>
#include <EmbeddedSystems/Barometer.h>

IMU imu(Wire, 0x68);

TelemetryBoard telemBoard = TelemetryBoard();

void setup() {
  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000);

  telemBoard.initTelem();
  telemBoard.setState(HIGH_POWER);

  imu.setup();
}

void loop() {
  imu.readSensor();

  Vector3<int16_t> accelVector = imu.getAccelVals();

  TelemetryBoard::RocketPacket newRocketPacket;
  newRocketPacket.accelX = accelVector.x;
  newRocketPacket.accelY = accelVector.y;
  newRocketPacket.accelZ = accelVector.z;

  newRocketPacket.state = 0;
  newRocketPacket.altitude = 10;
  newRocketPacket.temperature = 25;
  newRocketPacket.timestamp = millis();

  telemBoard.setRocketPacket(newRocketPacket);

  telemBoard.onLoop();

  // Serial.print("Pressure: ");
  // Serial.println(pressure);
  // Serial.print("Temp: ");
  // Serial.println(temp);

  delay(10);
}