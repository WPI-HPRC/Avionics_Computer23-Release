#include <Arduino.h>

#include <GroundStation/TelemetryBoard.h>

#include <EmbeddedSystems/IMU.h>
#include <EmbeddedSystems/MS5611.h>

IMU imu(Wire, 0x68);
MS5611 barometer;

TelemetryBoard telemBoard = TelemetryBoard();

double referencePressure;

void setup() {
  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000);

  telemBoard.initTelem();
  telemBoard.setState(HIGH_POWER);

  if(telemBoard.getState() == LOW_POWER || telemBoard.getState() == HIGH_POWER) {
    imu.setup();
    barometer.begin();
    referencePressure = barometer.readPressure();
  }
}

void loop() {

  if(telemBoard.getState() == LOW_POWER || telemBoard.getState() == HIGH_POWER) {
    imu.readSensor();

    // Vector3<int16_t> accelVector = imu.getAccelVals();
    // Vector3<int16_t> gyroVector = imu.getGyroVals();

    TelemetryBoard::RocketPacket newRocketPacket;
    // newRocketPacket.accelX = accelVector.x;
    // newRocketPacket.accelY = accelVector.y;
    // newRocketPacket.accelZ = accelVector.z;
    
    // newRocketPacket.gyroX = gyroVector.x;
    // newRocketPacket.gyroY = gyroVector.y;
    // newRocketPacket.gyroZ = gyroVector.z;

    newRocketPacket.state = 0;

    float temp = barometer.readTemperature();
    float pressure = barometer.readPressure();

    float absAlt = barometer.getAltitude(pressure);
    // float relAlt = barometer.getAltitude(pressure, referencePressure;

    newRocketPacket.altitude = absAlt; // m
    newRocketPacket.temperature = temp; // C
    newRocketPacket.pressure = pressure; // Pa

    newRocketPacket.timestamp = millis();

    telemBoard.setRocketPacket(newRocketPacket);
  }

  telemBoard.onLoop();

  delay(100);
}