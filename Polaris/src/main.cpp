#include <Arduino.h>

#include <GroundStation/TelemetryBoard.h>

//Peripherals
#include <Peripherials/MS5611.h>
#include <Peripherials/ICM42688.h>

TelemetryBoard * telemBoard = new TelemetryBoard(teensy);

RocketPacket currentRocketPacket;

MS5611 * barometer = new MS5611(0x77);
ICM42688 * imu = new ICM42688(Wire, 0x68);

void setup() {
  Serial.begin(115200);

  telemBoard->init();
  telemBoard->setState(RX);

  //Initialize Sensors
  if(barometer->begin()) {
    Serial.println("Successfully initialized barometer!");
  } else {
    Serial.println("Barometer Not Initialized!");
  }
  if(imu->begin() < 0) {
    Serial.println("IMU Not Initialized");
  }

  //Configure Barometer
  barometer->setOversampling(OSR_ULTRA_LOW);

  //Configure IMU
  imu->setAccelFS(ICM42688::gpm8); // Range: +/- 8G
  imu->setGyroFS(ICM42688::dps500); // Range: +/- 500 deg/s

  imu->setAccelODR(ICM42688::odr100); // Output Rate: 100Hz
  imu->setGyroODR(ICM42688::odr100); // Output Rate: 100Hz

}

void loop() {

    if(telemBoard->getState() == TX) {
      //Read Sensor Data
      barometer->read();
      imu->getAGT();

      float acX = imu->accX();
      float acY = imu->accY();
      float acZ = imu->accZ();

      float gyX = imu->gyrX();
      float gyY = imu->gyrY();
      float gyZ = imu->gyrZ();

      float newTemperature = barometer->getTemperature(); // deg C
      float newPressure = barometer->getPressure(); // hPa or mbar


      currentRocketPacket.timestamp = millis();
      currentRocketPacket.pressure = newPressure;
      currentRocketPacket.temperature = newTemperature;
      currentRocketPacket.acX = acX;
      currentRocketPacket.acY = acY;
      currentRocketPacket.acZ = acZ;
      currentRocketPacket.gyX = gyX;
      currentRocketPacket.gyY = gyY;
      currentRocketPacket.gyZ = gyZ;

      telemBoard->setCurrentPacket(currentRocketPacket);
    }

    telemBoard->onLoop();

    delay(100);

}