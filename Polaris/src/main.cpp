#include <Arduino.h>

#include <GroundStation/TelemetryBoard.h>
#include <lib/MyMetroTimer.h>
#include <Flash/Flash.h>

#define CONVERSION 1000
#define LOOP_FREQUENCY 10

MyMetro timer = MyMetro(CONVERSION / LOOP_FREQUENCY);
FlashChip flash = FlashChip();

int counter = 0;
uint32_t timestamp;

TelemetryBoard telemBoard = TelemetryBoard();

void setup() {
  Serial.begin(115200);

  flash.init();
  flash.initialWrite();
  telemBoard.setState(TX);
  telemBoard.init();
}

void loop() {
    if(timer.check() == 1) {
        counter++;

        timestamp = counter * (CONVERSION/LOOP_FREQUENCY);

        telemBoard.onLoop(timestamp);
        timer.reset();
    }

    // Flash things for later: 
      // flash.writeStruct(&frame, sizeof(frame)); 
      //  update whenever the frame is defined 
      // you will probably have a function that logs the data to the flash chip
      // and then you can call that function here
      // For testing the reading of the flash chip
        // flash.readData();
        // make a counter (say it's like 5)
        // if the counter is 5, then read the data

} 