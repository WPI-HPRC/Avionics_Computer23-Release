#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "MyMetroTimer.h"

MyMetro::MyMetro() {
    previous_millis = 0;
    interval_millis = 1000;
}

MyMetro::MyMetro(unsigned long a_interval_millis) {
    previous_millis = 0;
    interval_millis = a_interval_millis;
}


void MyMetro::interval(unsigned long a_interval_millis)
{
    interval_millis = a_interval_millis;
}

uint8_t MyMetro::check()
{
    unsigned long now = millis();

    if ( interval_millis == 0 )
    {
        previous_millis = now;
        return 1;
    }

    if ( (now - previous_millis) >= interval_millis)
    {
        #ifdef NOCATCH_UP
            previous_millis = now ;
        #else
            previous_millis += interval_millis ;
        #endif
        return 1;
    }

    return 0;
}

void MyMetro::reset()
{
    previous_millis = millis();
}