

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "Metro.h"

Metro::Metro() :
    previous_millis(0)
{
    interval_millis = 1000;
}


Metro::Metro(unsigned long a_interval_millis) :
    previous_millis(0)
{
    interval_millis = a_interval_millis;
}


void Metro::interval(unsigned long a_interval_millis)
{
    interval_millis = a_interval_millis;
}

uint8_t Metro::check()
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

void Metro::reset()
{
    previous_millis = millis();
}
