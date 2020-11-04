/**
 * @author Markus Bader <markus.bader@mx-robotics.com>
 * @license Simplified BSD License
 */

#include "car/time/cycle_rate.h"

using namespace car::time;

CycleRate::CycleRate(uint32_t duration_ms) 
: duration(duration_ms)
{
    last = millis();
    next = last + duration;
}

int CycleRate::passed()
{
    int counter = 0;
    uint32_t now = millis();
    if(now > next){
        last = now;
        while(next < now) {
            counter++;
            next+=duration;
        }
    }
    return counter;
}   
