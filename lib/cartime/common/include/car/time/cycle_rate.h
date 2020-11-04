/**
 * @author Markus Bader <markus.bader@mx-robotics.com>
 * @license Simplified BSD License
 */


#ifndef CAR_TIME_CYCLE_RATE_H
#define CAR_TIME_CYCLE_RATE_H

#include <Arduino.h>

namespace car {
namespace time {

/**
 * Class to generate a constant loop cycle
 **/
class CycleRate {
    uint32_t last;
    uint32_t next;
    uint32_t duration; 
public:
    /**
     * Constructer
     * @param ms cycle time;
     **/
    CycleRate(uint32_t ms);

    /**
     * used to check if a cylce was passed
     * @returns returns zero if no cycle was passed otherwise the number of missed cycles;
     **/
    int passed();  
};

};
};
#endif //CAR_TIME_CYCLE_RATE_H


