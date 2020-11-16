/**
 * @author Markus Bader <markus.bader@mx-robotics.com>
 * @license Simplified BSD License
 */


#ifndef CAR_FIRMWARE_BLDC_H
#define CAR_FIRMWARE_BLDC_H

#include <car/bldc/motor.h>

#if defined(NEW_BOARD)

constexpr car::bldc::INHPins inhibitPins_{33, 26, 31};
constexpr car::bldc::PWMPins initPins{10, 22, 23};
constexpr car::bldc::ISPins isPins{A15, A16, A17};

constexpr car::bldc::INHPins inhibitPins2{28, 8, 25};
constexpr car::bldc::PWMPins initPins2{5, 6, 9};
constexpr car::bldc::ISPins isPins2{A15, A16, A17};


#endif // NEW_BOARD

#endif // CAR_FIRMWARE_BLDC_H