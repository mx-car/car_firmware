/**
 * @author Markus Bader <markus.bader@mx-robotics.com>
 * @license Simplified BSD License
 */

#ifndef CAR_CAR_H
#define CAR_CAR_H

#include <car/firmware/bldc.h>
#include <car/com/mc/interface.h>
#include <car/bldc/Controller.h>
#include <vector>
#include <Servo.h>

namespace car
{

    class Car
    {

    public:
        static Car &getInstance(); // Singleton handler

        void uart_init();
        void uart_receive();
        void uart_send();

    private:
        Car();
        std::vector<Motor> motors;

        car::com::mc::Interface msg_tx; /// object to hande the serial communication
        car::com::mc::Interface msg_rx; /// object to hande the serial communication
        car::com::objects::Text text;   /// object to debug msgs
        Controller *motor_controller;


        Servo steering_servo; 
    };

} // namespace car

#endif // CAR_CAR_H