/**
 * @author Markus Bader <markus.bader@mx-robotics.com>
 * @license Simplified BSD License
 */

#ifndef CAR_CAR_H
#define CAR_CAR_H

#include <car/firmware/bldc.h>
#include <car/com/mc/interface.h>
#include <car/bldc/driver.h>
#include <car/motion/odometry.h>
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

        void odometrie_update();

    private:
        Car();

        car::com::mc::Interface msg_tx; /// object to hande the serial communication
        car::com::mc::Interface msg_rx; /// object to hande the serial communication
        car::com::objects::Text text;   /// object to debug msgs
        car::bldc::Driver *motor_driver;
        car::motion::OdomAckermann *odometry;


        Servo steering_servo; 
        car::com::objects::ConfigAckermann *config_ackermann;
        car::com::objects::CommandAckermann *command_ackermann;
        car::com::objects::StateAckermann *state_ackermann;
    };

} // namespace car

#endif // CAR_CAR_H