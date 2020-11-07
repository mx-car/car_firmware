
#include <car/car.h>

using namespace car;

Car::Car()
{
    constexpr INHPins inhibitPins_{33, 26, 31};
    constexpr PWMPins initPins{10, 22, 23};
    constexpr ISPins isPins{A15, A16, A17};

    constexpr INHPins inhibitPins2{28, 8, 25};
    constexpr PWMPins initPins2{5, 6, 9};
    constexpr ISPins isPins2{A15, A16, A17};
    //    motors.emplace_back(std::move(Motor(inhibitPins_, initPins, 2, isPins)));
    //motors.emplace_back(std::move(Motor(inhibitPins2, initPins2, 2, isPins2)));
    Motor *motor0 = new Motor(inhibitPins_, initPins, 2, isPins);
    Motor *motor1 = new Motor(inhibitPins2, initPins2, 14, isPins2);

    motor_controller = &Controller::getInstance();
    motor_controller->registerMotors(motor0); // 80
    motor0->setAngleOffset(-10);  
    motor0->setAsRightWheel();
    motor_controller->registerMotors(motor1);
    motor0->setAngleOffset(-10); // - 110 is da best for direction, - 10 for the other one                   // - 10 seems aight for CCW

    steering_servo.attach(4); 
}

void Car::uart_init()
{
    Serial.begin(115200); /// init serial
    msg_rx.try_sync();    /// blocks until a sync message arrives
}

void Car::uart_receive()
{
    if (msg_rx.receive())
    { /// check for messages
        static car::com::objects::Object object;
        while (msg_rx.pop_object(object).isValid())
        {
            switch (object.type)
            {
            case car::com::objects::TYPE_SYNC:                         /// case sync object
                car::com::objects::Time::compute_offset(msg_rx.stamp); /// set clock
                break;
            case car::com::objects::TYPE_RACE_CAR:
            {
                static car::com::objects::RaceCar car_target;
                object.get(car_target);
                motor_controller->setCommand(car_target.motors.motor[0]*100., 0);
                motor_controller->setCommand(car_target.motors.motor[1]*100., 1);
                steering_servo.write(car_target.motors.servo*90+90);
            }
            break;
            default: /// case unkown type
                text.write("Unknown type received");
                continue;
            }
        }
    }
}

void Car::uart_send()
{
    msg_tx.reset(); /// removes all objects in message
    if (!text.empty())
    {
        msg_tx.push_object(car::com::objects::Object(text, car::com::objects::TYPE_TEXT));
    }
    {
        car::com::objects::RaceCar car_state;
        car_state.motors.motor[car::com::objects::LEFT] = motor_controller->getCommand(car::com::objects::LEFT) / 100.;
        car_state.motors.motor[car::com::objects::RIGHT] = motor_controller->getCommand(car::com::objects::RIGHT) / 100.;
        car_state.measurments.rps[car::com::objects::LEFT] = motor_controller->motors[car::com::objects::LEFT]->speedRPS;
        car_state.measurments.rps[car::com::objects::RIGHT] = motor_controller->motors[car::com::objects::RIGHT]->speedRPS;
        car_state.motors.stamp.fromMicros(motor_controller->getTStampCommand());
        car_state.stamp = car::com::objects::Time::now();
        msg_tx.push_object(car::com::objects::Object(car_state, car::com::objects::TYPE_RACE_CAR));
    }
    msg_tx.send();
}
Car &Car::getInstance()
{
    static Car instance;
    return instance;
}
