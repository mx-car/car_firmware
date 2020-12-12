
#include <car/car.h>
#include <HardwareSerial.h>

using namespace car;

Car::Car()
{
    constexpr car::bldc::INHPins inhibitPins_{33, 26, 31};
    constexpr car::bldc::PWMPins initPins{10, 22, 23};
    constexpr car::bldc::ISPins isPins{A15, A16, A17};

    constexpr car::bldc::INHPins inhibitPins2{28, 8, 25};
    constexpr car::bldc::PWMPins initPins2{5, 6, 9};
    constexpr car::bldc::ISPins isPins2{A15, A16, A17};
    //    motors.emplace_back(std::move(Motor(inhibitPins_, initPins, 2, isPins)));
    //motors.emplace_back(std::move(Motor(inhibitPins2, initPins2, 2, isPins2)));
    
    car::bldc::Motor *motor0 = new car::bldc::Motor(inhibitPins_, initPins, 2, isPins);
    car::bldc::Motor *motor1 = new car::bldc::Motor(inhibitPins2, initPins2, 14, isPins2);

    motor_driver = &car::bldc::Driver::getInstance();
    motor_driver->registerMotors(motor0); // 80
    motor0->setAsLeftWheel();
    motor_driver->registerMotors(motor1);


    steering_servo.attach(4); 
    config_ackermann = NULL;
    command_ackermann = NULL;  
    odometry = NULL;
    state_ackermann = new car::com::objects::StateAckermann;    

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
    using namespace car::com::objects;
        static Object object;
        while (msg_rx.pop_object(object).isValid())
        {
            switch (object.type)
            {
            case TYPE_SYNC:                         /// case sync object
                Time::compute_offset(msg_rx.stamp); /// set clock
                break;
            case TYPE_CONFIG_ACKERMANN:
            {
                if(config_ackermann == NULL)  {
                    config_ackermann = new ConfigAckermann;
                    state_ackermann = new car::com::objects::StateAckermann();
                }
                object.get(*config_ackermann);
                if(!odometry) {
                    odometry = new car::motion::OdomAckermann;
                    odometry->init(config_ackermann->wheel_diameter, config_ackermann->wheel_displacement, config_ackermann->axis_displacement);
                }
            }
            break;
            case TYPE_COMMAND_ACKERMANN:
            {
                if(command_ackermann == NULL) command_ackermann = new CommandAckermann;
                object.get(*command_ackermann);
                if(command_ackermann->units == CommandAckermann::UNIT_DIRECT){
                    motor_driver->setCommand(command_ackermann->forward, command_ackermann->inhibitor, LEFT);
                    motor_driver->setCommand(command_ackermann->forward, command_ackermann->inhibitor, RIGHT);
                    steering_servo.write(command_ackermann->steering*90+90);
                }
            }
            break;
            default: /// case unkown type
                continue;
            }
        }
    }
}

void Car::uart_send()
{
    using namespace car::com::objects;
    using namespace car::bldc;
    msg_tx.reset(); /// removes all objects in message
    if (!text.empty())
    {
        msg_tx.push_object(Object(text, TYPE_TEXT));
    }
    if(state_ackermann){
        state_ackermann->wheels[REAR_WHEEL_LEFT].target[ROTATION] = motor_driver->getCommand(LEFT) / 100.;
        state_ackermann->wheels[REAR_WHEEL_RIGHT].target[ROTATION] = motor_driver->getCommand(RIGHT) / 100.;
        state_ackermann->wheels_tstamp.target = Time::fromMicros(motor_driver->getTStampCommand());
        state_ackermann->wheels[REAR_WHEEL_LEFT].speed[ROTATION] = motor_driver->motors[LEFT]->speedRPS;
        state_ackermann->wheels[REAR_WHEEL_RIGHT].speed[ROTATION] = motor_driver->motors[RIGHT]->speedRPS;
        state_ackermann->wheels_tstamp.speed =  Time::fromMicros(motor_driver->getTStampMeasurement());
        if(odometry)  odometry->getPose(state_ackermann->pose);
        state_ackermann->stamp = Time::now();
        msg_tx.push_object(Object(*state_ackermann, TYPE_STATE_ACKERMANN));
    }
    if(command_ackermann){
        msg_tx.push_object(Object(*command_ackermann, TYPE_COMMAND_ACKERMANN));
    }
    if(config_ackermann){
        msg_tx.push_object(Object(*config_ackermann, TYPE_CONFIG_ACKERMANN));
    }
    
    
    msg_tx.send();
}
Car &Car::getInstance()
{
    static Car instance;
    return instance;
}

void Car::odometrie_update()
{
    if(!odometry) return;
    using namespace car::com::objects;
    odometry->update(motor_driver->motors[car::com::objects::LEFT]->speedRPS, motor_driver->motors[car::com::objects::RIGHT]->speedRPS, 0.);
}