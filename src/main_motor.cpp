//
// Created by firat on 20.01.20.
//
#include <array>
#include "car/bldc/driver.h"
#include "car/bldc/utils.h"
#include <arm_math.h>
#include <car/com/mc/interface.h>
#include <car/time/cycle_rate.h>
#include <Servo.h>

#if defined(NEW_BOARD)
constexpr INHPins inhibitPins_{33, 26, 31};
constexpr PWMPins initPins{10, 22, 23};
constexpr ISPins isPins{A15, A16, A17};
Motor motor0(inhibitPins_, initPins, 2, isPins);

constexpr INHPins inhibitPins2{28, 8, 25};
constexpr PWMPins initPins2{5, 6, 9};
constexpr ISPins isPins2{A15, A16, A17};

Motor motor1(inhibitPins2, initPins2, 14, isPins2);

car::time::CycleRate cycle_uart(100);
car::time::CycleRate cycle_whatchdog(1000);

Servo myservo; 

#else

constexpr INHPins inhibitPins_{33, 24, 31};
constexpr PWMPins initPins{21, 23, 22};
constexpr ISPins isPins{A15, A16, A17};
Motor motor0(inhibitPins_, initPins, 10, isPins);
#endif

volatile bool flag = false;

void ftm0_isr(void)
{
    FTM0_SC &= ~FTM_SC_TOF;
    flag = true;
}

car::com::mc::Interface msg_tx; /// object to hande the serial communication
car::com::mc::Interface msg_rx; /// object to hande the serial communication
car::com::objects::Text text;   /// object to debug msgs

car::com::objects::State getCommmand(Driver &controller){
    static car::com::objects::State state;
    state.rps[car::com::objects::State::LEFT] = driver.command[car::com::objects::State::LEFT];
    state.rps[car::com::objects::State::RIGHT] = driver.command[car::com::objects::State::RIGHT];
    state.stamp.fromMicros(driver.tstamp_command[car::com::objects::State::LEFT] / 2UL + driver.tstamp_command[car::com::objects::State::RIGHT] / 2UL );
    return state;
}

car::com::objects::State getState(Driver &driver){
    static car::com::objects::State state;
    state.rps[car::com::objects::State::LEFT] = driver.speed[car::com::objects::State::LEFT];
    state.rps[car::com::objects::State::RIGHT] = driver.speed[car::com::objects::State::RIGHT];
    state.stamp.fromMicros(driver.tstamp_state[car::com::objects::State::LEFT] / 2UL + driver.tstamp_state[car::com::objects::State::RIGHT] / 2UL );
    state.stamp.now();
    return state;
}


void setCommmand(Driver &driver, car::com::objects::State state){
    driver.tstamp_command[car::com::objects::State::LEFT] = micros();
    driver.tstamp_command[car::com::objects::State::RIGHT] = micros();
    driver.command[car::com::objects::State::LEFT] = state.rps[car::com::objects::State::LEFT];
    driver.command[car::com::objects::State::RIGHT] = state.rps[car::com::objects::State::RIGHT];
    myservo.write(state.steering);
}

void setup()
{
#if defined(NEW_BOARD)
    Driver::getInstance().registerMotors(&motor0); // 80
    motor0.setAngleOffset(-10);                        // - 10 seems aight for CCW
    motor0.setAsRightWheel();
    Driver::getInstance().registerMotors(&motor1);
    motor1.setAngleOffset(-10); // - 110 is da best for direction, - 10 for the other one
    myservo.attach(4); 
#else
    Driver::getInstance().registerMotors(&x);
#endif
    Serial.begin(115200); /// init serial
    msg_rx.try_sync();    /// blocks until a sync message arrives

    while (!Serial)
        ;
    delay(1000);
    Driver::getInstance().initHardware(13);

    cli(); //Disable global interrupts
    NVIC_SET_PRIORITY(IRQ_FTM0, 64);
    NVIC_ENABLE_IRQ(IRQ_FTM0);
    sei(); //Enable global interrupts
}

void loop()
{

    if (flag)
    {
        flag = false;
        Driver::getInstance().run();
        //uint16_t rotaryEncoderValue = RotaryEncoderCommunication::SPITransfer(motor1);
        //Serial.println(rotaryEncoderValue);
    }

    if (cycle_uart.passed() > 0)
    {
        msg_tx.reset(); /// removes all objects in message
        if (!text.empty())
        {
            msg_tx.push_object(car::com::objects::Object(text, car::com::objects::TYPE_TEXT));
        }
        msg_tx.push_object(car::com::objects::Object(getCommmand(Driver::getInstance()), car::com::objects::TYPE_COMMAND_RAW));
        msg_tx.push_object(car::com::objects::Object(getState(Driver::getInstance()), car::com::objects::TYPE_STATE_RAW));
        msg_tx.send();
    }

    /// sends the message
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
            case car::com::objects::TYPE_COMMAND_RAW:
            {
                static car::com::objects::CmdRaw command;                
                object.get(command);
                setCommmand(Driver::getInstance(), command);
            }
            break;
            default: /// case unkown type
                text.write("Unknown type received");
                continue;
            }
        }
    }
}
