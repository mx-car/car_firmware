//
// Created by firat on 20.01.20.
//
#include <array>
#include "car/bldc/driver.h"
#include "car/bldc/utils.h"
#include <car/time/cycle_rate.h>
#include <Servo.h>

#define CALIBRATE 1

constexpr car::bldc::INHPins inhibitPins_{33, 26, 31};
constexpr car::bldc::PWMPins initPins{10, 22, 23};
constexpr car::bldc::ISPins isPins{A15, A16, A17};
car::bldc::Motor motor0(inhibitPins_, initPins, 2, isPins);

constexpr car::bldc::INHPins inhibitPins2{28, 8, 25};
constexpr car::bldc::PWMPins initPins2{5, 6, 9};
constexpr car::bldc::ISPins isPins2{A15, A16, A17};

car::bldc::Motor motor1(inhibitPins2, initPins2, 14, isPins2);

Servo myservo; 


volatile bool flag = false;

void ftm0_isr(void)
{
    FTM0_SC &= ~FTM_SC_TOF;
    flag = true;
}

void setup()
{

    car::bldc::Driver::getInstance().registerMotors(&motor0); //
    motor0.setAsLeftWheel();
    car::bldc::Driver::getInstance().registerMotors(&motor1);
    myservo.attach(4);

    Serial.begin(115200); /// init serial
    //msg_rx.try_sync();    /// blocks until a sync message arrives

    while (!Serial)
        ;
    delay(1000);
    car::bldc::Driver::getInstance().initHardware(13);

    cli(); //Disable global interrupts
    NVIC_SET_PRIORITY(IRQ_FTM0, 64);
    NVIC_ENABLE_IRQ(IRQ_FTM0);
    sei(); //Enable global interrupts
}

void loop() {

    if (flag) {
        flag = false;
#if CALIBRATE
        car::bldc::Diagnostics::calculateAngleFiner(motor0,motor1);
#else
        car::bldc::Driver::getInstance().run();
        //uint16_t rotaryEncoderValue = car::bldc::RotaryEncoder::SPITransfer(motor0);
        //Serial.println(rotaryEncoderValue);
        //
        //
        #endif

    }
}