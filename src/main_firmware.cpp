//
// Created by firat on 20.01.20.
//
#include <array>
#include "car/bldc/driver.h"
#include "car/bldc/utils.h"
#include <arm_math.h>
#include <car/time/cycle_rate.h>
#include <Servo.h>

#include <car/car.h>

car::time::CycleRate cycle_uart(100);
car::time::CycleRate cycle_whatchdog(1000);
//car::time::CycleRate cycle_odom(10);


car::Car vehicle = car::Car::getInstance();

volatile bool flag = false;

void ftm0_isr(void)
{
    FTM0_SC &= ~FTM_SC_TOF;
    flag = true;
}



void setup()
{
    vehicle.uart_init();    /// blocks until a sync message arrives

    while (!Serial);
    
    delay(1000);
    car::bldc::Driver::getInstance().initHardware(13);

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
        car::bldc::Driver::getInstance().run();
    }

    if (cycle_uart.passed() > 0) vehicle.uart_send();
    //if (cycle_odom.passed() > 0) vehicle.odometrie_update();
    vehicle.uart_receive();

}
