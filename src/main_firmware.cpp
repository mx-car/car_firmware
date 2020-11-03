/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.

  This example code is in the public domain.
 */

#include <car/firmware.h>

#include <car/com/mc/cycle_rate.h>
#include <car/motion/motion.h>

car::com::mc::CycleRate cycle_rate(500); /// object for a constant cycle control
car::motion::MotionVW motion;


int led = 13;
bool led_target = LOW;

// the setup routine runs once when you press reset:
void setup() {
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);   // wait for a second
  digitalWrite(led, led_target); 
}

// the loop routine runs over and over again forever:
void loop() {

  if(cycle_rate.passed() > 0){
    led_target = !led_target;
    digitalWrite(led, led_target);    // turn the LED off by making the voltage LOW
  }

  delay(10);               // wait for a second

}