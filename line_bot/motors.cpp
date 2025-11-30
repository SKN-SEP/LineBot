// Libraries
#include <Arduino.h>
#include "motors.hpp"

/**
  * @brief Initializes motor pins.
  *
  * @param en The enable pin.
  *
  * @param in1 First input pin.
  *
  * @param in2 Second input pin.
  */
mz_motors::Motor::Motor(uint8_t en, uint8_t in1, uint8_t in2) {
  this->en_pin = en;
  this->in_pin_1 = in1;
  this->in_pin_2 = in2;
}

/**
  * @brief Spins motor forward.
  *
  * @param speed The duty cycle of the motor.
  *
  * @details
  * Sends three signals, high to first input pin, low to second input pin and speed to enable pin.
  */
void mz_motors::Motor::forward(uint8_t speed) {
  digitalWrite(this->in_pin_1, 1);
  digitalWrite(this->in_pin_2, 0);
  analogWrite(this->en_pin, speed);
}

/**
  * @brief Spins motor backward.
  *
  * @param speed The duty cycle of the motor
  *
  * @details
  * Sends three signals, low to first input pin, high to second input pin and speed to enable pin.
  */
void mz_motors::Motor::backward(uint8_t speed) {
  digitalWrite(this->in_pin_1, 0);
  digitalWrite(this->in_pin_2, 1);
  analogWrite(this->en_pin, speed);
}

/**
  * @brief Stops motor.
  *
  * @details
  * Sends three signals, low to first input pin, low to second input pin and 0 to enable pin.
  */
void mz_motors::Motor::stop() {
  digitalWrite(this->in_pin_1, 0);
  digitalWrite(this->in_pin_2, 0);
  analogWrite(this->en_pin, 0);
}