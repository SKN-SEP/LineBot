#ifndef MOTORS_HPP
#define MOTORS_HPP

namespace mz_motors {
  // Constants
  const uint8_t ENA = 10; /**< First enable pin */
  const uint8_t IN1 = A0; /**< First input pin */
  const uint8_t IN2 = A1; /**< Second input pin */
  const uint8_t IN3 = A2; /**< Third pin */
  const uint8_t IN4 = A3; /**< Fourth pin */
  const uint8_t ENB = 9; /**< Second enable pin */

  /**
    * @brief Manages motor spin direction and speed.
    */
  class Motor {
    private:
    // Motor control pins
    uint8_t en_pin; /**< Enable pin */
    uint8_t in_pin_1; /**< First input pin */
    uint8_t in_pin_2; /**< Second input pin */

    public:
    /**
    * @brief Initializes motor pins.
    *
    * @param en The enable pin.
    *
    * @param in1 First input pin.
    *
    * @param in2 Second input pin.
    */
    Motor(uint8_t en, uint8_t in1, uint8_t in2);

    /**
      * @brief Spins motor forward.
      *
      * @param speed The duty cycle of the motor.
      *
      * @details
      * Sends three signals, high to first input pin, low to second input pin and speed to enable pin.
      */
    void forward(uint8_t speed);

    /**
      * @brief Spins motor backward.
      *
      * @param speed The duty cycle of the motor
      *
      * @details
      * Sends three signals, low to first input pin, high to second input pin and speed to enable pin.
      */
    void backward(uint8_t speed);
    
    /**
      * @brief Stops motor.
      *
      * @details
      * Sends three signals, low to first input pin, low to second input pin and 0 to enable pin.
      */
    void stop();
  };
}

#endif