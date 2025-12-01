// Libraries
#include "motors.hpp"
#include <QTRSensors.h>

// Constants
const uint16_t BAUD_RATE = 9600; /**< Baud rate */
const uint8_t QTR_INPUT_PINS[8] = {12, 11, 7, 6, 5, 4, 3, 2}; /**< Line sensor input pins*/
const uint8_t QTR_LEDON_PIN = 13; /**< Line sensor emitter pin */

// PID constants
const uint8_t BASE_SPEED = 90; /**< Normal motor duty cycle */
const uint8_t MAX_SPEED = 120; /**< Maximum motor duty cycle */
const uint8_t MIN_SPEED = 0; /**< Minimum motor duty cycle */

const float KP = 0.03; /**< Propotional term, reacts how far you are off the line */
const float KI = 0.0; /**< Intergral term, not used */ 
const float KD = 0.1; /**< Derivative term, reacts to change in error */

// Global variables
mz_motors::Motor leftMT(mz_motors::ENA, mz_motors::IN1, mz_motors::IN2); /**< Left motor */
mz_motors::Motor rightMT(mz_motors::ENB, mz_motors::IN3, mz_motors::IN4); /**< Right motor */
uint16_t sensorValues[8];  /**< Sensor values */
QTRSensors qtr8; /**< Line sensor */
float prev_error; /**< Error from previous iteration */

// Initializes variables
void setup() {
  // Serial communication
  Serial.begin(BAUD_RATE);
  Serial.println("Started configuring Qtr8");

  // Configure pins
  pinMode(mz_motors::ENA, OUTPUT);
  pinMode(mz_motors::IN1, OUTPUT);
  pinMode(mz_motors::IN2, OUTPUT);
  pinMode(mz_motors::IN3, OUTPUT);
  pinMode(mz_motors::IN4, OUTPUT);
  pinMode(mz_motors::ENB, OUTPUT);

  for (int i=0; i<8; i++) pinMode(QTR_INPUT_PINS[i], INPUT);
  pinMode(QTR_LEDON_PIN, OUTPUT);

  // Calibrate qtr8
  Serial.println("Started configuring qtr8");
  qtr8.setTypeRC();
  qtr8.setSensorPins(QTR_INPUT_PINS, 8);
  qtr8.setEmitterPin(QTR_LEDON_PIN);

  // Calibrate (2.5ms/single read -> calibrates is 10 reads), total 1.25 second
  for (int i=0; i<50; i++) qtr8.calibrate();
  Serial.println("Completed configuring Qtr8");
  
  prev_error = 0;
}

// Executes program
void loop() {
  // Read line position  
  uint16_t position = qtr8.readLineBlack(sensorValues);
  Serial.print("Position: ");
  Serial.println(position);

  // Propotional term
  float curr_error = position - 3500.0;
  float p_out = curr_error * KP;
  float d_out = (curr_error-prev_error) * KD;

  int velocity = int(p_out + d_out);
  prev_error = curr_error;

  // Update motors speed
  int vL = BASE_SPEED + velocity;
  int vR = BASE_SPEED - velocity;

  leftMT.forward(min(max(vL, MIN_SPEED), MAX_SPEED));
  rightMT.forward(min(max(vR, MIN_SPEED), MAX_SPEED));
}
