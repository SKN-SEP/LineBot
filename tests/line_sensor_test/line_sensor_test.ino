// Libraries
#include <QTRSensors.h>

// Qtr8 constants
#define IN8 2
#define IN7 3
#define IN6 4
#define IN5 5
#define IN4 6
#define IN3 7
#define IN2 8
#define IN1 9
#define LEDON 10

// Global variables
QTRSensors qtr;
uint16_t sensorValues[8];

void setup() {
  // Setup serial communication
  Serial.begin(9600);
  Serial.println("Started configuring Qtr8");

  for (int i=0; i<8; i++) {
  pinMode((uint8_t[]){IN1, IN2, IN3, IN4, IN5, IN6, IN7, IN8}[i], INPUT);
  }

  
  // Configure sensor
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){IN1, IN2, IN3, IN4, IN5, IN6, IN7, IN8}, 8);
  qtr.setEmitterPin(LEDON);

  // Turn on esp32 node to inform about calibration
  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, 1);

  // Calibrate (2.5ms/single read -> calibrates is 10 reads), total 10 seconds
  for (int i=0; i<400; i++) qtr.calibrate();
  digitalWrite(LED_BUILTIN, 0);

  Serial.println("Completed configuring Qtr8");
}

void loop() {
  // Read line position  
  uint16_t position = qtr.readLineBlack(sensorValues);

  // Print out values
  for (int i=0; i<8; i++) {
    Serial.print(sensorValues[i]);
    Serial.print(' ');
  }

  Serial.print("Position: ");
  Serial.println(position);
}
