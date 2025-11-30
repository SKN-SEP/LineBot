// Motor pin constants
#define ENA 10
#define IN1 A0 
#define IN2 A1 
#define IN3 A2
#define IN4 A3
#define ENB 9

// Functions
void moveForward(uint8_t en, uint8_t in_1, uint8_t in_2, uint8_t duty_cycle);
void moveBackward(uint8_t en, uint8_t in_1, uint8_t in_2, uint8_t duty_cycle);
void stop(uint8_t en, uint8_t in_1, uint8_t in_2);

void setup() {
  // Set up pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

}

void loop() {
  // Test code
  moveForward(ENA, IN1, IN2, 250);
  moveForward(ENB, IN3, IN4, 250);
  delay(2000);

  stop(ENA, IN1, IN2);
  stop(ENB, IN3, IN4);
  delay(2000);

  moveBackward(ENA, IN1, IN2, 250);
  moveBackward(ENB, IN3, IN4, 250);
  delay(2000);

  stop(ENA, IN1, IN2);
  stop(ENB, IN3, IN4);
  delay(2000);

}

void moveForward(uint8_t en, uint8_t in_1, uint8_t in_2, uint8_t duty_cycle) {
  analogWrite(en, duty_cycle);
  digitalWrite(in_1, 1);
  digitalWrite(in_2, 0);
}

void moveBackward(uint8_t en, uint8_t in_1, uint8_t in_2, uint8_t duty_cycle) {
  analogWrite(en, duty_cycle);
  digitalWrite(in_1, 0);
  digitalWrite(in_2, 1);
}

void stop(uint8_t en, uint8_t in_1, uint8_t in_2) {
  analogWrite(en, 0);
  digitalWrite(in_1, 0);
  digitalWrite(in_2, 0);
}

