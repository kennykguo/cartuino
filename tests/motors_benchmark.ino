#include <Adafruit_MotorShield.h>

// Motor terminal definitions
#define MOTOR_A_TERMINAL 3
#define MOTOR_B_TERMINAL 1
#define MOTOR_C_TERMINAL 2
#define MOTOR_D_TERMINAL 4

// Settings for the slow turning
#define TURN_SPEED 100       // Reduced speed for slow turning (0-255)
#define TURN_DURATION 2000   // How long to turn in each direction (milliseconds)
#define PAUSE_DURATION 500   // Pause between direction changes (milliseconds)

// Create the motor shield object
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Get pointers to the motors
Adafruit_DCMotor *MOTOR_A = AFMS.getMotor(MOTOR_A_TERMINAL);
Adafruit_DCMotor *MOTOR_B = AFMS.getMotor(MOTOR_B_TERMINAL);
Adafruit_DCMotor *MOTOR_C = AFMS.getMotor(MOTOR_C_TERMINAL);
Adafruit_DCMotor *MOTOR_D = AFMS.getMotor(MOTOR_D_TERMINAL);

void setup() {
  Serial.begin(9600);
  Serial.println("Slow Left-Right Turn Test");
  
  // Initialize the motor shield
  AFMS.begin();
  
  // Initialize all motors to stopped
  stop();
  
  delay(1000); // Wait a second before starting
}

void loop() {
  // Turn left slowly
  Serial.println("Turning left...");
  turnLeft(TURN_SPEED);
  delay(TURN_DURATION);
  
  // Stop briefly
  stop();
  delay(PAUSE_DURATION);
  
  // Turn right slowly
  Serial.println("Turning right...");
  turnRight(TURN_SPEED);
  delay(TURN_DURATION);
  
  // Stop briefly
  stop();
  delay(PAUSE_DURATION);
}

// Function to stop all motors
void stop() {
  MOTOR_A->setSpeed(0);
  MOTOR_A->run(RELEASE);

  MOTOR_B->setSpeed(0);
  MOTOR_B->run(RELEASE);

  MOTOR_C->setSpeed(0);
  MOTOR_C->run(RELEASE);

  MOTOR_D->setSpeed(0);
  MOTOR_D->run(RELEASE);
}

// Function to turn left slowly
void turnLeft(int speed) {
  // Left side motors backward
  MOTOR_A->setSpeed(speed);
  MOTOR_A->run(BACKWARD);
  
  MOTOR_C->setSpeed(speed);
  MOTOR_C->run(BACKWARD);
  
  // Right side motors forward
  MOTOR_B->setSpeed(speed);
  MOTOR_B->run(FORWARD);
  
  MOTOR_D->setSpeed(speed);
  MOTOR_D->run(FORWARD);
}

// Function to turn right slowly
void turnRight(int speed) {
  // Left side motors forward
  MOTOR_A->setSpeed(speed);
  MOTOR_A->run(FORWARD);
  
  MOTOR_C->setSpeed(speed);
  MOTOR_C->run(FORWARD);
  
  // Right side motors backward
  MOTOR_B->setSpeed(speed);
  MOTOR_B->run(BACKWARD);
  
  MOTOR_D->setSpeed(speed);
  MOTOR_D->run(BACKWARD);
}