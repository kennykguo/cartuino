// Arduino Cartpole Controller
// Interfaces with DE1-SOC via JP1/JP2 expansion header UART
// SHOULD MODIFY TO USE SERIAL1 when benchmarking script works


// Pin definitions for motor control
#define MOTOR_A_DIR1 5  // Motor A direction pin 1
#define MOTOR_A_DIR2 6  // Motor A direction pin 2
#define MOTOR_B_DIR1 9  // Motor B direction pin 1
#define MOTOR_B_DIR2 10 // Motor B direction pin 2
#define MOTOR_PWM    11 // PWM speed control

// IR sensor pins for position detection
#define IR_SENSOR_1  A0 // First IR sensor analog input
#define IR_SENSOR_2  A1 // Second IR sensor analog input

// Encoder pins for angle measurement
#define ENCODER_A  2  // Encoder phase A (interrupt pin)
#define ENCODER_B  3  // Encoder phase B (interrupt pin)

// UART pins for DE1-SoC communication
// Arduino TX connects to DE1-SoC RX (JP1 pin D0 or JP2 pin D0)
// Arduino RX connects to DE1-SoC TX (JP1 pin D1 or JP2 pin D1)
// Note: You must also connect GND between Arduino and DE1-SoC
#define DE1_SOC_RX   0 // Arduino TX (output) connects to DE1-SoC RX (input)
#define DE1_SOC_TX   1 // Arduino RX (input) connects to DE1-SoC TX (output)

// Constants for system
#define STRIP_WIDTH      0.025  // Width of black/white strips in meters
#define PULSES_PER_REV   600.0  // Encoder pulses per revolution
#define RADIANS_PER_PULSE (2.0 * PI / PULSES_PER_REV)
#define MOTOR_POWER      200    // PWM value (0-255) for motor power
#define SAMPLING_RATE    100    // Hz (10ms between samples)
#define BAUD_RATE        115200 // UART baud rate
#define FAIL_ANGLE       0.2    // Radians (approx 11.5 degrees)
#define FAIL_POSITION    0.5    // Meters from center

// Variables for encoder reading
volatile long encoder_count = 0;
volatile unsigned long last_encoder_time = 0;
volatile float angular_velocity = 0.0;
volatile byte last_encoded = 0;

// Variables for IR position sensing
volatile int strip_count = 0;
volatile unsigned long last_transition_time = 0;
volatile float cart_velocity = 0.0;
volatile float cart_position = 0.0;

// Control variables
unsigned long last_sample_time = 0;
int motor_action = 0;  // -1: left, 0: stop, 1: right
boolean is_failed = false;

// Software serial for DE1-SoC communication
#include <SoftwareSerial.h>

SoftwareSerial de1SocSerial(DE1_SOC_TX, DE1_SOC_RX); // RX, TX

void setup() {
  // Initialize hardware serial at 115200 baud (for debugging with PC)
  Serial.begin(BAUD_RATE);
  Serial.println("Cartpole Controller Starting...");
  
  // Initialize software serial for DE1-SoC communication
  de1SocSerial.begin(BAUD_RATE);
  
  // Configure motor pins as outputs
  pinMode(MOTOR_A_DIR1, OUTPUT);
  pinMode(MOTOR_A_DIR2, OUTPUT);
  pinMode(MOTOR_B_DIR1, OUTPUT);
  pinMode(MOTOR_B_DIR2, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);
  
  // Initialize motor state (stopped)
  digitalWrite(MOTOR_A_DIR1, LOW);
  digitalWrite(MOTOR_A_DIR2, LOW);
  digitalWrite(MOTOR_B_DIR1, LOW);
  digitalWrite(MOTOR_B_DIR2, LOW);
  analogWrite(MOTOR_PWM, 0);
  
  // Configure encoder pins with pull-up resistors
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  
  // Configure IR sensor pins as inputs
  pinMode(IR_SENSOR_1, INPUT);
  pinMode(IR_SENSOR_2, INPUT);
  
  // Configure UART pins
  pinMode(DE1_SOC_RX, OUTPUT);
  pinMode(DE1_SOC_TX, INPUT_PULLUP);
  
  // Attach interrupts for encoder
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoder_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), encoder_interrupt, CHANGE);
  
  // Initialize the system in reset state
  resetSystem();
  
  Serial.println("Cartpole Controller Initialized");
}

void loop() {
  // Maintain consistent sampling rate (10ms = 100Hz)
  unsigned long current_time = millis();
  if (current_time - last_sample_time >= (1000 / SAMPLING_RATE)) {
    last_sample_time = current_time;
    
    // Read current state
    updateCartPosition();
    
    // Check for failure conditions
    float pole_angle = getAngle();
    is_failed = (abs(pole_angle) > FAIL_ANGLE) || (abs(cart_position) > FAIL_POSITION);
    
    // Send state to DE1-SOC
    sendState();
    
    // Check for incoming action from DE1-SOC
    if (de1SocSerial.available() > 0) {
      receiveAction();
    }
  }
  
  // Apply motor control based on latest action
  applyMotorControl();
}

// Interrupt handler for encoder signals
void encoder_interrupt() {
  unsigned long current_time = micros();
  
  // Read both encoder pins
  int MSB = digitalRead(ENCODER_A);
  int LSB = digitalRead(ENCODER_B);
  
  // Convert the two pin values into a single number
  int encoded = (MSB << 1) | LSB;
  
  // XOR with previous encoded value to determine direction
  int sum = (last_encoded << 2) | encoded;
  
  // Update encoder count based on direction
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoder_count++;
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoder_count--;
  }
  
  // Calculate angular velocity (radians/second)
  unsigned long time_diff = current_time - last_encoder_time;
  if (time_diff > 1000) { // Avoid division by zero and noise
    float pulse_rate = 1000000.0 / time_diff; // pulses per second
    angular_velocity = pulse_rate * RADIANS_PER_PULSE;
    
    // Adjust sign based on direction
    if ((sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)) {
      angular_velocity = -angular_velocity;
    }
  }
  
  // Save current state for next interrupt
  last_encoded = encoded;
  last_encoder_time = current_time;
}

// Update cart position based on IR sensors
void updateCartPosition() {
  // Read IR sensors
  int ir1_value = analogRead(IR_SENSOR_1);
  int ir2_value = analogRead(IR_SENSOR_2);
  
  // Simplified position tracking based on IR readings
  // In a real implementation, you would use more sophisticated tracking
  // based on the black/white strip transitions
  
  // For this example, we'll estimate position based on analog readings
  // and use a simulated model for testing
  
  // Threshold for detecting black vs white
  static const int IR_THRESHOLD = 500;
  
  // Detecting transitions for velocity calculation
  static boolean last_ir1_state = false;
  boolean ir1_state = (ir1_value > IR_THRESHOLD);
  
  // If we detect a transition, update position and velocity
  if (ir1_state != last_ir1_state) {
    unsigned long current_time = micros();
    unsigned long time_diff = current_time - last_transition_time;
    
    // Update strip count (direction based on which sensor triggered first)
    if (ir1_state != (ir2_value > IR_THRESHOLD)) {
      strip_count++;  // Moving right
      cart_velocity = STRIP_WIDTH / (time_diff / 1000000.0);
    } else {
      strip_count--;  // Moving left
      cart_velocity = -STRIP_WIDTH / (time_diff / 1000000.0);
    }
    
    // Update position
    cart_position = strip_count * STRIP_WIDTH;
    
    last_transition_time = current_time;
    last_ir1_state = ir1_state;
  } else {
    // If no new transitions, gradually decay velocity
    // This simulates friction
    cart_velocity *= 0.98;
  }
  
  // Update position based on velocity (simple physics model)
  // For more accuracy, you'd integrate properly, but this approximation works for 10ms intervals
  cart_position += cart_velocity * (1.0 / SAMPLING_RATE);
}

// Get pole angle in radians
float getAngle() {
  // Convert encoder count to angle in radians
  return encoder_count * RADIANS_PER_PULSE;
}

// Send the current state to DE1-SOC via UART
void sendState() {
  // Format: cart_position,cart_velocity,pole_angle,pole_angular_velocity,is_failed
  de1SocSerial.print(cart_position, 6);     // Position in meters
  de1SocSerial.print(",");
  de1SocSerial.print(cart_velocity, 6);     // Velocity in m/s
  de1SocSerial.print(",");
  de1SocSerial.print(getAngle(), 6);        // Angle in radians
  de1SocSerial.print(",");
  de1SocSerial.print(angular_velocity, 6);  // Angular velocity in rad/s
  de1SocSerial.print(",");
  de1SocSerial.println(is_failed ? 1 : 0);  // Failure state (0 or 1)
  
  // Optional: Echo state to hardware serial for debugging
  Serial.print("State: ");
  Serial.print(cart_position, 6);
  Serial.print(",");
  Serial.print(cart_velocity, 6);
  Serial.print(",");
  Serial.print(getAngle(), 6);
  Serial.print(",");
  Serial.print(angular_velocity, 6);
  Serial.print(",");
  Serial.println(is_failed ? 1 : 0);
}

// Receive action from DE1-SOC via UART
void receiveAction() {
  // Read the action value
  int action = de1SocSerial.parseInt();
  
  // Debug output
  Serial.print("Received action: ");
  Serial.println(action);
  
  // Check for reset signal
  if (action == 0) {
    resetSystem();
  } else {
    motor_action = action;  // -1 for left, 1 for right
  }
}

// Apply the current motor action
void applyMotorControl() {
  if (is_failed) {
    // If system has failed, stop motors
    digitalWrite(MOTOR_A_DIR1, LOW);
    digitalWrite(MOTOR_A_DIR2, LOW);
    digitalWrite(MOTOR_B_DIR1, LOW);
    digitalWrite(MOTOR_B_DIR2, LOW);
    analogWrite(MOTOR_PWM, 0);
    return;
  }
  
  // Apply action based on received command
  switch (motor_action) {
    case -1:  // Move left
      digitalWrite(MOTOR_A_DIR1, HIGH);
      digitalWrite(MOTOR_A_DIR2, LOW);
      digitalWrite(MOTOR_B_DIR1, HIGH);
      digitalWrite(MOTOR_B_DIR2, LOW);
      analogWrite(MOTOR_PWM, MOTOR_POWER);
      break;
      
    case 1:  // Move right
      digitalWrite(MOTOR_A_DIR1, LOW);
      digitalWrite(MOTOR_A_DIR2, HIGH);
      digitalWrite(MOTOR_B_DIR1, LOW);
      digitalWrite(MOTOR_B_DIR2, HIGH);
      analogWrite(MOTOR_PWM, MOTOR_POWER);
      break;
      
    case 0:  // Stop
    default:
      digitalWrite(MOTOR_A_DIR1, LOW);
      digitalWrite(MOTOR_A_DIR2, LOW);
      digitalWrite(MOTOR_B_DIR1, LOW);
      digitalWrite(MOTOR_B_DIR2, LOW);
      analogWrite(MOTOR_PWM, 0);
      break;
  }
}

// Reset the system to initial state
void resetSystem() {
  // Stop motors
  motor_action = 0;
  applyMotorControl();
  
  // Reset encoder count and position tracking
  encoder_count = 0;
  angular_velocity = 0.0;
  strip_count = 0;
  cart_position = 0.0;
  cart_velocity = 0.0;
  
  // Reset failure state
  is_failed = false;
  
  Serial.println("System reset");
}