#include <Adafruit_MotorShield.h>
#include <math.h>
#include <Wire.h>
#include <PinChangeInterrupt.h>  // Add this library


// Pin definitions
#define IR_SENSOR_PIN A0      // IR sensor for position tracking
#define ENCODER_PIN_A 10       // Rotary encoder pin A (Green)
#define ENCODER_PIN_B 11       // Rotary encoder pin B (White)

// Motor terminal definitions
#define MOTOR_A_TERMINAL 3
#define MOTOR_B_TERMINAL 1
#define MOTOR_C_TERMINAL 2
#define MOTOR_D_TERMINAL 4

// IR sensor calibration
#define IR_BLACK_THRESHOLD 500  // Threshold for black/white detection
#define STRIPS_COUNT 120        // Total number of strips (60 black + 60 white)
#define TRACK_LENGTH 22.0       // Total length in inches
#define STRIP_WIDTH (TRACK_LENGTH / STRIPS_COUNT)  // Width of each strip

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
// State limits - same as the simulation for consistency
#define MAX_ANGLE_RAD 0.35f    // Maximum allowed pole angle (radians)
#define MAX_POSITION 5.0f     // Maximum allowed cart position (inches from center)
#define MAX_ANGULAR_VELOCITY 4.0f  // Maximum allowed angular velocity

// Control parameters
#define MOTOR_MAX_SPEED 150    // Maximum motor speed

#define CONTROL_FREQUENCY 50   // Control frequency in Hz
#define CONTROL_INTERVAL (1000 / CONTROL_FREQUENCY)  // Control interval in milliseconds

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// Encoder constants
#define ENCODER_PPR 600        // Pulses per revolution
#define DEGREES_PER_COUNT (360.0 / (4.0 * ENCODER_PPR))  // Degrees per count (4x mode)
#define RADIANS_PER_COUNT (DEGREES_PER_COUNT * M_PI / 180.0)  // Radians per count

// Create the motor shield object
Adafruit_MotorShield afms = Adafruit_MotorShield();

// Get pointers to the motors
Adafruit_DCMotor *motor_a = afms.getMotor(MOTOR_A_TERMINAL);
Adafruit_DCMotor *motor_b = afms.getMotor(MOTOR_B_TERMINAL);
Adafruit_DCMotor *motor_c = afms.getMotor(MOTOR_C_TERMINAL);
Adafruit_DCMotor *motor_d = afms.getMotor(MOTOR_D_TERMINAL);

// Encoder variables
volatile int32_t encoder_count = 0;
volatile int8_t encoder_dir = 0;      // 1=CW, -1=CCW
volatile bool encoder_active = false; // Flag to control encoder processing
uint8_t prev_encoder_state = 0;
float pole_angle = 0.0;               // Current pole angle in radians
float pole_angular_velocity = 0.0;    // Current pole angular velocity in radians/sec

// IR sensor variables
int strip_count = 0;
float cart_position = 0.0;            // Current cart position in inches (0 = center)
float cart_velocity = 0.0;            // Current cart velocity in inches/sec
bool last_strip_was_black = false;
unsigned long last_strip_time = 0;
unsigned long last_ir_read_time = 0;
int last_ir_value = 0;

// System state variables
bool is_running = false;
unsigned long last_control_time = 0;
unsigned long last_state_update_time = 0;
unsigned long last_i2c_check = 0;     // For I2C health monitoring

// Communication variables
char incoming_byte;
char command_buffer[64];
int buffer_index = 0;

void setup() {
  Serial.begin(115200);
  
  // Set up encoder pins
  pinMode(ENCODER_PIN_A, INPUT);
  pinMode(ENCODER_PIN_B, INPUT);
  
  pinMode(IR_SENSOR_PIN, INPUT);
  
  // Initialize the motor shield with longer timeout
  Wire.begin();
  delay(100);
  afms.begin();
  delay(100);
  
  stop_motors();
  
  // Initialize encoder state BEFORE attaching interrupts
  prev_encoder_state = (digitalRead(ENCODER_PIN_A) << 1) | digitalRead(ENCODER_PIN_B);
  
  // Use pin change interrupts instead
  attachPCINT(digitalPinToPCINT(ENCODER_PIN_A), read_encoder, CHANGE);
  attachPCINT(digitalPinToPCINT(ENCODER_PIN_B), read_encoder, CHANGE);
  
  Serial.println("CartPole Arduino Controller Ready");
  Serial.println("Commands: START, STOP, RESET");
}

void loop() {
  // Check I2C health periodically
  check_i2c_health();
  
  // Process serial commands
  process_serial_commands();
  
  // Update state measurements
  update_state_variables();
  
  // Control loop - run at fixed frequency
  unsigned long current_time = millis();
  if (is_running && (current_time - last_control_time >= CONTROL_INTERVAL)) {
    last_control_time = current_time;
    
    // Send state data to host computer
    send_state_to_host();
    
    // Check if we're in a terminal state
    if (is_terminal_state()) {
      Serial.println("TERMINAL STATE REACHED");
      stop_motors();
      is_running = false;
      encoder_active = false; // Disable encoder processing
      
      // Reset I2C and motor shield to recover from any issues
      reset_i2c();
      
      // Flush serial
      while(Serial.available()) Serial.read();
    }
  }
}

void check_i2c_health() {
  // Check every 3 seconds
  if (millis() - last_i2c_check > 3000) {
    last_i2c_check = millis();
    
    // Try to communicate with motor shield
    Wire.beginTransmission(0x60); // Motor shield address
    byte error = Wire.endTransmission();
    
    // If communication failed, reset I2C
    if (error != 0) {
      reset_i2c();
    }
  }
}

void reset_i2c() {
  // Reset I2C bus
  Wire.end();
  delay(50);
  Wire.begin();
  delay(50);
  afms.begin();
  delay(50);
}

void process_serial_commands() {
  // Clear any garbage in the buffer first
  if (Serial.available() > 20) {
    while (Serial.available()) Serial.read();
    buffer_index = 0;
    return;
  }
  
  while (Serial.available() > 0) {
    incoming_byte = Serial.read();
    
    // Handle end of command
    if (incoming_byte == '\n' || incoming_byte == '\r') {
      if (buffer_index > 0) {
        command_buffer[buffer_index] = '\0';
        execute_command(command_buffer);
        buffer_index = 0;
      }
    } else if (buffer_index < sizeof(command_buffer) - 1) {
      command_buffer[buffer_index++] = incoming_byte;
    }
  }
}

void execute_command(char* command) {
  if (strncmp(command, "START", 5) == 0) {
    // Enable encoder first
    encoder_active = true;
    delay(50);
    
    // Reinitialize I2C to ensure clean state
    reset_i2c();
    
    Serial.println("Starting episode");
    reset_state();
    is_running = true;
  }
  else if (strncmp(command, "STOP", 4) == 0) {
    Serial.println("Stopping episode");
    stop_motors();
    is_running = false;
    encoder_active = false; // Disable encoder processing
  }
  else if (strncmp(command, "RESET", 5) == 0) {
    Serial.println("Resetting state");
    encoder_active = false; // Temporarily disable
    reset_state();
    encoder_active = true;  // Re-enable if already running
  }
  else if (strncmp(command, "MOTOR:", 6) == 0) {
    int motor_speed = atoi(command + 6);
    set_motor_speed(motor_speed);
    Serial.print("Motor speed set to: ");
    Serial.println(motor_speed);
  }
  else if (strncmp(command, "RESET_I2C", 9) == 0) {
    Serial.println("Resetting I2C and motor shield");
    reset_i2c();
  }
}

void reset_state() {
  noInterrupts();
  encoder_count = 0;
  encoder_dir = 0;
  interrupts();
  
  pole_angle = 0.0;
  pole_angular_velocity = 0.0;
  cart_position = 0.0;
  cart_velocity = 0.0;
  strip_count = 0;
  last_strip_was_black = false;
  last_strip_time = millis();
  last_state_update_time = millis();
  
  stop_motors();
}

void update_state_variables() {
  unsigned long current_time = millis();
  float delta_time = (current_time - last_state_update_time) / 1000.0;
  
  if (delta_time > 0.001) {
    noInterrupts();
    int32_t current_count = encoder_count;
    interrupts();
    
    float new_angle = current_count * RADIANS_PER_COUNT;
    float angle_delta = new_angle - pole_angle;
    pole_angular_velocity = angle_delta / delta_time;
    pole_angle = new_angle;
    
    update_cart_position(delta_time);
    
    last_state_update_time = current_time;
  }
}

void update_cart_position(float delta_time) {
  int ir_value = analogRead(IR_SENSOR_PIN);
  bool is_black = (ir_value > IR_BLACK_THRESHOLD);
  
  if (is_black != last_strip_was_black) {
    unsigned long current_strip_time = millis();
    unsigned long strip_time_delta = current_strip_time - last_strip_time;
    
    if (is_black) {
      strip_count++;
    } else {
      strip_count--;
    }
    
    cart_position = strip_count * STRIP_WIDTH;
    
    if (strip_time_delta > 0) {
      float strip_velocity = STRIP_WIDTH / (strip_time_delta / 1000.0);
      strip_velocity *= (is_black ? 1.0 : -1.0);
      cart_velocity = 0.7 * cart_velocity + 0.3 * strip_velocity;
    }
    
    last_strip_time = current_strip_time;
    last_strip_was_black = is_black;
  }
  
  unsigned long current_time = millis();
  if (current_time - last_ir_read_time > 10) {
    int ir_delta = ir_value - last_ir_value;
    
    if (abs(ir_delta) > 10) {
      float direction = ir_delta > 0 ? 1.0 : -1.0;
      float approximate_velocity = direction * abs(ir_delta) / 100.0;
      cart_velocity = 0.8 * cart_velocity + 0.2 * approximate_velocity;
    }
    
    last_ir_value = ir_value;
    last_ir_read_time = current_time;
  }
  
  if (millis() - last_strip_time > 300) {
    cart_velocity *= 0.9;
    if (fabs(cart_velocity) < 0.05) {
      cart_velocity = 0.0;
    }
  }
}

bool is_terminal_state() {
  return (fabs(pole_angle) > MAX_ANGLE_RAD || 
         fabs(cart_position) > MAX_POSITION);
}

void send_state_to_host() {
  Serial.print("STATE:");
  Serial.print(pole_angle, 6);
  Serial.print(",");
  Serial.print(pole_angular_velocity, 6);
  Serial.print(",");
  Serial.print(cart_position, 6);
  Serial.print(",");
  Serial.println(cart_velocity, 6);
}
void set_motor_speed(int speed) {
  speed = constrain(speed, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
  
  if (speed == 0) {
    stop_motors();
  } else if (speed > 0) {
    // RIGHT movement: A & C forward, B & D backward
    motor_a->setSpeed(abs(speed));
    motor_a->run(FORWARD);
    motor_c->setSpeed(abs(speed));
    motor_c->run(FORWARD);
    
    motor_b->setSpeed(abs(speed));
    motor_b->run(BACKWARD);
    motor_d->setSpeed(abs(speed));
    motor_d->run(BACKWARD);
  } else {
    // LEFT movement: A & C backward, B & D forward
    motor_a->setSpeed(abs(speed));
    motor_a->run(BACKWARD);
    motor_c->setSpeed(abs(speed));
    motor_c->run(BACKWARD);
    
    motor_b->setSpeed(abs(speed));
    motor_b->run(FORWARD);
    motor_d->setSpeed(abs(speed));
    motor_d->run(FORWARD);
  }
}

void stop_motors() {
  motor_a->setSpeed(0);
  motor_a->run(RELEASE);
  motor_b->setSpeed(0);
  motor_b->run(RELEASE);
  motor_c->setSpeed(0);
  motor_c->run(RELEASE);
  motor_d->setSpeed(0);
  motor_d->run(RELEASE);
}

void read_encoder() {
  if (!encoder_active) return;
  
  static uint32_t last_isr = 0;
  uint32_t now = micros();
  
  // Increased debounce time for stability
  if (now - last_isr < 1000) return; // 1ms debounce
  last_isr = now;
  
  uint8_t curr_state = (digitalRead(ENCODER_PIN_A) << 1) | digitalRead(ENCODER_PIN_B);
  
  static const int8_t transitions[16] = {
     0, -1,  1,  0, 
     1,  0,  0, -1, 
    -1,  0,  0,  1, 
     0,  1, -1,  0
  };
  
  int8_t delta = transitions[(prev_encoder_state << 2) | curr_state];
  
  if (delta) {
    encoder_count += delta;
    encoder_dir = delta > 0 ? 1 : -1;
    prev_encoder_state = curr_state;
  }
}