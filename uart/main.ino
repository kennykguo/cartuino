/*
 * Arduino Reliable Communication Protocol
 * For communication with DE1-SoC FPGA
 * 
 * Uses synchronous clocked communication with explicit
 * timing for increased reliability
 */

// Pin definitions
#define DATA_PIN 0  // D0 for data line
#define CLOCK_PIN 1 // D1 for clock line

// Communication parameters
#define BIT_PERIOD_MS 2    // 2ms per bit - 10x faster
#define MSG_BUFFER_SIZE 64 // Buffer size for messages
char tx_buffer[MSG_BUFFER_SIZE];
char rx_buffer[MSG_BUFFER_SIZE];
int rx_buffer_pos = 0;

// Message counter
int message_counter = 0;

// Protocol constants
#define START_SEQUENCE 0xAA // 10101010 pattern for synchronization
#define END_SEQUENCE 0x55   // 01010101 pattern for end of message
#define ACK_BYTE 0xCC       // 11001100 pattern for acknowledgment
#define NACK_BYTE 0x33      // 00110011 pattern for negative acknowledgment

// Timestamp for communication
unsigned long lastActivityTime = 0;

// Button for sending test message
#define BUTTON_PIN 2
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

void setup() {
  // Setup USB serial for debugging output
  Serial.begin(115200); // Higher baud rate for faster debugging
  
  // Configure DATA and CLOCK pins
  pinMode(DATA_PIN, INPUT); // Default to input mode
  pinMode(CLOCK_PIN, INPUT); // Default to input mode
  
  // Configure built-in LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  // Initial LED flash to show we're ready
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  
  delay(1000); // Give time for Serial Monitor to connect
  
  Serial.println("\n\n");
  Serial.println("==============================================");
  Serial.println("Arduino Reliable Communication Protocol Ready");
  Serial.println("==============================================");
  Serial.println("Auto-sending messages every 5 seconds");
  Serial.println("Will also receive messages from DE1-SoC");
  Serial.println("==============================================");
  
  // Check initial pin states
  Serial.print("Initial DATA pin state: ");
  Serial.println(digitalRead(DATA_PIN) ? "HIGH" : "LOW");
  Serial.print("Initial CLOCK pin state: ");
  Serial.println(digitalRead(CLOCK_PIN) ? "HIGH" : "LOW");
}

// Set the DATA pin direction
void setDataPinMode(int mode) {
  pinMode(DATA_PIN, mode);
}

// Set the DATA pin value (when configured as OUTPUT)
void setDataPin(int high) {
  digitalWrite(DATA_PIN, high ? HIGH : LOW);
}

// Read the DATA pin value
int readDataPin() {
  return digitalRead(DATA_PIN);
}

// Read the CLOCK pin value
int readClockPin() {
  return digitalRead(CLOCK_PIN);
}

// Send a single bit with clock monitoring and timeout
void sendBit(int bit) {
  unsigned long startTime = millis();
  boolean timedOut = false;
  
  // Set the data value
  setDataPin(bit);
  Serial.print("Setting data pin to ");
  Serial.println(bit ? "HIGH" : "LOW");
  
  // Wait for clock to go LOW (DE1-SoC signals ready to read)
  Serial.println("Waiting for clock LOW...");
  while(readClockPin() == HIGH && !timedOut) {
    if (millis() - startTime > 500) { // 500ms timeout
      timedOut = true;
      Serial.println("TIMEOUT waiting for clock LOW!");
    }
  }
  
  if (!timedOut) {
    Serial.println("Clock went LOW");
    
    // Wait for clock to go HIGH again (bit transmission complete)
    startTime = millis();
    Serial.println("Waiting for clock HIGH...");
    while(readClockPin() == LOW && !timedOut) {
      if (millis() - startTime > 500) { // 500ms timeout
        timedOut = true;
        Serial.println("TIMEOUT waiting for clock HIGH!");
      }
    }
    
    if (!timedOut) {
      Serial.println("Clock went HIGH");
    }
  }
  
  if (timedOut) {
    Serial.println("Communication error - clock signal timeout");
  }
}

// Send a byte (8 bits) MSB first
void sendByte(unsigned char byte) {
  // Set DATA pin as output
  setDataPinMode(OUTPUT);
  
  Serial.print("Sending byte: 0x");
  if (byte < 16) Serial.print("0");
  Serial.println(byte, HEX);
  
  // Send each bit, MSB first
  for (int i = 7; i >= 0; i--) {
    int bit = (byte >> i) & 0x01;
    sendBit(bit);
    Serial.print("Sent bit ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(bit);
  }
}

// Receive a single bit by monitoring clock transitions
int receiveBit() {
  int bit;
  
  // Wait for clock to go LOW (indicates data is ready)
  while(readClockPin() == HIGH) {
    // Add timeout check if needed
  }
  
  // Read data bit
  bit = readDataPin();
  
  // Wait for clock to go HIGH again (end of bit)
  while(readClockPin() == LOW) {
    // Add timeout check if needed
  }
  
  return bit;
}

// Receive a byte (8 bits) MSB first
unsigned char receiveByte() {
  unsigned char byte = 0;
  
  // Set DATA pin as input
  setDataPinMode(INPUT);
  
  // Receive 8 bits, MSB first
  for (int i = 7; i >= 0; i--) {
    int bit = receiveBit();
    byte |= (bit << i);
    Serial.print("Received bit ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(bit);
  }
  
  Serial.print("Received byte: 0x");
  if (byte < 16) Serial.print("0");
  Serial.println(byte, HEX);
  
  return byte;
}

// Send a message with protocol framing
void sendMessage(char *message) {
  int len = strlen(message);
  unsigned char checksum = 0;
  
  // Turn on LED during transmission
  digitalWrite(LED_BUILTIN, HIGH);
  
  Serial.print("Sending message: \"");
  Serial.print(message);
  Serial.print("\" (");
  Serial.print(len);
  Serial.println(" bytes)");
  
  // 1. Send start sequence for synchronization
  sendByte(START_SEQUENCE);
  
  // 2. Send length byte
  sendByte((unsigned char)len);
  checksum ^= len; // XOR for simple checksum
  
  // 3. Send each byte of the message
  for (int i = 0; i < len; i++) {
    sendByte((unsigned char)message[i]);
    checksum ^= message[i]; // Update checksum
  }
  
  // 4. Send checksum
  sendByte(checksum);
  
  // 5. Send end sequence
  sendByte(END_SEQUENCE);
  
  // Skip waiting for ACK to avoid blocking
  Serial.println("Message sent - continuing without waiting for ACK");
  
  // Turn off LED
  digitalWrite(LED_BUILTIN, LOW);
}

// Receive a message with protocol framing
int receiveMessage() {
  unsigned char byte, length, checksum = 0, calculated_checksum = 0;
  int i, success = 0;
  
  // Turn on LED during reception
  digitalWrite(LED_BUILTIN, HIGH);
  
  Serial.println("Waiting for start sequence...");
  
  // Set DATA pin as input
  setDataPinMode(INPUT);
  
  // 1. Wait for and verify start sequence
  byte = receiveByte();
  if (byte != START_SEQUENCE) {
    Serial.print("Invalid start sequence: 0x");
    if (byte < 16) Serial.print("0");
    Serial.print(byte, HEX);
    Serial.print(", expected 0x");
    if (START_SEQUENCE < 16) Serial.print("0");
    Serial.println(START_SEQUENCE, HEX);
    digitalWrite(LED_BUILTIN, LOW);
    return 0;
  }
  
  // 2. Receive length byte
  length = receiveByte();
  calculated_checksum ^= length;
  
  Serial.print("Expecting message of ");
  Serial.print(length);
  Serial.println(" bytes");
  
  // Ensure length is reasonable
  if (length >= MSG_BUFFER_SIZE) {
    Serial.print("Message too long: ");
    Serial.print(length);
    Serial.println(" bytes");
    digitalWrite(LED_BUILTIN, LOW);
    return 0;
  }
  
  // 3. Receive each byte of the message
  rx_buffer_pos = 0;
  for (i = 0; i < length; i++) {
    byte = receiveByte();
    rx_buffer[rx_buffer_pos++] = byte;
    calculated_checksum ^= byte;
  }
  
  // Null-terminate the string
  rx_buffer[rx_buffer_pos] = '\0';
  
  // 4. Receive checksum
  checksum = receiveByte();
  
  // 5. Receive end sequence
  byte = receiveByte();
  if (byte != END_SEQUENCE) {
    Serial.print("Invalid end sequence: 0x");
    if (byte < 16) Serial.print("0");
    Serial.print(byte, HEX);
    Serial.print(", expected 0x");
    if (END_SEQUENCE < 16) Serial.print("0");
    Serial.println(END_SEQUENCE, HEX);
    success = 0;
  } else if (checksum != calculated_checksum) {
    Serial.print("Checksum error: received 0x");
    if (checksum < 16) Serial.print("0");
    Serial.print(checksum, HEX);
    Serial.print(", calculated 0x");
    if (calculated_checksum < 16) Serial.print("0");
    Serial.println(calculated_checksum, HEX);
    success = 0;
  } else {
    Serial.print("Message received successfully: \"");
    Serial.print(rx_buffer);
    Serial.println("\"");
    success = 1;
  }
  
  // Skip sending ACK/NACK to avoid potential issues
  Serial.print("Message processing complete with status: ");
  Serial.println(success ? "SUCCESS" : "FAILED");
  
  // Turn off LED
  digitalWrite(LED_BUILTIN, LOW);
  
  return success;
}

// Check if a CLOCK pulse is detected (falling edge)
bool clockPulseDetected() {
  static int lastClockState = HIGH;
  int currentClockState = readClockPin();
  bool detected = false;
  
  if (lastClockState == HIGH && currentClockState == LOW) {
    detected = true;
  }
  
  lastClockState = currentClockState;
  return detected;
}

void loop() {
  static unsigned long lastPinCheckTime = 0;
  
  // Check for incoming message from DE1-SoC
  int clockState = readClockPin();
  if (clockState == LOW) {
    Serial.println("CLOCK pulse detected - attempting to receive message");
    receiveMessage();
    lastActivityTime = millis();
  }
  
  // Print pin states every second for debugging
  if (millis() - lastPinCheckTime > 1000) {
    Serial.print("DATA pin: ");
    Serial.print(digitalRead(DATA_PIN) ? "HIGH" : "LOW");
    Serial.print(", CLOCK pin: ");
    Serial.println(digitalRead(CLOCK_PIN) ? "HIGH" : "LOW");
    lastPinCheckTime = millis();
  }
  
  // Auto-send a simple test message every 5 seconds
  if (millis() - lastActivityTime > 5000) {
    Serial.println("\n--- Auto-sending test message ---");
    
    // Try a different approach - use manual bit-banging instead of protocol
    Serial.println("Sending manual pulse pattern instead of protocol message");
    
    // Ensure DATA pin is output
    pinMode(DATA_PIN, OUTPUT);
    
    // Send 10 pulses with 50ms timing (visible timing)
    for (int i = 0; i < 10; i++) {
      digitalWrite(DATA_PIN, HIGH);
      Serial.println("DATA: HIGH");
      delay(50);
      digitalWrite(DATA_PIN, LOW);
      Serial.println("DATA: LOW");
      delay(50);
    }
    
    // Return to normal state
    digitalWrite(DATA_PIN, LOW);
    pinMode(DATA_PIN, INPUT);
    
    Serial.println("Pulse pattern sent");
    lastActivityTime = millis();
  }
  
  // Small delay
  delay(5);
}