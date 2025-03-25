/*
 * Arduino Simplified Communication Protocol
 * For communication with DE1-SoC FPGA
 * 
 * Uses a simple bit-banging approach with DE1-SOC controlling the clock
 */

// Pin definitions
#define DATA_PIN 0  // D0 for data line
#define CLOCK_PIN 1 // D1 for clock line (controlled by DE1-SOC)

// Communication parameters
#define BIT_PERIOD_MS 50   // 50ms per bit - slower for reliability
#define MSG_BUFFER_SIZE 64 // Buffer size for messages
char tx_buffer[MSG_BUFFER_SIZE];
char rx_buffer[MSG_BUFFER_SIZE];
int rx_buffer_pos = 0;

// Message counter
int message_counter = 0;

// Protocol constants
#define START_BYTE 0xAA // 10101010 pattern for synchronization
#define END_BYTE 0x55   // 01010101 pattern for end of message

// Debug flag - set to 1 to enable verbose output
#define DEBUG 1

// Timestamp for communication
unsigned long lastActivityTime = 0;

// Button for sending test message
#define BUTTON_PIN 2
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

void setup() {
  // Setup USB serial for debugging output
  Serial.begin(9600);
  while (!Serial && millis() < 3000); // Wait for Serial, but timeout after 3 seconds
  
  // Configure pins with initial states
  pinMode(DATA_PIN, INPUT); // Default to input mode
  pinMode(CLOCK_PIN, INPUT); // Always input (DE1-SOC controls clock)
  
  // Configure built-in LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  // Configure button with pull-up resistor
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  // Initial LED flash pattern to show we're ready
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  
  Serial.println("\n\n==============================================");
  Serial.println("Arduino Simplified Communication Protocol Ready");
  Serial.println("==============================================");
  Serial.println("Controls:");
  Serial.println("- Button on pin 2: Send test message to DE1-SoC");
  Serial.println("- Will automatically receive messages from DE1-SoC");
  Serial.println("==============================================");
  
  // Initial delay to ensure both systems are ready
  delay(500);
}

// Set the DATA pin mode (INPUT or OUTPUT)
void setDataPinMode(int mode) {
  pinMode(DATA_PIN, mode);
  // Small delay to ensure mode change takes effect
  delayMicroseconds(500);
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

// Wait for CLOCK transition with timeout
// Returns true if transition detected, false if timeout
bool waitForClockTransition(bool waitForHigh, unsigned long timeoutMs) {
  unsigned long startTime = millis();
  
  while (digitalRead(CLOCK_PIN) != (waitForHigh ? HIGH : LOW)) {
    if (millis() - startTime > timeoutMs) {
      if (DEBUG) {
        Serial.print("Clock transition timeout waiting for ");
        Serial.println(waitForHigh ? "HIGH" : "LOW");
      }
      return false;
    }
    delayMicroseconds(100); // Short delay while polling
  }
  
  return true;
}

// Send a byte (8 bits, MSB first)
// Arduino watches for clock transitions controlled by DE1-SOC
bool sendByte(unsigned char byte) {
  if (DEBUG) {
    Serial.print("Sending byte: 0x");
    if (byte < 16) Serial.print("0");
    Serial.println(byte, HEX);
  }
  
  // Set DATA pin as output
  setDataPinMode(OUTPUT);
  
  // Send each bit, MSB first
  for (int i = 7; i >= 0; i--) {
    // Set the data bit
    int bit = (byte >> i) & 0x01;
    setDataPin(bit);
    
    if (DEBUG) {
      Serial.print("Set bit ");
      Serial.print(i);
      Serial.print(" to ");
      Serial.println(bit);
    }
    
    // Wait for clock to go LOW (DE1-SOC signals it's reading)
    if (!waitForClockTransition(false, 1000)) { // 1 second timeout
      Serial.print("Timeout waiting for clock LOW at bit ");
      Serial.println(i);
      return false;
    }
    
    // Keep data bit stable while clock is LOW
    
    // Wait for clock to go HIGH again (bit transmission complete)
    if (!waitForClockTransition(true, 1000)) { // 1 second timeout
      Serial.print("Timeout waiting for clock HIGH at bit ");
      Serial.println(i);
      return false;
    }
    
    if (DEBUG) {
      Serial.print("Sent bit ");
      Serial.println(i);
    }
  }
  
  return true;
}

// Send a message with framing bytes
bool sendMessage(char *message) {
  int len = strlen(message);
  bool success = true;
  
  // Turn on LED during transmission
  digitalWrite(LED_BUILTIN, HIGH);
  
  Serial.print("Sending message: \"");
  Serial.print(message);
  Serial.print("\" (");
  Serial.print(len);
  Serial.println(" bytes)");
  
  // 1. Send start byte for synchronization
  if (!sendByte(START_BYTE)) {
    Serial.println("Failed to send START_BYTE");
    digitalWrite(LED_BUILTIN, LOW);
    return false;
  }
  
  // 2. Send length byte
  if (!sendByte((unsigned char)len)) {
    Serial.println("Failed to send length");
    digitalWrite(LED_BUILTIN, LOW);
    return false;
  }
  
  // 3. Send each byte of the message
  for (int i = 0; i < len; i++) {
    if (!sendByte((unsigned char)message[i])) {
      Serial.print("Failed to send byte at position ");
      Serial.println(i);
      digitalWrite(LED_BUILTIN, LOW);
      return false;
    }
  }
  
  // 4. Send end byte
  if (!sendByte(END_BYTE)) {
    Serial.println("Failed to send END_BYTE");
    digitalWrite(LED_BUILTIN, LOW);
    return false;
  }
  
  Serial.println("Message sent successfully");
  
  // Turn off LED
  digitalWrite(LED_BUILTIN, LOW);
  
  return true;
}

// Receive a byte (8 bits, MSB first)
// Arduino watches for clock transitions controlled by DE1-SOC
int receiveByte() {
  unsigned char byte = 0;
  
  // Set DATA pin as input
  setDataPinMode(INPUT);
  
  // Receive 8 bits, MSB first
  for (int i = 7; i >= 0; i--) {
    // Wait for clock to go LOW (indicates DE1-SOC is signaling bit available)
    if (!waitForClockTransition(false, 1000)) { // 1 second timeout
      Serial.print("Timeout waiting for clock LOW at bit ");
      Serial.println(i);
      return -1;
    }
    
    // Read data bit while clock is LOW
    int bit = readDataPin();
    
    // Wait for clock to go HIGH again (bit receive complete)
    if (!waitForClockTransition(true, 1000)) { // 1 second timeout
      Serial.print("Timeout waiting for clock HIGH at bit ");
      Serial.println(i);
      return -1;
    }
    
    // Store the bit
    if (bit) {
      byte |= (1 << i);
    }
    
    if (DEBUG) {
      Serial.print("Received bit ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(bit);
    }
  }
  
  if (DEBUG) {
    Serial.print("Received byte: 0x");
    if (byte < 16) Serial.print("0");
    Serial.println(byte, HEX);
  }
  
  return byte;
}

// Receive a message byte by byte
bool receiveMessage() {
  int byte_value, length;
  bool success = false;
  
  // Turn on LED during reception
  digitalWrite(LED_BUILTIN, HIGH);
  
  Serial.println("Receiving message...");
  
  // Set DATA pin as input
  setDataPinMode(INPUT);
  
  // 1. Look for start byte
  byte_value = receiveByte();
  if (byte_value < 0 || byte_value != START_BYTE) {
    Serial.print("Invalid or missing start byte: 0x");
    if (byte_value >= 0 && byte_value < 16) Serial.print("0");
    if (byte_value >= 0) Serial.print(byte_value, HEX);
    else Serial.print("ERROR");
    Serial.print(", expected 0x");
    if (START_BYTE < 16) Serial.print("0");
    Serial.println(START_BYTE, HEX);
    digitalWrite(LED_BUILTIN, LOW);
    return false;
  }
  
  Serial.println("Start byte detected");
  
  // 2. Receive length byte
  byte_value = receiveByte();
  if (byte_value < 0) {
    Serial.println("Error receiving length byte");
    digitalWrite(LED_BUILTIN, LOW);
    return false;
  }
  
  length = byte_value;
  Serial.print("Expecting message of ");
  Serial.print(length);
  Serial.println(" bytes");
  
  // Ensure length is reasonable
  if (length >= MSG_BUFFER_SIZE || length <= 0) {
    Serial.print("Invalid message length: ");
    Serial.println(length);
    digitalWrite(LED_BUILTIN, LOW);
    return false;
  }
  
  // 3. Receive each byte of the message
  rx_buffer_pos = 0;
  for (int i = 0; i < length; i++) {
    byte_value = receiveByte();
    if (byte_value < 0) {
      Serial.print("Error receiving data byte ");
      Serial.println(i);
      digitalWrite(LED_BUILTIN, LOW);
      return false;
    }
    
    rx_buffer[rx_buffer_pos++] = byte_value;
  }
  
  // Null-terminate the string
  rx_buffer[rx_buffer_pos] = '\0';
  
  // 4. Receive end byte
  byte_value = receiveByte();
  if (byte_value < 0) {
    Serial.println("Error receiving end byte");
    digitalWrite(LED_BUILTIN, LOW);
    return false;
  }
  
  if (byte_value != END_BYTE) {
    Serial.print("Invalid end byte: 0x");
    if (byte_value < 16) Serial.print("0");
    Serial.print(byte_value, HEX);
    Serial.print(", expected 0x");
    if (END_BYTE < 16) Serial.print("0");
    Serial.println(END_BYTE, HEX);
    success = false;
  } else {
    Serial.print("Message received successfully: \"");
    Serial.print(rx_buffer);
    Serial.println("\"");
    success = true;
  }
  
  // Turn off LED
  digitalWrite(LED_BUILTIN, LOW);
  
  return success;
}

// Detect clock activity (for detecting incoming transmission)
bool clockActivityDetected() {
  static int lastClockState = -1;
  int currentClockState = readClockPin();
  bool detected = false;
  
  // Initialize on first call
  if (lastClockState == -1) {
    lastClockState = currentClockState;
    return false;
  }
  
  // Check for any change in clock state
  if (lastClockState != currentClockState) {
    detected = true;
  }
  
  lastClockState = currentClockState;
  return detected;
}

void loop() {
  // Check for clock activity (which indicates DE1-SOC is sending data)
  if (clockActivityDetected()) {
    Serial.println("Clock activity detected - preparing to receive");
    
    // Give the DE1-SOC time to prepare the first data bit
    delay(BIT_PERIOD_MS / 2);
    
    // Try to receive a message
    receiveMessage();
    
    lastActivityTime = millis();
  }
  
  // Check if button is pressed to send test message
  int reading = digitalRead(BUTTON_PIN);
  
  // Debounce button
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // If button state has changed and it's pressed (LOW due to pull-up)
    if (reading == LOW && lastButtonState == HIGH) {
      Serial.println("\nButton pressed - Sending test message");
      
      // Create test message
      sprintf(tx_buffer, "ARD_MSG_%d", message_counter++);
      
      // Send it
      sendMessage(tx_buffer);
      lastActivityTime = millis();
    }
  }
  
  lastButtonState = reading;
  
  // If it's been 5 seconds since last activity, print a heartbeat message
  if (millis() - lastActivityTime > 5000) {
    Serial.println("Waiting for activity...");
    lastActivityTime = millis();
  }
  
  // Small delay
  delay(10);
}