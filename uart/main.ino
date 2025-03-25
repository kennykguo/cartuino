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
#define BIT_PERIOD_MS 20   // 20ms per bit
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
  Serial.begin(9600);
  
  // Configure DATA and CLOCK pins
  pinMode(DATA_PIN, INPUT); // Default to input mode
  pinMode(CLOCK_PIN, INPUT); // Default to input mode
  
  // Configure built-in LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  // Configure button with pull-up resistor
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  // Initial LED flash to show we're ready
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  
  Serial.println("\n\n");
  Serial.println("==============================================");
  Serial.println("Arduino Reliable Communication Protocol Ready");
  Serial.println("==============================================");
  Serial.println("Controls:");
  Serial.println("- Button on pin 2: Send test message to DE1-SoC");
  Serial.println("- Will automatically receive messages from DE1-SoC");
  Serial.println("==============================================");
}

// Set the DATA pin direction
void setDataPinMode(int mode) {
  pinMode(DATA_PIN, mode);
  // Small delay to ensure mode change takes effect
  delayMicroseconds(100);
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

// Wait for CLOCK to transition with timeout
bool waitForClock(bool waitForHigh, unsigned long timeoutMs) {
  unsigned long startTime = millis();
  
  while (digitalRead(CLOCK_PIN) != (waitForHigh ? HIGH : LOW)) {
    if (millis() - startTime > timeoutMs) {
      Serial.print("Clock timeout waiting for ");
      Serial.println(waitForHigh ? "HIGH" : "LOW");
      return false;
    }
    delayMicroseconds(100); // Short delay while polling
  }
  
  return true;
}

// Send a single bit with clock monitoring and timeout
bool sendBit(int bit) {
  // Set the data value
  setDataPin(bit);
  
  // Wait for clock to go LOW (DE1-SoC signals ready to read)
  if (!waitForClock(false, 100)) {
    return false;
  }
  
  // Wait for clock to go HIGH again (bit transmission complete)
  if (!waitForClock(true, 100)) {
    return false;
  }
  
  return true;
}

// Send a byte (8 bits) MSB first
bool sendByte(unsigned char byte) {
  // Set DATA pin as output
  setDataPinMode(OUTPUT);
  
  Serial.print("Sending byte: 0x");
  if (byte < 16) Serial.print("0");
  Serial.println(byte, HEX);
  
  // Send each bit, MSB first
  for (int i = 7; i >= 0; i--) {
    int bit = (byte >> i) & 0x01;
    if (!sendBit(bit)) {
      Serial.print("Failed to send bit ");
      Serial.println(i);
      return false;
    }
  }
  
  return true;
}

// Receive a single bit by monitoring clock transitions
int receiveBit() {
  int bit;
  
  // Wait for clock to go LOW (indicates data is ready)
  if (!waitForClock(false, 100)) {
    return -1; // Error
  }
  
  // Read data bit
  bit = readDataPin();
  
  // Wait for clock to go HIGH again (end of bit)
  if (!waitForClock(true, 100)) {
    return -1; // Error
  }
  
  return bit;
}

// Receive a byte (8 bits) MSB first
int receiveByte() {
  unsigned char byte = 0;
  
  // Set DATA pin as input
  setDataPinMode(INPUT);
  
  // Receive 8 bits, MSB first
  for (int i = 7; i >= 0; i--) {
    int bit = receiveBit();
    if (bit < 0) {
      Serial.print("Failed to receive bit ");
      Serial.println(i);
      return -1; // Error
    }
    byte |= (bit << i);
  }
  
  Serial.print("Received byte: 0x");
  if (byte < 16) Serial.print("0");
  Serial.println(byte, HEX);
  
  return byte;
}

// Signal to DE1-SoC that Arduino is ready to receive
void signalReadiness() {
  // Set DATA as output
  setDataPinMode(OUTPUT);
  
  // Set DATA high to indicate readiness
  setDataPin(HIGH);
  
  // Small delay for stability
  delay(10);
}

// Send a message with protocol framing
bool sendMessage(char *message) {
  int len = strlen(message);
  unsigned char checksum = 0;
  int response;
  
  // Turn on LED during transmission
  digitalWrite(LED_BUILTIN, HIGH);
  
  Serial.print("Sending message: \"");
  Serial.print(message);
  Serial.print("\" (");
  Serial.print(len);
  Serial.println(" bytes)");
  
  // Signal readiness to DE1-SoC before sending
  signalReadiness();
  
  // 1. Send start sequence for synchronization
  if (!sendByte(START_SEQUENCE)) {
    Serial.println("Failed to send start sequence");
    digitalWrite(LED_BUILTIN, LOW);
    return false;
  }
  
  // 2. Send length byte
  if (!sendByte((unsigned char)len)) {
    Serial.println("Failed to send length");
    digitalWrite(LED_BUILTIN, LOW);
    return false;
  }
  checksum ^= len; // XOR for simple checksum
  
  // 3. Send each byte of the message
  for (int i = 0; i < len; i++) {
    if (!sendByte((unsigned char)message[i])) {
      Serial.print("Failed to send byte ");
      Serial.println(i);
      digitalWrite(LED_BUILTIN, LOW);
      return false;
    }
    checksum ^= message[i]; // Update checksum
  }
  
  // 4. Send checksum
  if (!sendByte(checksum)) {
    Serial.println("Failed to send checksum");
    digitalWrite(LED_BUILTIN, LOW);
    return false;
  }
  
  // 5. Send end sequence
  if (!sendByte(END_SEQUENCE)) {
    Serial.println("Failed to send end sequence");
    digitalWrite(LED_BUILTIN, LOW);
    return false;
  }
  
  // 6. Wait for ACK from receiver
  setDataPinMode(INPUT);
  response = receiveByte();
  
  if (response < 0) {
    Serial.println("Error receiving ACK");
    digitalWrite(LED_BUILTIN, LOW);
    return false;
  } else if (response == ACK_BYTE) {
    Serial.println("Received ACK - message successfully delivered");
    digitalWrite(LED_BUILTIN, LOW);
    return true;
  } else {
    Serial.print("Did not receive ACK - got 0x");
    if (response < 16) Serial.print("0");
    Serial.print(response, HEX);
    Serial.println(" instead");
    digitalWrite(LED_BUILTIN, LOW);
    return false;
  }
}

// Receive a message with protocol framing
int receiveMessage() {
  int byte_value;
  unsigned char length, checksum = 0, calculated_checksum = 0;
  int i, success = 0;
  unsigned long startTime;
  
  // Turn on LED during reception
  digitalWrite(LED_BUILTIN, HIGH);
  
  Serial.println("Preparing to receive message");
  
  // Signal to DE1-SOC that we're ready to receive
  signalReadiness();
  
  // Set DATA pin as input to receive data
  setDataPinMode(INPUT);
  
  // 1. Wait for and verify start sequence (with timeout)
  startTime = millis();
  while (true) {
    // Check for timeout
    if (millis() - startTime > 3000) { // 3 second timeout
      Serial.println("Timeout waiting for start sequence");
      digitalWrite(LED_BUILTIN, LOW);
      return 0;
    }
    
    byte_value = receiveByte();
    if (byte_value < 0) {
      // Error receiving byte, retry
      delay(50);
      continue;
    } else if (byte_value == START_SEQUENCE) {
      // Found start sequence
      break;
    } else {
      Serial.print("Received unexpected byte: 0x");
      if (byte_value < 16) Serial.print("0");
      Serial.println(byte_value, HEX);
      delay(50);
      // Keep waiting for start sequence
    }
  }
  
  // 2. Receive length byte
  byte_value = receiveByte();
  if (byte_value < 0) {
    Serial.println("Error receiving length byte");
    digitalWrite(LED_BUILTIN, LOW);
    return 0;
  }
  
  length = byte_value;
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
    byte_value = receiveByte();
    if (byte_value < 0) {
      Serial.print("Error receiving data byte ");
      Serial.println(i);
      digitalWrite(LED_BUILTIN, LOW);
      return 0;
    }
    
    rx_buffer[rx_buffer_pos++] = byte_value;
    calculated_checksum ^= byte_value;
  }
  
  // Null-terminate the string
  rx_buffer[rx_buffer_pos] = '\0';
  
  // 4. Receive checksum
  byte_value = receiveByte();
  if (byte_value < 0) {
    Serial.println("Error receiving checksum");
    digitalWrite(LED_BUILTIN, LOW);
    return 0;
  }
  
  checksum = byte_value;
  
  // 5. Receive end sequence
  byte_value = receiveByte();
  if (byte_value < 0) {
    Serial.println("Error receiving end sequence");
    digitalWrite(LED_BUILTIN, LOW);
    return 0;
  }
  
  if (byte_value != END_SEQUENCE) {
    Serial.print("Invalid end sequence: 0x");
    if (byte_value < 16) Serial.print("0");
    Serial.print(byte_value, HEX);
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
  
  // 6. Send ACK or NACK based on success
  setDataPinMode(OUTPUT);
  if (success) {
    sendByte(ACK_BYTE);
  } else {
    sendByte(NACK_BYTE);
  }
  
  // Return to input mode by default
  setDataPinMode(INPUT);
  
  // Turn off LED
  digitalWrite(LED_BUILTIN, LOW);
  
  return success;
}

// Check if a CLOCK falling edge is detected
bool clockFallingEdgeDetected() {
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
  // Check for incoming message from DE1-SoC (detect activity)
  if (clockFallingEdgeDetected()) {
    Serial.println("Detected clock activity - preparing to receive");
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
  // and signal readiness to the DE1-SOC
  if (millis() - lastActivityTime > 5000) {
    Serial.println("Waiting for activity...");
    lastActivityTime = millis();
    
    // Periodically signal readiness to DE1-SOC
    signalReadiness();
    delay(10);
    setDataPinMode(INPUT);
  }
  
  // Small delay
  delay(10);
}