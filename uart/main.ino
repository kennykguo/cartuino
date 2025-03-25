/*
 * Arduino Feather 32u4 Asynchronous Communication Protocol
 * For communication with DE1-SoC FPGA
 * 
 * Uses a simple asynchronous protocol with handshaking 
 * to ensure reliable communication despite clock differences
 */

// Pin definitions
#define DATA_PIN 0      // D0 for data line
#define HANDSHAKE_PIN 1 // D1 for handshake line
#define BUTTON_PIN 2    // Button for sending test messages

// Communication parameters
#define BIT_DELAY_MS 50       // Bit transmission delay
#define START_BITS 2          // Number of start bits (1-1)
#define STOP_BITS 2           // Number of stop bits (0-0)
#define TIMEOUT_MS 5000       // Timeout in milliseconds
#define MSG_BUFFER_SIZE 64    // Buffer size for messages

char tx_buffer[MSG_BUFFER_SIZE];
char rx_buffer[MSG_BUFFER_SIZE];
int message_counter = 0;

// Button debounce variables
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

// Communication timestamp
unsigned long lastActivityTime = 0;

void setup() {
  // Setup USB serial for debugging output
  Serial.begin(9600);
  
  // Configure pins
  pinMode(DATA_PIN, INPUT);        // Start as input
  pinMode(HANDSHAKE_PIN, INPUT);   // Start as input
  pinMode(BUTTON_PIN, INPUT_PULLUP); // Button with pull-up
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Initial LED flash to indicate ready
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  
  Serial.println("\n\n");
  Serial.println("==============================================");
  Serial.println("Arduino Feather Asynchronous Communication Protocol");
  Serial.println("==============================================");
  Serial.println("Controls:");
  Serial.println("- Button on pin 2: Send test message to DE1-SoC");
  Serial.println("- Will automatically receive messages from DE1-SoC");
  Serial.println("==============================================");
}

// Wait for handshake pin to reach desired state with timeout
// Returns true on success, false on timeout
bool waitForHandshake(int desiredState, unsigned long timeoutMs) {
  unsigned long startTime = millis();
  
  while (digitalRead(HANDSHAKE_PIN) != desiredState) {
    // Check for timeout
    if ((millis() - startTime) > timeoutMs) {
      Serial.print("Handshake timeout waiting for ");
      Serial.print(desiredState);
      Serial.println(" state");
      return false;
    }
    // Small delay to prevent CPU hogging
    delay(1);
  }
  return true;
}

// Send a single bit
void sendBit(int bit) {
  // Configure pins for sending
  pinMode(DATA_PIN, OUTPUT);
  pinMode(HANDSHAKE_PIN, OUTPUT);
  
  // Set the data pin to the bit value
  digitalWrite(DATA_PIN, bit);
  
  // Signal that data is ready by setting handshake high
  digitalWrite(HANDSHAKE_PIN, HIGH);
  
  // Wait for receiver to acknowledge by setting their handshake pin high
  pinMode(HANDSHAKE_PIN, INPUT);
  if (!waitForHandshake(HIGH, TIMEOUT_MS)) {
    Serial.println("Timeout waiting for bit acknowledgment");
    return;
  }
  
  // Reset our handshake line to low to complete the bit transmission
  pinMode(HANDSHAKE_PIN, OUTPUT);
  digitalWrite(HANDSHAKE_PIN, LOW);
  
  // Wait for receiver to reset their handshake pin
  pinMode(HANDSHAKE_PIN, INPUT);
  if (!waitForHandshake(LOW, TIMEOUT_MS)) {
    Serial.println("Timeout waiting for bit completion");
    return;
  }
  
  // Small delay before next bit
  delay(BIT_DELAY_MS);
}

// Receive a single bit
int receiveBit() {
  int bit;
  
  // Configure pins for receiving
  pinMode(DATA_PIN, INPUT);
  pinMode(HANDSHAKE_PIN, OUTPUT);
  digitalWrite(HANDSHAKE_PIN, LOW);  // Initialize to low
  
  // Wait for sender to signal data is ready
  pinMode(HANDSHAKE_PIN, INPUT);
  if (!waitForHandshake(HIGH, TIMEOUT_MS)) {
    Serial.println("Timeout waiting for bit start");
    return -1;  // Error condition
  }
  
  // Read the data bit
  bit = digitalRead(DATA_PIN);
  
  // Acknowledge receipt by setting handshake high
  pinMode(HANDSHAKE_PIN, OUTPUT);
  digitalWrite(HANDSHAKE_PIN, HIGH);
  
  // Wait for sender to reset their handshake
  pinMode(HANDSHAKE_PIN, INPUT);
  if (!waitForHandshake(LOW, TIMEOUT_MS)) {
    Serial.println("Timeout waiting for bit reset");
    return -1;  // Error condition
  }
  
  // Reset our handshake line
  pinMode(HANDSHAKE_PIN, OUTPUT);
  digitalWrite(HANDSHAKE_PIN, LOW);
  
  return bit;
}

// Send a byte, LSB first with start and stop bits
void sendByte(unsigned char byte) {
  Serial.print("Sending byte: 0x");
  if (byte < 16) Serial.print("0");
  Serial.println(byte, HEX);
  
  // Send start bits (1-1)
  for (int i = 0; i < START_BITS; i++) {
    sendBit(1);
  }
  
  // Send 8 data bits, LSB first
  for (int i = 0; i < 8; i++) {
    int bit = (byte >> i) & 0x01;
    sendBit(bit);
  }
  
  // Send stop bits (0-0)
  for (int i = 0; i < STOP_BITS; i++) {
    sendBit(0);
  }
}

// Receive a byte with start and stop bits
// Returns -1 on error
int receiveByte() {
  int startBits[START_BITS];
  int dataBits[8];
  int stopBits[STOP_BITS];
  unsigned char byte = 0;
  
  // Read start bits
  for (int i = 0; i < START_BITS; i++) {
    startBits[i] = receiveBit();
    if (startBits[i] != 1) {
      Serial.print("Invalid start bit ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(startBits[i]);
      return -1;
    }
  }
  
  // Read 8 data bits, LSB first
  for (int i = 0; i < 8; i++) {
    dataBits[i] = receiveBit();
    if (dataBits[i] < 0) {
      Serial.print("Error receiving data bit ");
      Serial.println(i);
      return -1;
    }
    byte |= (dataBits[i] << i);
  }
  
  // Read stop bits
  for (int i = 0; i < STOP_BITS; i++) {
    stopBits[i] = receiveBit();
    if (stopBits[i] != 0) {
      Serial.print("Invalid stop bit ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(stopBits[i]);
      return -1;
    }
  }
  
  Serial.print("Received byte: 0x");
  if (byte < 16) Serial.print("0");
  Serial.println(byte, HEX);
  
  return byte;
}

// Calculate simple checksum
unsigned char calculateChecksum(char *data, int length) {
  unsigned char checksum = 0;
  for (int i = 0; i < length; i++) {
    checksum ^= data[i];  // XOR checksum
  }
  return checksum;
}

// Send a message
void sendMessage(char *message) {
  int len = strlen(message);
  unsigned char checksum = calculateChecksum(message, len);
  
  // Turn on LED during transmission
  digitalWrite(LED_BUILTIN, HIGH);
  
  Serial.print("Sending message: \"");
  Serial.print(message);
  Serial.print("\" (");
  Serial.print(len);
  Serial.println(" bytes)");
  
  // Send length byte
  sendByte((unsigned char)len);
  
  // Send each character
  for (int i = 0; i < len; i++) {
    sendByte((unsigned char)message[i]);
  }
  
  // Send checksum
  sendByte(checksum);
  
  // Wait for acknowledgment (ACK/NACK)
  int response = receiveByte();
  if (response == 0xCC) {  // ACK
    Serial.println("Message successfully delivered (ACK received)");
  } else {
    Serial.print("Message delivery failed (NACK or invalid response: 0x");
    if (response < 16) Serial.print("0");
    Serial.print(response, HEX);
    Serial.println(")");
  }
  
  // Turn off LED
  digitalWrite(LED_BUILTIN, LOW);
}

// Receive a message
bool receiveMessage() {
  unsigned char length, checksum, calculatedChecksum;
  int i, success = false;
  
  // Turn on LED during reception
  digitalWrite(LED_BUILTIN, HIGH);
  
  Serial.println("Waiting to receive message...");
  
  // Get length byte
  int lenByte = receiveByte();
  if (lenByte < 0 || lenByte >= MSG_BUFFER_SIZE) {
    Serial.print("Invalid message length: ");
    Serial.println(lenByte);
    sendByte(0x33);  // NACK
    digitalWrite(LED_BUILTIN, LOW);
    return false;
  }
  
  length = (unsigned char)lenByte;
  Serial.print("Expecting message of ");
  Serial.print(length);
  Serial.println(" bytes");
  
  // Get message data
  for (i = 0; i < length; i++) {
    int byte = receiveByte();
    if (byte < 0) {
      Serial.println("Error receiving message data");
      sendByte(0x33);  // NACK
      digitalWrite(LED_BUILTIN, LOW);
      return false;
    }
    rx_buffer[i] = (char)byte;
  }
  
  // Null-terminate the string
  rx_buffer[length] = '\0';
  
  // Get checksum
  int checksumByte = receiveByte();
  if (checksumByte < 0) {
    Serial.println("Error receiving checksum");
    sendByte(0x33);  // NACK
    digitalWrite(LED_BUILTIN, LOW);
    return false;
  }
  
  checksum = (unsigned char)checksumByte;
  calculatedChecksum = calculateChecksum(rx_buffer, length);
  
  if (checksum != calculatedChecksum) {
    Serial.print("Checksum error: received 0x");
    if (checksum < 16) Serial.print("0");
    Serial.print(checksum, HEX);
    Serial.print(", calculated 0x");
    if (calculatedChecksum < 16) Serial.print("0");
    Serial.println(calculatedChecksum, HEX);
    sendByte(0x33);  // NACK
    success = false;
  } else {
    Serial.print("Message received successfully: \"");
    Serial.print(rx_buffer);
    Serial.println("\"");
    sendByte(0xCC);  // ACK
    success = true;
  }
  
  // Turn off LED
  digitalWrite(LED_BUILTIN, LOW);
  
  return success;
}

// Check if a handshake signal is detected (rising edge on handshake pin)
bool handshakeDetected() {
  static int lastHandshakeState = LOW;
  int currentHandshakeState = digitalRead(HANDSHAKE_PIN);
  bool detected = false;
  
  if (lastHandshakeState == LOW && currentHandshakeState == HIGH) {
    detected = true;
  }
  
  lastHandshakeState = currentHandshakeState;
  return detected;
}

void loop() {
  // Check for button press to send test message
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
  
  // Check for incoming message (detect handshake signal)
  if (handshakeDetected()) {
    // Reset handshake detection by briefly going into output mode
    pinMode(HANDSHAKE_PIN, OUTPUT);
    digitalWrite(HANDSHAKE_PIN, LOW);
    pinMode(HANDSHAKE_PIN, INPUT);
    
    receiveMessage();
    lastActivityTime = millis();
  }
  
  // If it's been 5 seconds since last activity, print a heartbeat message
  if (millis() - lastActivityTime > 5000) {
    Serial.println("Waiting for activity...");
    lastActivityTime = millis();
  }
  
  // Small delay
  delay(10);
}