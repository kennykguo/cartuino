/*
 * Arduino Bidirectional Communication Protocol (Master)
 * Uses handshake synchronization for reliable operation
 * D2 = Data, D3 = Clock, D4 = Ready/Request (new handshake line)
 */

// Pin definitions
#define DATA_PIN 2    // D2 connects to DE1-SoC D0
#define CLOCK_PIN 3   // D3 connects to DE1-SoC D1
#define READY_PIN 4   // D4 connects to DE1-SoC D2 (new handshake line)

// Communication parameters
#define BIT_PERIOD 5        // 5ms per bit - faster than before
#define MSG_BUFFER_SIZE 32  // Buffer size for messages (20 chars + protocol bytes)
char tx_buffer[MSG_BUFFER_SIZE];
char rx_buffer[MSG_BUFFER_SIZE];

// Protocol constants
#define START_BYTE 0xAA  // 10101010 pattern
#define END_BYTE 0x55    // 01010101 pattern
#define ACK_BYTE 0xCC    // 11001100 pattern

// State tracking
int message_counter = 0;
bool waiting_for_response = false;
unsigned long last_activity_time = 0;
unsigned long message_interval = 5000;  // Auto-send every 5 seconds

void setup() {
  // Setup serial for debugging
  Serial.begin(9600);
  
  // Configure pins
  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(READY_PIN, OUTPUT);
  
  // Initialize pins to known states
  digitalWrite(CLOCK_PIN, HIGH);
  digitalWrite(DATA_PIN, LOW);
  digitalWrite(READY_PIN, LOW); // Default: not requesting attention
  
  // Configure LED
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Ready indicator
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
    delay(50);
  }
  
  Serial.println("\n===== Arduino Master Communication Protocol =====");
  Serial.println("Using D2 (data), D3 (clock), D4 (ready/request)");
  Serial.println("Automatic bidirectional communication enabled");
  Serial.println("==============================================");
}

// Assert READY signal to request attention from FPGA
bool requestAttention(unsigned long timeout_ms = 500) {
  unsigned long start_time = millis();
  
  // Signal that we want to initiate communication
  digitalWrite(READY_PIN, HIGH);
  
  // Wait until DE1-SoC acknowledges by pulling READY line LOW
  // (DE1-SoC will set its D2 as INPUT, which allows pullup to work)
  while (digitalRead(READY_PIN) == HIGH) {
    if (millis() - start_time > timeout_ms) {
      // Timeout expired
      digitalWrite(READY_PIN, LOW); // Reset signal
      Serial.println("Timeout waiting for FPGA to acknowledge ready signal");
      return false;
    }
    delay(1);
  }
  
  // DE1-SoC has acknowledged
  return true;
}

// Detect if DE1-SoC is requesting our attention
bool fpgaRequestingAttention() {
  // If READY line is LOW (pulled down by DE1-SoC), it wants to send data
  return digitalRead(READY_PIN) == LOW;
}

// Acknowledge DE1-SoC's request by setting READY as INPUT
void acknowledgeRequest() {
  pinMode(READY_PIN, INPUT_PULLUP);
  delay(1); // Small delay for stability
}

// Reset READY line to normal state after communication
void resetReadyLine() {
  pinMode(READY_PIN, OUTPUT);
  digitalWrite(READY_PIN, LOW);
  delay(1);
}

// Send a single bit
void sendBit(int bit) {
  digitalWrite(DATA_PIN, bit ? HIGH : LOW);
  
  // Brief delay for data to stabilize
  delayMicroseconds(200);
  
  // Clock cycle
  digitalWrite(CLOCK_PIN, LOW);
  delayMicroseconds(BIT_PERIOD * 500); // Half period
  
  digitalWrite(CLOCK_PIN, HIGH);
  delayMicroseconds(BIT_PERIOD * 500); // Half period
}

// Send a byte (8 bits) MSB first
void sendByte(unsigned char byte) {
  Serial.print("TX: 0x");
  if (byte < 16) Serial.print("0");
  Serial.println(byte, HEX);
  
  for (int i = 7; i >= 0; i--) {
    int bit = (byte >> i) & 0x01;
    sendBit(bit);
  }
}

// Receive a bit by waiting for clock transitions
int receiveBit() {
  int bit;
  
  // Set data pin as input
  pinMode(DATA_PIN, INPUT);
  
  // Wait for clock to go LOW
  while (digitalRead(CLOCK_PIN) != LOW) {
    delayMicroseconds(100);
  }
  
  // Brief delay for data to stabilize
  delayMicroseconds(100);
  
  // Read data bit
  bit = digitalRead(DATA_PIN);
  
  // Wait for clock to go HIGH
  while (digitalRead(CLOCK_PIN) != HIGH) {
    delayMicroseconds(100);
  }
  
  return bit;
}

// Receive a byte (8 bits) MSB first
unsigned char receiveByte() {
  unsigned char byte = 0;
  
  for (int i = 7; i >= 0; i--) {
    int bit = receiveBit();
    byte |= (bit << i);
  }
  
  Serial.print("RX: 0x");
  if (byte < 16) Serial.print("0");
  Serial.println(byte, HEX);
  
  return byte;
}

// Send a complete message
bool sendMessage(char *message) {
  int len = strlen(message);
  unsigned char checksum = 0;
  
  digitalWrite(LED_BUILTIN, HIGH); // Indicate transmission
  
  Serial.print("Sending: \"");
  Serial.print(message);
  Serial.println("\"");
  
  // Request attention from DE1-SoC
  if (!requestAttention(500)) {
    digitalWrite(LED_BUILTIN, LOW);
    return false;
  }
  
  // Switch DATA pin to output mode
  pinMode(DATA_PIN, OUTPUT);
  
  // Send start byte
  sendByte(START_BYTE);
  
  // Send length byte
  sendByte((unsigned char)len);
  checksum ^= len;
  
  // Send message bytes
  for (int i = 0; i < len; i++) {
    sendByte((unsigned char)message[i]);
    checksum ^= message[i];
  }
  
  // Send checksum
  sendByte(checksum);
  
  // Send end byte
  sendByte(END_BYTE);
  
  // Wait for ACK from DE1-SoC
  pinMode(DATA_PIN, INPUT);
  unsigned char response = receiveByte();
  
  // Reset ready line
  resetReadyLine();
  
  digitalWrite(LED_BUILTIN, LOW); // End transmission indication
  
  return (response == ACK_BYTE);
}

// Receive a complete message from DE1-SoC
bool receiveMessage() {
  unsigned char byte, length, checksum = 0, calculated_checksum = 0;
  bool success = false;
  
  digitalWrite(LED_BUILTIN, HIGH); // Indicate reception
  
  Serial.println("DE1-SoC wants to send data...");
  
  // Acknowledge the request
  acknowledgeRequest();
  
  // Switch to input mode
  pinMode(DATA_PIN, INPUT);
  
  // Check for start byte
  byte = receiveByte();
  if (byte != START_BYTE) {
    Serial.println("Invalid start byte!");
    resetReadyLine();
    digitalWrite(LED_BUILTIN, LOW);
    return false;
  }
  
  // Get length byte
  length = receiveByte();
  calculated_checksum ^= length;
  
  // Validate length
  if (length >= MSG_BUFFER_SIZE || length == 0) {
    Serial.println("Invalid message length!");
    resetReadyLine();
    digitalWrite(LED_BUILTIN, LOW);
    return false;
  }
  
  // Get message bytes
  int pos = 0;
  for (int i = 0; i < length; i++) {
    byte = receiveByte();
    rx_buffer[pos++] = byte;
    calculated_checksum ^= byte;
  }
  
  // Null-terminate
  rx_buffer[pos] = '\0';
  
  // Get checksum
  checksum = receiveByte();
  
  // Get end byte
  byte = receiveByte();
  
  // Verify message
  if (byte != END_BYTE) {
    Serial.println("Invalid end byte!");
    success = false;
  } else if (checksum != calculated_checksum) {
    Serial.println("Checksum error!");
    success = false;
  } else {
    Serial.print("Received: \"");
    Serial.print(rx_buffer);
    Serial.println("\"");
    success = true;
  }
  
  // Send ACK
  pinMode(DATA_PIN, OUTPUT);
  sendByte(success ? ACK_BYTE : 0x33); // ACK or NACK
  
  // Reset ready line
  resetReadyLine();
  
  digitalWrite(LED_BUILTIN, LOW); // End reception indication
  
  return success;
}

void loop() {
  unsigned long current_time = millis();
  
  // Check if DE1-SoC is requesting to send us data
  if (fpgaRequestingAttention()) {
    if (receiveMessage()) {
      Serial.println("Message received successfully");
    } else {
      Serial.println("Message reception failed");
    }
    last_activity_time = current_time;
  }
  
  // Periodically send test messages
  if (current_time - last_activity_time >= message_interval) {
    // Create test message
    sprintf(tx_buffer, "ARD_MSG_%d", message_counter++);
    
    // Send it
    if (sendMessage(tx_buffer)) {
      Serial.println("Message sent successfully");
    } else {
      Serial.println("Message transmission failed");
    }
    
    last_activity_time = current_time;
  }
  
  // Small delay to prevent CPU hogging
  delay(1);
}