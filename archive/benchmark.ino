/*
 * Arduino Bidirectional Communication Protocol
 * Using explicit bit-banging with master-slave roles
 * D2 - Data (bidirectional with DE1-SoC D0)
 * D3 - Clock (to DE1-SoC D1)
 * D5 - Sync (to DE1-SoC D2)
 */

// Pin definitions
#define DATA_PIN 2   // For bidirectional data line
#define CLOCK_PIN 3  // For clock line
#define SYNC_PIN 5   // For sync/handshake line (D5)

// Communication parameters
#define BIT_PERIOD_MS 5     // Slower but more reliable (5ms per bit)
#define SETUP_DELAY_US 500  // Longer setup time
#define SYNC_PULSE_MS 20    // Longer sync pulse
#define MSG_BUFFER_SIZE 32  // Buffer size for messages
#define START_BYTE 0xAA     // 10101010 pattern for sync

// Message buffers and counter
char tx_buffer[MSG_BUFFER_SIZE];
char rx_buffer[MSG_BUFFER_SIZE];
int message_counter = 0;

void setup() {
  Serial.begin(19200);
  
  // Configure pins with explicit modes
  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(SYNC_PIN, OUTPUT);
  
  // Initialize pins to known states
  digitalWrite(DATA_PIN, LOW);
  digitalWrite(CLOCK_PIN, HIGH);
  digitalWrite(SYNC_PIN, LOW);
  
  // Configure built-in LED
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Ready indicator
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
  
  Serial.println("\nBidirectional Communication Protocol Ready");
}

// Send a single bit with explicit timing
void sendBit(int bit) {
  // Set data bit value first and hold it stable
  pinMode(DATA_PIN, OUTPUT);
  digitalWrite(DATA_PIN, bit ? HIGH : LOW);
  delayMicroseconds(SETUP_DELAY_US);
  
  // Toggle clock LOW (active, FPGA reads on this edge)
  digitalWrite(CLOCK_PIN, LOW);
  delay(BIT_PERIOD_MS / 2);
  
  // Return clock HIGH (inactive)
  digitalWrite(CLOCK_PIN, HIGH);
  delay(BIT_PERIOD_MS / 2);
}

// Receive a single bit
int receiveBit() {
  int bit;
  
  // First set data pin as input
  pinMode(DATA_PIN, INPUT);
  
  // Signal ready to receive with clock HIGH
  digitalWrite(CLOCK_PIN, HIGH);
  delay(5);
  
  // Toggle clock LOW to signal ready to read
  digitalWrite(CLOCK_PIN, LOW);
  delay(BIT_PERIOD_MS); // Wait for FPGA to set data
  
  // Read the bit
  bit = digitalRead(DATA_PIN);
  
  // Return clock to HIGH
  digitalWrite(CLOCK_PIN, HIGH);
  delay(5);
  
  return bit;
}

// Send a byte (8 bits) MSB first
void sendByte(unsigned char byte) {
  Serial.print("TX: 0x");
  if (byte < 16) Serial.print("0");
  Serial.println(byte, HEX);
  
  // Set data pin as output
  pinMode(DATA_PIN, OUTPUT);
  
  // Send each bit, MSB first
  for (int i = 7; i >= 0; i--) {
    int bit = (byte >> i) & 0x01;
    sendBit(bit);
  }
  
  // Pause between bytes
  delay(10);
}

// Receive a byte (8 bits) MSB first
unsigned char receiveByte() {
  unsigned char byte = 0;
  
  // Receive 8 bits, MSB first
  for (int i = 7; i >= 0; i--) {
    int bit = receiveBit();
    byte |= (bit << i);
  }
  
  Serial.print("RX: 0x");
  if (byte < 16) Serial.print("0");
  Serial.println(byte, HEX);
  
  return byte;
}

// Send a message with sync protocol
void sendMessage() {
  // Create test message
  sprintf(tx_buffer, "ARD_MSG_%d", message_counter++);
  int len = strlen(tx_buffer);
  
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.print("Sending: \""); Serial.print(tx_buffer); Serial.println("\"");
  
  // Strong sync sequence
  digitalWrite(SYNC_PIN, HIGH);
  delay(SYNC_PULSE_MS); // Longer sync pulse
  
  // Reset clock state and wait
  digitalWrite(CLOCK_PIN, HIGH);
  delay(10);
  
  // Send distinctive preamble pattern
  // Send 0xAA, 0x55, 0xAA sequence (alternating pattern)
  sendByte(0xAA);
  delay(5);
  sendByte(0x55);
  delay(5);
  sendByte(0xAA);
  delay(5);
  
  // Send actual message
  sendByte(START_BYTE);  // Final start byte
  sendByte((unsigned char)len);
  
  // Send message bytes
  for (int i = 0; i < len; i++) {
    sendByte((unsigned char)tx_buffer[i]);
  }
  
  Serial.println("Message sent");
}

// Check for and receive response from DE1-SoC
bool receiveResponse() {
  unsigned char byte;
  unsigned long startTime = millis();
  
  // Reset state and wait for response signal
  pinMode(DATA_PIN, INPUT);
  
  // Wait for FPGA to pull DATA high (response ready)
  while (digitalRead(DATA_PIN) == LOW) {
    if (millis() - startTime > RESPONSE_WAIT_MS) {
      Serial.println("No response");
      digitalWrite(SYNC_PIN, LOW);
      return false;
    }
    delayMicroseconds(100);
  }
  
  Serial.println("Response detected");
  
  // Quick clock pulse acknowledgment
  digitalWrite(CLOCK_PIN, LOW);
  delayMicroseconds(500);
  digitalWrite(CLOCK_PIN, HIGH);
  delayMicroseconds(500);
  
  // Read response with active clock control
  // First, look for start byte pattern (multiple attempts)
  for (int attempt = 0; attempt < 3; attempt++) {
    // Read a byte with active clock control
    byte = 0;
    for (int i = 7; i >= 0; i--) {
      digitalWrite(CLOCK_PIN, HIGH);
      delayMicroseconds(SETUP_DELAY_US);
      digitalWrite(CLOCK_PIN, LOW);
      delayMicroseconds(BIT_PERIOD_MS * 500);
      int bit = digitalRead(DATA_PIN);
      byte |= (bit << i);
      digitalWrite(CLOCK_PIN, HIGH);
      delayMicroseconds(BIT_PERIOD_MS * 500);
    }
    
    Serial.print("RX: 0x");
    if (byte < 16) Serial.print("0");
    Serial.println(byte, HEX);
    
    if (byte == START_BYTE) {
      // Found valid start byte, proceed with message
      break;
    }
    
    if (attempt == 2) {
      Serial.println("Failed to find valid start byte");
      digitalWrite(SYNC_PIN, LOW);
      digitalWrite(LED_BUILTIN, LOW);
      return false;
    }
  }
  
  // Read length
  byte = 0;
  for (int i = 7; i >= 0; i--) {
    digitalWrite(CLOCK_PIN, HIGH);
    delayMicroseconds(SETUP_DELAY_US);
    digitalWrite(CLOCK_PIN, LOW);
    delayMicroseconds(BIT_PERIOD_MS * 500);
    int bit = digitalRead(DATA_PIN);
    byte |= (bit << i);
    digitalWrite(CLOCK_PIN, HIGH);
    delayMicroseconds(BIT_PERIOD_MS * 500);
  }
  
  int length = byte;
  if (length >= MSG_BUFFER_SIZE || length == 0) {
    Serial.println("Invalid length");
    digitalWrite(SYNC_PIN, LOW);
    return false;
  }
  
  // Read message bytes
  for (int i = 0; i < length; i++) {
    byte = 0;
    for (int j = 7; j >= 0; j--) {
      digitalWrite(CLOCK_PIN, HIGH);
      delayMicroseconds(SETUP_DELAY_US);
      digitalWrite(CLOCK_PIN, LOW);
      delayMicroseconds(BIT_PERIOD_MS * 500);
      int bit = digitalRead(DATA_PIN);
      byte |= (bit << j);
      digitalWrite(CLOCK_PIN, HIGH);
      delayMicroseconds(BIT_PERIOD_MS * 500);
    }
    rx_buffer[i] = (char)byte;
  }
  rx_buffer[length] = '\0';
  
  Serial.print("Response: \"");
  Serial.print(rx_buffer);
  Serial.println("\"");
  
  digitalWrite(SYNC_PIN, LOW);
  digitalWrite(LED_BUILTIN, LOW);
  return true;
}

// Complete bidirectional exchange
void performExchange() {
  // First send message to DE1-SoC
  sendMessage();
  
  // Then look for response
  receiveResponse();
}

// Main loop
void loop() {
  static unsigned long lastTime = 0;
  unsigned long currentTime = millis();
  
  // Perform exchange every 5 seconds
  if (currentTime - lastTime >= 5000) {
    Serial.println("\n----- NEW BIDIRECTIONAL EXCHANGE -----");
    performExchange();
    lastTime = currentTime;
  }
  
  delay(10);
}