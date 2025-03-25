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
#define BIT_PERIOD_MS 10    // Bit transfer rate (10ms per bit)
#define MSG_BUFFER_SIZE 32  // Buffer size for messages
char tx_buffer[MSG_BUFFER_SIZE];
char rx_buffer[MSG_BUFFER_SIZE];
int message_counter = 0;

// Protocol constants
#define START_BYTE 0xAA  // 10101010 pattern for sync
#define RESPONSE_WAIT_MS 500 // Time to wait for response after sending

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
  // First set clock HIGH (inactive)
  digitalWrite(CLOCK_PIN, HIGH);
  delay(5);
  
  // Set data bit value
  digitalWrite(DATA_PIN, bit ? HIGH : LOW);
  delay(5); // Setup time
  
  // Toggle clock LOW (active, FPGA reads on this edge)
  digitalWrite(CLOCK_PIN, LOW);
  delay(BIT_PERIOD_MS); // Hold time
  
  // Return clock HIGH (inactive)
  digitalWrite(CLOCK_PIN, HIGH);
  delay(5); // Recovery time
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
  
  // Turn on LED during transmission
  digitalWrite(LED_BUILTIN, HIGH);
  
  Serial.print("Sending: \"");
  Serial.print(tx_buffer);
  Serial.println("\"");
  
  // Step 1: Begin with sync handshake
  Serial.println("Starting sync handshake");
  digitalWrite(SYNC_PIN, HIGH);
  delay(100); // Long delay to ensure FPGA notices
  
  // Step 2: Make sure clock is in known state
  digitalWrite(CLOCK_PIN, HIGH);
  delay(50);
  
  // Step 3: Send start byte (synchronization pattern)
  sendByte(START_BYTE);
  
  // Step 4: Send length byte
  sendByte((unsigned char)len);
  
  // Step 5: Send message bytes
  for (int i = 0; i < len; i++) {
    sendByte((unsigned char)tx_buffer[i]);
  }
  
  // Step 6: End transmission
  Serial.println("Message sent");
  
  // Keep SYNC high to indicate we're ready for response
  delay(50);
  
  // Turn off LED for sending
  digitalWrite(LED_BUILTIN, LOW);
}

// Check for and receive response from DE1-SoC
bool receiveResponse() {
  unsigned char byte, length;
  int i;
  
  Serial.println("Waiting for response...");
  
  // Set data pin as input to check if FPGA wants to respond
  pinMode(DATA_PIN, INPUT);
  
  // Wait for FPGA to pull DATA high to indicate response ready
  unsigned long startTime = millis();
  while (digitalRead(DATA_PIN) == LOW) {
    // Timeout after specified wait time
    if (millis() - startTime > RESPONSE_WAIT_MS) {
      Serial.println("No response detected");
      digitalWrite(SYNC_PIN, LOW); // Release SYNC line
      return false;
    }
    delay(1);
  }
  
  Serial.println("Response detected");
  
  // Turn on LED for receiving
  digitalWrite(LED_BUILTIN, HIGH);
  
  // Acknowledge by toggling clock
  digitalWrite(CLOCK_PIN, LOW);
  delay(20);
  digitalWrite(CLOCK_PIN, HIGH);
  delay(20);
  
  // Receive start byte
  byte = receiveByte();
  if (byte != START_BYTE) {
    Serial.println("Invalid start byte in response");
    digitalWrite(SYNC_PIN, LOW); // Release SYNC line
    digitalWrite(LED_BUILTIN, LOW);
    return false;
  }
  
  // Receive length byte
  length = receiveByte();
  if (length >= MSG_BUFFER_SIZE || length == 0) {
    Serial.println("Invalid response length");
    digitalWrite(SYNC_PIN, LOW);
    digitalWrite(LED_BUILTIN, LOW);
    return false;
  }
  
  // Receive message bytes
  for (i = 0; i < length; i++) {
    byte = receiveByte();
    rx_buffer[i] = (char)byte;
  }
  rx_buffer[i] = '\0'; // Null-terminate
  
  Serial.print("Response received: \"");
  Serial.print(rx_buffer);
  Serial.println("\"");
  
  // End bidirectional session
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