/*
 * Arduino Bidirectional Communication Protocol (Master)
 * For communication with DE1-SoC FPGA
 * Uses D2 for data and D3 for clock, connecting to DE1-SoC D0 and D1
 */

// Pin definitions
#define CLOCK_PIN 3  // D3 for clock line - connects to DE1-SoC D1
#define DATA_PIN 2   // D2 for data line - connects to DE1-SoC D0

// Communication parameters
#define BIT_PERIOD_MS 20    // 20ms per bit
#define MSG_BUFFER_SIZE 32  // Buffer size for messages
char tx_buffer[MSG_BUFFER_SIZE];
char rx_buffer[MSG_BUFFER_SIZE];

// Message counter and state tracking
int message_counter = 0;
unsigned long lastMessageTime = 0;
unsigned long messageSendInterval = 5000;  // Send message every 5 seconds

// Protocol constants
#define START_BYTE 0xAA  // 10101010 pattern for synchronization
#define END_BYTE 0x55    // 01010101 pattern for end of message
#define ACK_BYTE 0xCC    // 11001100 pattern for acknowledgment

void setup() {
  // Setup USB serial for debugging output
  Serial.begin(9600);
  
  // Configure pins with pulldowns to prevent floating
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, INPUT);  // Start as input to avoid conflicts
  
  // Initialize pins to known states
  digitalWrite(CLOCK_PIN, HIGH);
  
  // Configure built-in LED
  pinMode(LED_BUILTIN, OUTPUT);
  
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
  Serial.println("Arduino Master Communication Protocol Ready");
  Serial.println("==============================================");
  Serial.println("Using D2 (data) and D3 (clock) to DE1-SoC D0/D1");
  Serial.println("Automatic bidirectional communication");
  Serial.println("Will initiate messages every 5 seconds");
  Serial.println("==============================================");
}

// Send a single bit (with explicit clock control)
void sendBit(int bit) {
  // Set data line to desired value
  pinMode(DATA_PIN, OUTPUT);
  digitalWrite(DATA_PIN, bit ? HIGH : LOW);
  delay(5);  // Setup time
  
  // Clock LOW to indicate bit is ready
  digitalWrite(CLOCK_PIN, LOW);
  delay(BIT_PERIOD_MS / 2);
  
  // Clock HIGH to complete bit transmission
  digitalWrite(CLOCK_PIN, HIGH);
  delay(BIT_PERIOD_MS / 2);
}

// Receive a single bit (with explicit clock control)
int receiveBit() {
  int bit;
  
  // Set data pin as input to allow DE1-SoC to drive it
  pinMode(DATA_PIN, INPUT);
  delay(5);  // Mode change settling time
  
  // Clock LOW to trigger DE1-SoC to set data bit
  digitalWrite(CLOCK_PIN, LOW);
  delay(BIT_PERIOD_MS / 2);
  
  // Read the data bit at the middle of the clock pulse
  bit = digitalRead(DATA_PIN);
  
  // Clock HIGH to complete the bit read
  digitalWrite(CLOCK_PIN, HIGH);
  delay(BIT_PERIOD_MS / 2);
  
  return bit;
}

// Send a byte (8 bits) MSB first
void sendByte(unsigned char byte) {
  Serial.print("Sending byte: 0x");
  if (byte < 16) Serial.print("0");
  Serial.println(byte, HEX);
  
  // Send each bit, MSB first
  for (int i = 7; i >= 0; i--) {
    int bit = (byte >> i) & 0x01;
    sendBit(bit);
  }
}

// Receive a byte (8 bits) MSB first
unsigned char receiveByte() {
  unsigned char byte = 0;
  
  // Receive 8 bits, MSB first
  for (int i = 7; i >= 0; i--) {
    int bit = receiveBit();
    byte |= (bit << i);
  }
  
  Serial.print("Received byte: 0x");
  if (byte < 16) Serial.print("0");
  Serial.println(byte, HEX);
  
  return byte;
}

// Initialize communication
void initCommunication() {
  Serial.println("Initializing communication sequence...");
  
  // Clear the bus
  pinMode(DATA_PIN, INPUT);  // First release the data line
  pinMode(CLOCK_PIN, OUTPUT);
  digitalWrite(CLOCK_PIN, HIGH);
  delay(200);  // Long setup delay
  
  // Send synchronization pattern (16 clock pulses)
  for (int i = 0; i < 16; i++) {
    digitalWrite(CLOCK_PIN, LOW);
    delay(30);
    digitalWrite(CLOCK_PIN, HIGH);
    delay(30);
  }
  
  // Reset to idle state
  digitalWrite(CLOCK_PIN, HIGH);
  delay(200);  // Another long delay
  
  Serial.println("Initialization complete");
}

// Reset the bus to a known state
void resetBus() {
  // Release the data line first
  pinMode(DATA_PIN, INPUT);
  delay(10);
  
  // Set clock high (idle state)
  digitalWrite(CLOCK_PIN, HIGH);
  delay(100);
  
  // Send a few reset pulses
  for (int i = 0; i < 4; i++) {
    digitalWrite(CLOCK_PIN, LOW);
    delay(20);
    digitalWrite(CLOCK_PIN, HIGH);
    delay(20);
  }
}

// Send a message to the DE1-SoC
bool sendMessage() {
  // Create test message
  sprintf(tx_buffer, "ARD_MSG_%d", message_counter++);
  int len = strlen(tx_buffer);
  unsigned char checksum = 0;
  
  // Turn on LED during transmission
  digitalWrite(LED_BUILTIN, HIGH);
  
  Serial.print("Sending message: \"");
  Serial.print(tx_buffer);
  Serial.print("\" (");
  Serial.print(len);
  Serial.println(" bytes)");
  
  // Send start byte
  sendByte(START_BYTE);
  
  // Send length byte
  sendByte((unsigned char)len);
  checksum ^= len;
  
  // Send each byte of the message
  for (int i = 0; i < len; i++) {
    sendByte((unsigned char)tx_buffer[i]);
    checksum ^= tx_buffer[i];
  }
  
  // Send checksum
  sendByte(checksum);
  
  // Send end byte
  sendByte(END_BYTE);
  
  Serial.println("Message sent, waiting for response...");
  
  // Turn off LED
  digitalWrite(LED_BUILTIN, LOW);
  
  // Give DE1-SoC time to process and prepare response
  delay(300);
  
  // Reset bus before receiving
  resetBus();
  
  return receiveResponse();
}

// Receive a response from the DE1-SoC
bool receiveResponse() {
  unsigned char byte, length, checksum = 0, calculated_checksum = 0;
  bool success = false;
  
  // Turn on LED during reception
  digitalWrite(LED_BUILTIN, HIGH);
  
  Serial.println("Receiving response from DE1-SoC...");
  
  // Wait for start byte with retry
  int max_retries = 5;
  bool start_found = false;
  
  for (int retry = 0; retry < max_retries && !start_found; retry++) {
    byte = receiveByte();
    if (byte == START_BYTE) {
      start_found = true;
      Serial.println("Start byte detected");
    } else {
      Serial.print("Retry ");
      Serial.print(retry + 1);
      Serial.print(": Invalid start byte: 0x");
      if (byte < 16) Serial.print("0");
      Serial.println(byte, HEX);
      
      // Send reset clock pulses between retries
      resetBus();
      delay(50);
    }
  }
  
  if (!start_found) {
    Serial.println("Failed to receive valid start byte");
    digitalWrite(LED_BUILTIN, LOW);
    return false;
  }
  
  // Receive length byte
  length = receiveByte();
  calculated_checksum ^= length;
  
  Serial.print("Expecting message of ");
  Serial.print(length);
  Serial.println(" bytes");
  
  // Ensure length is reasonable
  if (length >= MSG_BUFFER_SIZE || length == 0) {
    Serial.print("Invalid message length: ");
    Serial.print(length);
    Serial.println(" bytes");
    digitalWrite(LED_BUILTIN, LOW);
    return false;
  }
  
  // Receive message bytes
  int pos = 0;
  for (int i = 0; i < length; i++) {
    byte = receiveByte();
    rx_buffer[pos++] = byte;
    calculated_checksum ^= byte;
  }
  
  // Null-terminate the string
  rx_buffer[pos] = '\0';
  
  // Receive checksum
  checksum = receiveByte();
  
  // Receive end byte
  byte = receiveByte();
  
  // Verify checksum and end byte
  if (byte != END_BYTE) {
    Serial.print("Invalid end byte: 0x");
    if (byte < 16) Serial.print("0");
    Serial.println(byte, HEX);
    success = false;
  } else if (checksum != calculated_checksum) {
    Serial.print("Checksum error: received 0x");
    if (checksum < 16) Serial.print("0");
    Serial.print(checksum, HEX);
    Serial.print(", calculated 0x");
    if (calculated_checksum < 16) Serial.print("0");
    Serial.println(calculated_checksum, HEX);
    success = false;
  } else {
    Serial.print("Message received successfully: \"");
    Serial.print(rx_buffer);
    Serial.println("\"");
    success = true;
  }
  
  // Send ACK regardless of success (to complete the protocol)
  pinMode(DATA_PIN, OUTPUT);
  delay(10);
  sendByte(ACK_BYTE);
  
  // Turn off LED
  digitalWrite(LED_BUILTIN, LOW);
  
  return success;
}

// Perform a complete message exchange cycle
void performMessageExchange() {
  // Initialize communication with clock pulses
  initCommunication();
  
  // Send message and receive response
  bool success = sendMessage();
  
  if (success) {
    Serial.println("Complete message exchange successful!");
    // Flash LED to indicate success
    for (int i = 0; i < 3; i++) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(50);
      digitalWrite(LED_BUILTIN, LOW);
      delay(50);
    }
  } else {
    Serial.println("Message exchange failed");
    // Longer LED flash to indicate failure
    digitalWrite(LED_BUILTIN, HIGH);
    delay(300);
    digitalWrite(LED_BUILTIN, LOW);
  }
}

// Main loop
void loop() {
  unsigned long currentTime = millis();
  
  // If it's time to send a new message
  if (currentTime - lastMessageTime >= messageSendInterval) {
    Serial.println("\n\nStarting new message exchange cycle");
    
    performMessageExchange();
    
    // Update the last message time
    lastMessageTime = currentTime;
  }
  
  // Small delay
  delay(10);
}