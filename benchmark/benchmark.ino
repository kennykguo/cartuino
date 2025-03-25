/*
 * Arduino Bidirectional Communication Protocol (Master)
 * For communication with DE1-SoC FPGA
 * Uses D2 for data and D3 for clock, connecting to DE1-SoC D0 and D1
 */

// Pin definitions
#define CLOCK_PIN 3  // D3 for clock line - connects to DE1-SoC D1
#define DATA_PIN 2   // D2 for data line - connects to DE1-SoC D0

// Communication parameters
#define BIT_PERIOD_MS 20    // 20ms per bit - faster than previous version
#define MSG_BUFFER_SIZE 32  // Buffer size for messages
char tx_buffer[MSG_BUFFER_SIZE];
char rx_buffer[MSG_BUFFER_SIZE];

// Message counter and state tracking
int message_counter = 0;
bool message_exchange_in_progress = false;
bool waiting_for_response = false;

// Protocol constants
#define START_BYTE 0xAA  // 10101010 pattern for synchronization
#define END_BYTE 0x55    // 01010101 pattern for end of message
#define ACK_BYTE 0xCC    // 11001100 pattern for acknowledgment

// Timer for automatic message sending
unsigned long lastMessageTime = 0;
unsigned long messageSendInterval = 5000;  // Send message every 5 seconds

void setup() {
  // Setup USB serial for debugging output
  Serial.begin(9600);
  
  // Configure pins
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);  // Start as output
  
  // Initialize pins to known states
  digitalWrite(CLOCK_PIN, HIGH);
  digitalWrite(DATA_PIN, LOW);
  
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

// Send a single bit (simple approach)
void sendBit(int bit) {
  // First set data line
  digitalWrite(DATA_PIN, bit ? HIGH : LOW);
  delay(5);  // Give time for line to stabilize
  
  // Clock low (signal bit is ready)
  digitalWrite(CLOCK_PIN, LOW);
  delay(BIT_PERIOD_MS / 2);
  
  // Clock high (complete bit)
  digitalWrite(CLOCK_PIN, HIGH);
  delay(BIT_PERIOD_MS / 2);
}

// Receive a single bit
int receiveBit() {
  int bit;
  
  // Set data pin as input
  pinMode(DATA_PIN, INPUT);
  delay(2);  // Give time for mode change to take effect
  
  // Clock low (signal ready to read)
  digitalWrite(CLOCK_PIN, LOW);
  delay(BIT_PERIOD_MS / 2);
  
  // Read data bit
  bit = digitalRead(DATA_PIN);
  
  // Clock high (complete bit read)
  digitalWrite(CLOCK_PIN, HIGH);
  delay(BIT_PERIOD_MS / 2);
  
  return bit;
}

// Send a byte (8 bits) MSB first
void sendByte(unsigned char byte) {
  Serial.print("Sending byte: 0x");
  if (byte < 16) Serial.print("0");
  Serial.println(byte, HEX);
  
  // Set as output before sending
  pinMode(DATA_PIN, OUTPUT);
  delay(5);
  
  // Send each bit, MSB first
  for (int i = 7; i >= 0; i--) {
    int bit = (byte >> i) & 0x01;
    sendBit(bit);
  }
  
  // Small pause between bytes
  delay(10);
}

// Receive a byte (8 bits) MSB first
unsigned char receiveByte() {
  unsigned char byte = 0;
  
  // Set as input before receiving
  pinMode(DATA_PIN, INPUT);
  delay(5);
  
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
  
  // Signal initialization with clear clock/data pattern
  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  
  // Reset to known state
  digitalWrite(CLOCK_PIN, HIGH);
  digitalWrite(DATA_PIN, LOW);
  delay(100);
  
  // Send initialization pattern (10 clock pulses with data low)
  for (int i = 0; i < 10; i++) {
    digitalWrite(CLOCK_PIN, LOW);
    delay(25);
    digitalWrite(CLOCK_PIN, HIGH);
    delay(25);
  }
  
  // Another reset
  digitalWrite(CLOCK_PIN, HIGH);
  digitalWrite(DATA_PIN, LOW);
  delay(100);
  
  Serial.println("Initialization complete");
}

// Send a message
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
  
  // Send start byte
  sendByte(START_BYTE);
  
  // Send length byte
  sendByte((unsigned char)len);
  checksum ^= len;
  
  // Send each byte of the message
  for (int i = 0; i < len; i++) {
    sendByte((unsigned char)message[i]);
    checksum ^= message[i];
  }
  
  // Send checksum
  sendByte(checksum);
  
  // Send end byte
  sendByte(END_BYTE);
  
  // Turn off LED
  digitalWrite(LED_BUILTIN, LOW);
  
  Serial.println("Message sent, waiting for response...");
  
  // Set flag to wait for response
  waiting_for_response = true;
}

// Receive a message
bool receiveMessage() {
  unsigned char byte, length, checksum = 0, calculated_checksum = 0;
  bool success = false;
  
  // Turn on LED during reception
  digitalWrite(LED_BUILTIN, HIGH);
  
  Serial.println("Receiving response from DE1-SoC...");
  
  // Wait for start byte with retry
  int max_retries = 3;
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
  if (length >= MSG_BUFFER_SIZE) {
    Serial.print("Message too long: ");
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
  
  // Send ACK
  sendByte(ACK_BYTE);
  
  // Turn off LED
  digitalWrite(LED_BUILTIN, LOW);
  
  return success;
}

// Main loop
void loop() {
  unsigned long currentTime = millis();
  
  // If we're waiting for a response, check for it
  if (waiting_for_response) {
    // Attempt to receive the response
    bool received = receiveMessage();
    waiting_for_response = false;
    message_exchange_in_progress = false;
    
    if (received) {
      Serial.println("Message exchange completed successfully");
    } else {
      Serial.println("Failed to receive response");
    }
    
    // Update the last message time to reset the interval
    lastMessageTime = currentTime;
  }
  // If no message exchange is in progress and it's time to send a new message
  else if (!message_exchange_in_progress && (currentTime - lastMessageTime >= messageSendInterval)) {
    Serial.println("\nTime to send a test message");
    
    // Initialize communication to get clean state
    initCommunication();
    
    // Create and send test message
    sprintf(tx_buffer, "ARD_MSG_%d", message_counter++);
    
    // Mark that we're starting a message exchange
    message_exchange_in_progress = true;
    
    // Send the message
    sendMessage(tx_buffer);
  }
  
  // Small delay
  delay(5);
}