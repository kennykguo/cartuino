/*
 * Arduino Fast Bidirectional Communication Protocol (Master)
 * For communication with DE1-SoC FPGA
 * Uses 3-wire handshake protocol:
 * - D2 for data line - connects to DE1-SoC D0
 * - D3 for clock line - connects to DE1-SoC D1
 * - D4 for sync line - connects to DE1-SoC D2
 */

// Pin definitions
#define CLOCK_PIN 3  // D3 for clock line
#define DATA_PIN 2   // D2 for data line
#define SYNC_PIN 4   // D4 for sync/handshake line

// Communication parameters
#define BIT_PERIOD_MS 5     // Fast bit transfer (5ms per bit)
#define MSG_BUFFER_SIZE 32  // Buffer size for messages
char tx_buffer[MSG_BUFFER_SIZE];
char rx_buffer[MSG_BUFFER_SIZE];

// Message counter and timing
int message_counter = 0;
unsigned long lastMessageTime = 0;
unsigned long messageSendInterval = 3000;  // Send message every 3 seconds

// Protocol constants
#define START_BYTE 0xAA  // 10101010 pattern for synchronization
#define END_BYTE 0x55    // 01010101 pattern for end of message
#define CMD_ACK 0xCC     // Acknowledge receipt
#define CMD_NACK 0x33    // Negative acknowledge
#define CMD_RESPONSE 0xF0 // Response follows

// State variables
bool awaitingResponse = false;

void setup() {
  // Setup USB serial for debugging output
  Serial.begin(115200);  // Faster serial for debugging
  
  // Configure pins
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, INPUT);  // Start as input
  pinMode(SYNC_PIN, OUTPUT);
  
  // Initialize pins to known states
  digitalWrite(CLOCK_PIN, HIGH);  // Idle state is HIGH
  digitalWrite(SYNC_PIN, LOW);    // Idle state is LOW
  
  // Configure built-in LED
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Initial LED flash to show we're ready
  for (int i = 0; i < 2; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
    delay(50);
  }
  
  Serial.println("Arduino Fast Bidirectional Protocol Ready");
  Serial.println("Using D2(data), D3(clock), D4(sync) to DE1-SoC");
}

// Send a single bit with fast timing
void sendBit(int bit) {
  // Set data line to the bit value
  pinMode(DATA_PIN, OUTPUT);
  digitalWrite(DATA_PIN, bit ? HIGH : LOW);
  delayMicroseconds(200);  // Short setup time
  
  // Clock LOW to indicate bit is ready
  digitalWrite(CLOCK_PIN, LOW);
  delayMicroseconds(BIT_PERIOD_MS * 500);  // Half period in us
  
  // Clock HIGH to complete bit transmission
  digitalWrite(CLOCK_PIN, HIGH);
  delayMicroseconds(BIT_PERIOD_MS * 500);  // Half period in us
}

// Receive a single bit
int receiveBit() {
  int bit;
  
  // Set data pin as input
  pinMode(DATA_PIN, INPUT);
  delayMicroseconds(200);  // Short setup time
  
  // Clock LOW to signal ready to read
  digitalWrite(CLOCK_PIN, LOW);
  delayMicroseconds(BIT_PERIOD_MS * 500);  // Half period in us
  
  // Read the bit at the middle of the clock pulse
  bit = digitalRead(DATA_PIN);
  
  // Clock HIGH to complete the bit read
  digitalWrite(CLOCK_PIN, HIGH);
  delayMicroseconds(BIT_PERIOD_MS * 500);  // Half period in us
  
  return bit;
}

// Send a byte (8 bits) MSB first
void sendByte(unsigned char byte) {
  // Debug output
  Serial.print("TX: 0x");
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
  
  // Debug output
  Serial.print("RX: 0x");
  if (byte < 16) Serial.print("0");
  Serial.println(byte, HEX);
  
  return byte;
}

// Send a message to the DE1-SoC
bool sendMessage(const char *message) {
  int len = strlen(message);
  if (len > MSG_BUFFER_SIZE - 1) {
    len = MSG_BUFFER_SIZE - 1;
  }
  
  // Turn on LED during transmission
  digitalWrite(LED_BUILTIN, HIGH);
  
  Serial.print("Sending: \"");
  Serial.print(message);
  Serial.println("\"");
  
  // First, assert the SYNC line to get DE1-SoC's attention
  digitalWrite(SYNC_PIN, HIGH);
  delay(50); // Longer delay to ensure DE1-SoC is ready to receive
  
  // Reset the clock line to ensure synchronization
  digitalWrite(CLOCK_PIN, HIGH);
  delay(20);
  
  // Send start byte
  sendByte(START_BYTE);
  
  // Send length byte
  sendByte((unsigned char)len);
  
  // Send each byte of the message
  for (int i = 0; i < len; i++) {
    sendByte((unsigned char)message[i]);
  }
  
  // Send end byte
  sendByte(END_BYTE);
  
  // Keep SYNC high for a moment to ensure message completion
  delay(20);
  
  // Set SYNC line back to low
  digitalWrite(SYNC_PIN, LOW);
  
  // Give the DE1-SoC time to process before expecting a response
  delay(50);
  
  // Wait for acknowledgment or response command
  unsigned char response = receiveByte();
  
  // Turn off LED
  digitalWrite(LED_BUILTIN, LOW);
  
  if (response == CMD_ACK) {
    Serial.println("Received ACK");
    return true;
  } 
  else if (response == CMD_RESPONSE) {
    Serial.println("Response follows");
    awaitingResponse = true;
    return true;
  }
  else {
    Serial.println("Invalid response");
    return false;
  }
}

// Receive a response message
bool receiveResponse() {
  unsigned char byte, length;
  
  // Turn on LED during reception
  digitalWrite(LED_BUILTIN, HIGH);
  
  Serial.println("Receiving response...");
  
  // Look for start byte
  byte = receiveByte();
  if (byte != START_BYTE) {
    Serial.println("Invalid start byte");
    digitalWrite(LED_BUILTIN, LOW);
    return false;
  }
  
  // Get length
  length = receiveByte();
  if (length >= MSG_BUFFER_SIZE) {
    Serial.println("Message too long");
    digitalWrite(LED_BUILTIN, LOW);
    return false;
  }
  
  // Receive message bytes
  for (int i = 0; i < length; i++) {
    rx_buffer[i] = receiveByte();
  }
  rx_buffer[length] = '\0';
  
  // Receive end byte
  byte = receiveByte();
  
  // Turn off LED
  digitalWrite(LED_BUILTIN, LOW);
  
  if (byte != END_BYTE) {
    Serial.println("Invalid end byte");
    return false;
  }
  
  Serial.print("Response received: \"");
  Serial.print(rx_buffer);
  Serial.println("\"");
  
  // Send ACK
  sendByte(CMD_ACK);
  
  return true;
}

// Main loop
void loop() {
  unsigned long currentTime = millis();
  
  // If it's time to send a new message
  if (currentTime - lastMessageTime >= messageSendInterval) {
    // Create test message
    sprintf(tx_buffer, "ARD_MSG_%d", message_counter++);
    
    // Send the message
    if (sendMessage(tx_buffer)) {
      // If response is expected, receive it
      if (awaitingResponse) {
        if (receiveResponse()) {
          Serial.println("Bidirectional exchange completed");
        } else {
          Serial.println("Failed to receive response");
        }
        awaitingResponse = false;
      }
    }
    
    // Update timing
    lastMessageTime = currentTime;
  }
  
  // Small delay - keep it minimal
  delay(1);
}