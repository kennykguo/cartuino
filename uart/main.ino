/*
 * Arduino Asynchronous Communication Protocol - Simplified Version
 * For communication with DE1-SoC FPGA
 * 
 * Uses software UART on a single data line (D0)
 */

// Pin definitions
#define DATA_PIN 0  // D0 for data line

// Communication parameters
#define BIT_PERIOD_MS 5     // 5ms per bit (200 bps) - must match DE1-SoC
#define MSG_BUFFER_SIZE 32  // Buffer size for messages
char tx_buffer[MSG_BUFFER_SIZE];
char rx_buffer[MSG_BUFFER_SIZE];

// Message counter
int message_counter = 0;

// Protocol constants
#define START_SEQUENCE 0xAA // 10101010 pattern for synchronization
#define END_SEQUENCE 0x55   // 01010101 pattern for end of message

// Timestamp for communication
unsigned long lastActivityTime = 0;

void setup() {
  // Setup USB serial for debugging output
  Serial.begin(9600);
  
  // Configure DATA pin - start as output to set a known state
  pinMode(DATA_PIN, OUTPUT);
  digitalWrite(DATA_PIN, HIGH);  // Idle state is HIGH
  delay(100);  // Hold for stability
  
  // Now set to input for receiving
  pinMode(DATA_PIN, INPUT);
  
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
  
  Serial.println("\n\n");
  Serial.println("==============================================");
  Serial.println("Arduino Simple Communication Protocol Ready");
  Serial.println("==============================================");
  Serial.println("Listening for messages from DE1-SoC...");
  Serial.println("==============================================");
  
  lastActivityTime = millis();
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

// Send a byte using software UART approach
void sendByte(unsigned char byte) {
  Serial.print("Sending byte: 0x");
  if (byte < 16) Serial.print("0");
  Serial.println(byte, HEX);
  
  // Set DATA pin as output
  setDataPinMode(OUTPUT);
  
  // Send start bit (always 0)
  setDataPin(0);
  delay(BIT_PERIOD_MS);
  
  // Send 8 data bits, LSB first
  for (int i = 0; i < 8; i++) {
    int bit = (byte >> i) & 0x01;
    setDataPin(bit);
    delay(BIT_PERIOD_MS);
  }
  
  // Send stop bit (always 1)
  setDataPin(1);
  delay(BIT_PERIOD_MS);
  
  // Keep line in idle state
  setDataPin(1);
}

// Receive a byte using software UART approach
// Returns -1 on error
int receiveByte(int timeout_ms) {
  unsigned char byte = 0;
  int start_bit_found = 0;
  unsigned long start_time = millis();
  
  // Set DATA pin as input
  setDataPinMode(INPUT);
  
  // Wait for start bit (logic 0)
  while (!start_bit_found) {
    if (readDataPin() == 0) {
      // Potential start bit detected, wait for half a bit period to verify
      delay(BIT_PERIOD_MS / 2);
      
      // Check if it's still low (valid start bit)
      if (readDataPin() == 0) {
        start_bit_found = 1;
        Serial.println("Start bit detected");
      }
    }
    
    // Check for timeout
    if (millis() - start_time > timeout_ms) {
      Serial.println("Timeout waiting for start bit");
      return -1;
    }
    delay(1);  // Small delay between checks
  }
  
  // Start bit found, wait for the remaining half of start bit
  delay(BIT_PERIOD_MS / 2);
  
  // Read 8 data bits
  for (int i = 0; i < 8; i++) {
    // Sample in the middle of each bit
    delay(BIT_PERIOD_MS);
    if (readDataPin()) {
      byte |= (1 << i);  // LSB first
    }
  }
  
  // Wait for and verify stop bit
  delay(BIT_PERIOD_MS);
  if (readDataPin() != 1) {
    Serial.println("Invalid stop bit");
    return -1;
  }
  
  Serial.print("Received byte: 0x");
  if (byte < 16) Serial.print("0");
  Serial.println(byte, HEX);
  
  return byte;
}

// Send a message with simple framing
void sendMessage(char *message) {
  int len = strlen(message);
  
  // Turn on LED during transmission
  digitalWrite(LED_BUILTIN, HIGH);
  
  Serial.print("Sending message: \"");
  Serial.print(message);
  Serial.print("\" (");
  Serial.print(len);
  Serial.println(" bytes)");
  
  // Send start sequence byte
  sendByte(START_SEQUENCE);
  
  // Send length byte
  sendByte(len);
  
  // Send message bytes
  for (int i = 0; i < len; i++) {
    sendByte(message[i]);
  }
  
  // Send end sequence byte
  sendByte(END_SEQUENCE);
  
  // Set pin back to input to listen for next message
  setDataPinMode(INPUT);
  
  // Turn off LED
  digitalWrite(LED_BUILTIN, LOW);
  
  Serial.println("Message sent");
}

// Receive a message with simple framing
int receiveMessage(int timeout_ms) {
  int byte_val;
  int len;
  
  // Turn on LED during reception
  digitalWrite(LED_BUILTIN, HIGH);
  
  Serial.println("Receiving message...");
  
  // Wait for and verify start sequence
  byte_val = receiveByte(timeout_ms);
  if (byte_val != START_SEQUENCE) {
    if (byte_val != -1) {  // Only print error if not a timeout
      Serial.print("Invalid start sequence: 0x");
      if (byte_val < 16) Serial.print("0");
      Serial.print(byte_val, HEX);
      Serial.print(", expected 0x");
      if (START_SEQUENCE < 16) Serial.print("0");
      Serial.println(START_SEQUENCE, HEX);
    }
    digitalWrite(LED_BUILTIN, LOW);
    return 0;
  }
  
  // Receive length byte
  byte_val = receiveByte(timeout_ms);
  if (byte_val < 0) {
    Serial.println("Error receiving length byte");
    digitalWrite(LED_BUILTIN, LOW);
    return 0;
  }
  len = byte_val;
  
  Serial.print("Expecting message of ");
  Serial.print(len);
  Serial.println(" bytes");
  
  // Ensure length is reasonable
  if (len >= MSG_BUFFER_SIZE) {
    Serial.print("Message too long: ");
    Serial.print(len);
    Serial.println(" bytes");
    digitalWrite(LED_BUILTIN, LOW);
    return 0;
  }
  
  // Receive each byte of the message
  for (int i = 0; i < len; i++) {
    byte_val = receiveByte(timeout_ms);
    if (byte_val < 0) {
      Serial.print("Error receiving data byte ");
      Serial.println(i);
      digitalWrite(LED_BUILTIN, LOW);
      return 0;
    }
    rx_buffer[i] = byte_val;
  }
  
  // Null-terminate the string
  rx_buffer[len] = '\0';
  
  // Receive end sequence
  byte_val = receiveByte(timeout_ms);
  if (byte_val != END_SEQUENCE) {
    Serial.print("Invalid end sequence: 0x");
    if (byte_val < 16) Serial.print("0");
    Serial.print(byte_val, HEX);
    Serial.print(", expected 0x");
    if (END_SEQUENCE < 16) Serial.print("0");
    Serial.println(END_SEQUENCE, HEX);
    digitalWrite(LED_BUILTIN, LOW);
    return 0;
  }
  
  Serial.print("Message received successfully: \"");
  Serial.print(rx_buffer);
  Serial.println("\"");
  
  digitalWrite(LED_BUILTIN, LOW);
  return 1;
}

// Check if DATA pin is low (potential start bit)
bool detectStartBit() {
  setDataPinMode(INPUT);
  return (readDataPin() == 0);
}

void loop() {
  // Check for incoming message from DE1-SoC
  if (detectStartBit()) {
    Serial.println("Potential start bit detected!");
    
    if (receiveMessage(200)) {  // 200ms timeout for each byte
      Serial.print("Received message from DE1-SoC: ");
      Serial.println(rx_buffer);
      
      // Auto-respond with an acknowledgment message
      delay(100);  // Small delay before responding
      sprintf(tx_buffer, "ARD_ACK_%d", message_counter++);
      sendMessage(tx_buffer);
    }
    
    lastActivityTime = millis();
  }
  
  // If it's been 5 seconds since last activity, print diagnostic info
  if (millis() - lastActivityTime > 5000) {
    // Read and print current line state
    setDataPinMode(INPUT);
    int lineState = readDataPin();
    Serial.print("Waiting for activity... Line state: ");
    Serial.println(lineState);
    
    lastActivityTime = millis();
  }
  
  // Small delay to prevent CPU hogging
  delay(5);
}