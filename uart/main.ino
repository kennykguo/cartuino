/*
 * Arduino Asynchronous Communication Protocol
 * For communication with DE1-SoC FPGA
 * 
 * Uses software UART-like protocol with single data line
 */

// Pin definitions
#define DATA_PIN 0  // D0 for data line

// Communication parameters
#define BIT_PERIOD_MS 2     // 2ms per bit (500 bps) - must match DE1-SoC
#define MSG_BUFFER_SIZE 64  // Buffer size for messages
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

// Timeout constants
#define MAX_WAIT_TIME_MS 1000

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
  
  // Configure DATA pin
  pinMode(DATA_PIN, INPUT); // Default to input mode
  
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
  Serial.println("Arduino Asynchronous Communication Protocol Ready");
  Serial.println("==============================================");
  Serial.println("Controls:");
  Serial.println("- Button on pin 2: Send test message to DE1-SoC");
  Serial.println("- Will automatically receive messages from DE1-SoC");
  Serial.println("==============================================");
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

// Send a single bit - hold for BIT_PERIOD_MS
void sendBit(int bit) {
  // Set the data value and hold for the full bit period
  setDataPin(bit);
  delay(BIT_PERIOD_MS);
}

// Send a byte (8 bits) LSB first, with start and stop bits
void sendByte(unsigned char byte) {
  // Set DATA pin as output
  setDataPinMode(OUTPUT);
  
  Serial.print("Sending byte: 0x");
  if (byte < 16) Serial.print("0");
  Serial.println(byte, HEX);
  
  // Send start bit (always 0)
  sendBit(0);
  
  // Send 8 data bits, LSB first
  for (int i = 0; i < 8; i++) {
    int bit = (byte >> i) & 0x01;
    sendBit(bit);
  }
  
  // Send stop bit (always 1)
  sendBit(1);
  
  // Add extra idle time between bytes for stability
  delay(BIT_PERIOD_MS);
}

// Receive a byte (8 bits) LSB first, with start and stop bits
// Returns 0xFF on error
unsigned char receiveByte() {
  unsigned char byte = 0;
  int bit;
  unsigned long startTime;
  
  // Set DATA pin as input
  setDataPinMode(INPUT);
  
  // Wait for start bit (logic 0)
  startTime = millis();
  while (readDataPin() == 1) {
    if (millis() - startTime > MAX_WAIT_TIME_MS) {
      Serial.println("Timeout waiting for start bit");
      return 0xFF; // Indicate error
    }
  }
  
  // Start bit detected, wait for half bit period to sample in the middle
  delay(BIT_PERIOD_MS / 2);
  
  // Verify it's still a valid start bit
  if (readDataPin() != 0) {
    Serial.println("Invalid start bit");
    return 0xFF; // Indicate error
  }
  
  // Wait for remainder of the start bit
  delay(BIT_PERIOD_MS / 2);
  
  // Read 8 data bits
  for (int i = 0; i < 8; i++) {
    // Wait for half bit period to sample in the middle
    delay(BIT_PERIOD_MS / 2);
    
    // Read bit
    bit = readDataPin();
    if (bit) {
      byte |= (1 << i);
    }
    
    // Wait for remainder of bit period
    delay(BIT_PERIOD_MS / 2);
  }
  
  // Wait for half bit period to check stop bit in the middle
  delay(BIT_PERIOD_MS / 2);
  
  // Check for valid stop bit (logic 1)
  if (readDataPin() != 1) {
    Serial.println("Invalid stop bit");
    return 0xFF; // Indicate error
  }
  
  // Wait for remainder of stop bit
  delay(BIT_PERIOD_MS / 2);
  
  Serial.print("Received byte: 0x");
  if (byte < 16) Serial.print("0");
  Serial.println(byte, HEX);
  
  return byte;
}

// Send a message with protocol framing
int sendMessage(char *message) {
  int len = strlen(message);
  unsigned char checksum = 0;
  
  // Turn on LED during transmission
  digitalWrite(LED_BUILTIN, HIGH);
  
  Serial.print("Sending message: \"");
  Serial.print(message);
  Serial.print("\" (");
  Serial.print(len);
  Serial.println(" bytes)");
  
  // 1. Send start sequence for synchronization
  sendByte(START_SEQUENCE);
  
  // 2. Send length byte
  sendByte((unsigned char)len);
  checksum ^= len; // XOR for simple checksum
  
  // 3. Send each byte of the message
  for (int i = 0; i < len; i++) {
    sendByte((unsigned char)message[i]);
    checksum ^= message[i]; // Update checksum
  }
  
  // 4. Send checksum
  sendByte(checksum);
  
  // 5. Send end sequence
  sendByte(END_SEQUENCE);
  
  // 6. Wait for ACK/NACK from receiver with timeout
  unsigned long startTime = millis();
  unsigned char response;
  
  setDataPinMode(INPUT);
  response = receiveByte();
  
  if (response == 0xFF) {
    Serial.println("Error or timeout waiting for acknowledgment");
    digitalWrite(LED_BUILTIN, LOW);
    return 0;
  }
  
  if (response == ACK_BYTE) {
    Serial.println("Received ACK - message successfully delivered");
    digitalWrite(LED_BUILTIN, LOW);
    return 1;
  } else {
    Serial.print("Did not receive ACK - got 0x");
    if (response < 16) Serial.print("0");
    Serial.print(response, HEX);
    Serial.println(" instead");
    digitalWrite(LED_BUILTIN, LOW);
    return 0;
  }
}

// Receive a message with protocol framing
int receiveMessage() {
  unsigned char byte, length, checksum = 0, calculated_checksum = 0;
  int i, success = 0;
  
  // Turn on LED during reception
  digitalWrite(LED_BUILTIN, HIGH);
  
  Serial.println("Receiving message...");
  
  // 1. Wait for and verify start sequence
  byte = receiveByte();
  if (byte == 0xFF) {
    Serial.println("Error receiving start byte");
    digitalWrite(LED_BUILTIN, LOW);
    return 0;
  }
  
  if (byte != START_SEQUENCE) {
    Serial.print("Invalid start sequence: 0x");
    if (byte < 16) Serial.print("0");
    Serial.print(byte, HEX);
    Serial.print(", expected 0x");
    if (START_SEQUENCE < 16) Serial.print("0");
    Serial.println(START_SEQUENCE, HEX);
    digitalWrite(LED_BUILTIN, LOW);
    return 0;
  }
  
  // 2. Receive length byte
  length = receiveByte();
  if (length == 0xFF) {
    Serial.println("Error receiving length byte");
    digitalWrite(LED_BUILTIN, LOW);
    return 0;
  }
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
    byte = receiveByte();
    if (byte == 0xFF) {
      Serial.print("Error receiving data byte ");
      Serial.println(i);
      digitalWrite(LED_BUILTIN, LOW);
      return 0;
    }
    rx_buffer[rx_buffer_pos++] = byte;
    calculated_checksum ^= byte;
  }
  
  // Null-terminate the string
  rx_buffer[rx_buffer_pos] = '\0';
  
  // 4. Receive checksum
  checksum = receiveByte();
  if (checksum == 0xFF) {
    Serial.println("Error receiving checksum byte");
    digitalWrite(LED_BUILTIN, LOW);
    return 0;
  }
  
  // 5. Receive end sequence
  byte = receiveByte();
  if (byte == 0xFF) {
    Serial.println("Error receiving end sequence byte");
    digitalWrite(LED_BUILTIN, LOW);
    return 0;
  }
  
  if (byte != END_SEQUENCE) {
    Serial.print("Invalid end sequence: 0x");
    if (byte < 16) Serial.print("0");
    Serial.print(byte, HEX);
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
  
  digitalWrite(LED_BUILTIN, LOW);
  return success;
}

// Check if start bit detected (for polling)
bool startBitDetected() {
  setDataPinMode(INPUT);
  return (readDataPin() == 0);
}

void loop() {
  // Check for incoming message from DE1-SoC
  if (startBitDetected()) {
    if (receiveMessage()) {
      Serial.print("Received message from DE1-SoC: ");
      Serial.println(rx_buffer);
      
      // Auto-respond with an acknowledgment message
      sprintf(tx_buffer, "ARD_ACK_%d", message_counter++);
      delay(100); // Small delay before responding
      sendMessage(tx_buffer);
    }
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
  
  // Small delay for debounce and to prevent CPU hogging
  delay(10);
}