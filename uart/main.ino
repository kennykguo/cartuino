/*
 * Arduino Asynchronous Communication Protocol
 * For communication with DE1-SoC FPGA
 * 
 * Uses software UART-like protocol with single data line
 */

// Pin definitions
#define DATA_PIN 0  // D0 for data line

// Communication parameters
#define BIT_PERIOD_MS 5     // 5ms per bit (200 bps) - must match DE1-SoC
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
  
  // Configure DATA pin - start as output to set a known state
  pinMode(DATA_PIN, OUTPUT);
  digitalWrite(DATA_PIN, HIGH);  // Idle state is HIGH
  delay(100);  // Hold for stability
  
  // Now set to input for receiving
  pinMode(DATA_PIN, INPUT);
  
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
  
  // Reset line state with a break condition
  Serial.println("Sending line reset sequence...");
  pinMode(DATA_PIN, OUTPUT);
  digitalWrite(DATA_PIN, HIGH);
  delay(50);
  pinMode(DATA_PIN, INPUT);
  delay(50);
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
  
  // Ensure line is in idle state (HIGH) before starting
  setDataPin(1);
  delay(BIT_PERIOD_MS * 2);
  
  // Send start bit (always 0)
  // Hold it longer to ensure it's detected
  setDataPin(0);
  delay(BIT_PERIOD_MS * 1.5);
  
  // Send 8 data bits, LSB first
  for (int i = 0; i < 8; i++) {
    int bit = (byte >> i) & 0x01;
    sendBit(bit);
  }
  
  // Send stop bit (always 1) - hold longer for reliability
  setDataPin(1);
  delay(BIT_PERIOD_MS * 2);
}

// Receive a byte (8 bits) LSB first, with start and stop bits
// Returns 0xFF on error
unsigned char receiveByte() {
  unsigned char byte = 0;
  int bit;
  unsigned long startTime;
  int consecutive_zeros = 0;
  
  // Set DATA pin as input
  setDataPinMode(INPUT);
  
  // Wait for a stable line first (should be HIGH in idle)
  startTime = millis();
  while (1) {
    if (readDataPin() == 1) {
      break;  // Line is stable HIGH, ready to detect start bit
    }
    
    if (millis() - startTime > 100) {  // 100ms timeout for line to become stable
      // Try to reset line if it's stuck LOW
      setDataPinMode(OUTPUT);
      setDataPin(1);            // Force HIGH
      delay(20);
      setDataPinMode(INPUT);    // Back to input
      
      startTime = millis();
      consecutive_zeros = 0;
    }
  }
  
  // Wait for valid start bit (logic 0)
  startTime = millis();
  while (1) {
    if (readDataPin() == 0) {
      consecutive_zeros++;
      if (consecutive_zeros >= 3) {  // Require multiple consecutive zeros to confirm start bit
        break;  // Confirmed start bit
      }
    } else {
      consecutive_zeros = 0;  // Reset if we see a HIGH
    }
    
    if (millis() - startTime > MAX_WAIT_TIME_MS) {
      Serial.println("Timeout waiting for start bit");
      return 0xFF; // Indicate error
    }
    delay(1);  // Check every 1ms
  }
  
  // Start bit confirmed - skip to middle of the start bit
  // We've already consumed ~3ms detecting it, so adjust timing
  delay(BIT_PERIOD_MS / 2 - 3);
  
  // Read 8 data bits, sampling in the middle of each bit period
  byte = 0;
  for (int i = 0; i < 8; i++) {
    // Wait for full bit period to get to middle of next bit
    delay(BIT_PERIOD_MS);
    
    // Sample the bit
    bit = readDataPin();
    if (bit) {
      byte |= (1 << i);
    }
  }
  
  // Wait for stop bit (should be HIGH)
  delay(BIT_PERIOD_MS);
  
  // Check for valid stop bit
  if (readDataPin() != 1) {
    Serial.println("Invalid stop bit");
    // Continue anyway - we already have the byte
  }
  
  // Additional delay to ensure we're past the stop bit
  delay(BIT_PERIOD_MS);
  
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

// Check if potential start bit detected (for polling)
bool startBitDetected() {
  static int prev_state = 1;  // Keep track of previous state
  static int low_count = 0;   // Count consecutive low readings
  static unsigned long last_check = 0;
  int current_state;
  
  // Only check every 1ms to avoid excessive polling
  if (millis() - last_check < 1) {
    return false;
  }
  last_check = millis();
  
  setDataPinMode(INPUT);
  current_state = readDataPin();
  
  // If line went from HIGH to LOW
  if (prev_state == 1 && current_state == 0) {
    low_count = 1;
    prev_state = current_state;
    return false;  // Not confirmed yet
  }
  
  // If line stays LOW, increment counter
  if (prev_state == 0 && current_state == 0) {
    low_count++;
    // If we see enough consecutive LOWs, it's likely a real start bit
    if (low_count >= 3) {
      low_count = 0;
      // Don't reset prev_state to maintain edge detection
      return true;  // Confirmed start bit
    }
  }
  
  // Update previous state
  prev_state = current_state;
  
  // If line went back to HIGH, reset counter
  if (current_state == 1) {
    low_count = 0;
  }
  
  return false;  // No confirmed start bit
}

void loop() {
  // Check for incoming message from DE1-SoC
  if (startBitDetected()) {
    Serial.println("Potential start bit detected!");
    
    // Delay just a bit to confirm
    delay(2);
    
    if (readDataPin() == 0) {
      Serial.println("Start bit confirmed, receiving message...");
      
      if (receiveMessage()) {
        Serial.print("Received message from DE1-SoC: ");
        Serial.println(rx_buffer);
        
        // Auto-respond with an acknowledgment message
        sprintf(tx_buffer, "ARD_ACK_%d", message_counter++);
        delay(200); // Longer delay before responding
        sendMessage(tx_buffer);
        
        // After sending, return to input mode to listen
        setDataPinMode(INPUT);
      }
    } else {
      Serial.println("False start bit detection");
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
      
      // Reset line state first
      Serial.println("Resetting line state...");
      setDataPinMode(OUTPUT);
      setDataPin(1);  // Idle HIGH
      delay(100);     // Hold for stability
      
      // Create test message
      sprintf(tx_buffer, "ARD_MSG_%d", message_counter++);
      
      // Send it
      sendMessage(tx_buffer);
      
      // After sending, return to input mode
      setDataPinMode(INPUT);
      
      lastActivityTime = millis();
    }
  }
  
  lastButtonState = reading;
  
  // If it's been 5 seconds since last activity, print diagnostic info
  if (millis() - lastActivityTime > 5000) {
    // Read and print current line state
    setDataPinMode(INPUT);
    int lineState = readDataPin();
    Serial.print("Waiting for activity... Line state: ");
    Serial.println(lineState);
    
    // If line appears stuck LOW, try to reset it
    if (lineState == 0) {
      Serial.println("Line appears stuck LOW, attempting reset...");
      setDataPinMode(OUTPUT);
      setDataPin(1);  // Force HIGH
      delay(50);
      setDataPinMode(INPUT);
    }
    
    lastActivityTime = millis();
  }
  
  // Small delay for debounce and to prevent CPU hogging
  delay(5);
}