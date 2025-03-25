/*
 * Arduino Simple Bit-Banged Communication Protocol (Master)
 * For communication with DE1-SoC FPGA
 */

// Pin definitions - using D2 and D3 instead of D0/D1 to avoid UART conflicts
#define CLOCK_PIN 3  // D3 for clock line - connects to DE1-SoC D1
#define DATA_PIN 2   // D2 for data line - connects to DE1-SoC D0

// Communication parameters
#define BIT_PERIOD_MS 5     // 5ms per bit (200 bits/second) - adjust for reliability
#define MSG_BUFFER_SIZE 32  // Buffer size for messages
char tx_buffer[MSG_BUFFER_SIZE];
char rx_buffer[MSG_BUFFER_SIZE];

// Message counter
int message_counter = 0;

// Protocol constants
#define START_BYTE 0xAA  // 10101010 pattern for synchronization
#define END_BYTE 0x55    // 01010101 pattern for end of message

// Timer for automatic message sending
unsigned long lastMessageTime = 0;
unsigned long messageSendInterval = 5000;  // Send message every 5 seconds

void setup() {
  // Setup USB serial for debugging output
  Serial.begin(9600);
  
  // Configure pins
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, INPUT);  // Start as input
  
  // Initialize clock to HIGH (idle state)
  digitalWrite(CLOCK_PIN, HIGH);
  
  // No button configuration needed
  
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
  Serial.println("Press button to initiate message exchange");
  Serial.println("==============================================");
}

// Set the DATA pin direction with stabilization delay
void setDataPinMode(int mode) {
  pinMode(DATA_PIN, mode);
  // Add pull-down when in INPUT mode to prevent floating
  if (mode == INPUT) {
    pinMode(DATA_PIN, INPUT);  // No pullup
  }
  delayMicroseconds(500);  // Allow pin mode to stabilize
}

// Set the DATA pin value (when configured as OUTPUT)
void setDataPin(int high) {
  digitalWrite(DATA_PIN, high ? HIGH : LOW);
  delayMicroseconds(200);  // Allow pin value to stabilize
}

// Read the DATA pin value
int readDataPin() {
  int val = digitalRead(DATA_PIN);
  return val;
}

// Send a single bit with clock control
void sendBit(int bit) {
  // Set the data line first
  setDataPinMode(OUTPUT);
  setDataPin(bit);
  delay(1);  // Small setup delay
  
  // Toggle clock to signal bit is ready (HIGH to LOW)
  digitalWrite(CLOCK_PIN, LOW);
  delay(BIT_PERIOD_MS / 2);
  
  // Toggle clock back (LOW to HIGH) to complete bit transmission
  digitalWrite(CLOCK_PIN, HIGH);
  delay(BIT_PERIOD_MS / 2);
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

// Receive a single bit by controlling the clock
int receiveBit() {
  int bit;
  
  // Set DATA pin as input
  setDataPinMode(INPUT);
  delay(1);  // Small setup delay
  
  // Toggle clock to signal ready to receive (HIGH to LOW)
  digitalWrite(CLOCK_PIN, LOW);
  delay(BIT_PERIOD_MS / 2);
  
  // Read the data bit
  bit = readDataPin();
  
  // Toggle clock back (LOW to HIGH) to complete bit reception
  digitalWrite(CLOCK_PIN, HIGH);
  delay(BIT_PERIOD_MS / 2);
  
  return bit;
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

// Send a message with basic framing
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
  Serial.println("Message sent successfully");
  
  // Reset data pin to input with clear state before receiving
  setDataPinMode(INPUT);
  delay(50);  // Ensure clean transition
}

// Receive a message with basic framing
bool receiveMessage() {
  unsigned char byte, length, checksum = 0, calculated_checksum = 0;
  bool success = false;
  
  // Turn on LED during reception
  digitalWrite(LED_BUILTIN, HIGH);
  
  Serial.println("Waiting for message from DE1-SoC...");
  
  // Wait for start byte with retry
  int retries = 5;
  bool startFound = false;
  
  while (retries > 0 && !startFound) {
    byte = receiveByte();
    if (byte == START_BYTE) {
      startFound = true;
      Serial.println("Start byte detected successfully");
    } else {
      Serial.print("Invalid start byte: 0x");
      if (byte < 16) Serial.print("0");
      Serial.print(byte, HEX);
      Serial.print(", expected 0x");
      if (START_BYTE < 16) Serial.print("0");
      Serial.println(START_BYTE, HEX);
      
      // Try sending a few clock pulses to reset synchronization
      for(int i=0; i<2; i++) {
        digitalWrite(CLOCK_PIN, LOW);
        delay(BIT_PERIOD_MS/2);
        digitalWrite(CLOCK_PIN, HIGH);
        delay(BIT_PERIOD_MS/2);
      }
      
      retries--;
    }
  }
  
  if (!startFound) {
    Serial.println("Failed to receive valid start byte after retries");
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
    Serial.println("Invalid end byte");
    success = false;
  } else if (checksum != calculated_checksum) {
    Serial.println("Checksum error");
    success = false;
  } else {
    Serial.print("Message received successfully: \"");
    Serial.print(rx_buffer);
    Serial.println("\"");
    success = true;
  }
  
  // Turn off LED
  digitalWrite(LED_BUILTIN, LOW);
  return success;
}

void loop() {
  // Check if it's time to send a message based on interval
  unsigned long currentTime = millis();
  
  if (currentTime - lastMessageTime >= messageSendInterval) {
    Serial.println("\nTimer triggered - Initiating message exchange");
    
    // Create and send test message
    sprintf(tx_buffer, "ARD_MSG_%d", message_counter++);
    sendMessage(tx_buffer);
    
    // Brief delay to allow DE1-SoC to prepare for sending
    delay(200);
    
    // Extra synchronization - send a few clock pulses to reset state
    for(int i=0; i<3; i++) {
      digitalWrite(CLOCK_PIN, LOW);
      delay(BIT_PERIOD_MS/2);
      digitalWrite(CLOCK_PIN, HIGH);
      delay(BIT_PERIOD_MS/2);
    }
    
    // Wait for response from DE1-SoC
    bool responseReceived = receiveMessage();
    
    if (responseReceived) {
      Serial.println("Message exchange completed successfully");
    } else {
      Serial.println("Failed to complete message exchange");
    }
    
    // Update the last message time
    lastMessageTime = currentTime;
  }
  
  // Small delay
  delay(10);
}