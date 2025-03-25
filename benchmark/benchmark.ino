// Pin definitions
const int TX_PIN = 2;  // Data output pin
const int RX_PIN = 3;  // Data input pin
const int KEY0_PIN = 7;  // Button for transmission trigger

// Communication parameters
const unsigned long BIT_RATE = 500;  // 500 bps = 2ms per bit
const unsigned long BIT_DELAY = 1000000 / BIT_RATE;  // Microseconds per bit

// Test message
const char* TEST_MESSAGE = "Hello from Arduino!";

// Receive buffer
char receivedMessage[100];
int msgIndex = 0;

void setup() {
  pinMode(TX_PIN, OUTPUT);
  pinMode(RX_PIN, INPUT);
  pinMode(KEY0_PIN, INPUT_PULLUP);
  
  Serial.begin(9600);  // For debugging
  Serial.println("Arduino ready");
  
  // Initialize TX pin to idle state (HIGH)
  digitalWrite(TX_PIN, HIGH);
}

void loop() {
  // Check if button is pressed to send a message
  if (digitalRead(KEY0_PIN) == LOW) {
    delay(50);  // Debounce
    if (digitalRead(KEY0_PIN) == LOW) {
      Serial.println("Sending message");
      sendMessage(TEST_MESSAGE);
      
      // Wait for button release
      while (digitalRead(KEY0_PIN) == LOW) delay(10);
    }
  }
  
  // Check for incoming data (start bit)
  if (digitalRead(RX_PIN) == LOW) {
    char receivedChar = receiveByte();
    Serial.print("Received char: ");
    Serial.println(receivedChar);
  }
}

// Send a single byte
void sendByte(char c) {
  // Start bit (LOW)
  digitalWrite(TX_PIN, LOW);
  delayMicroseconds(BIT_DELAY);
  
  // Data bits (LSB first)
  for (int bit = 0; bit < 8; bit++) {
    digitalWrite(TX_PIN, (c >> bit) & 1);
    delayMicroseconds(BIT_DELAY);
  }
  
  // Stop bit (HIGH)
  digitalWrite(TX_PIN, HIGH);
  delayMicroseconds(BIT_DELAY);
  
  // Extra delay between bytes for reliability
  delayMicroseconds(BIT_DELAY);
}

// Send a complete message
void sendMessage(const char* message) {
  for (int i = 0; message[i] != '\0'; i++) {
    sendByte(message[i]);
  }
  // Send newline to mark end of message
  sendByte('\n');
}

// Receive a single byte
char receiveByte() {
  char receivedChar = 0;
  
  // We've already detected the start bit, now wait until middle of bit
  delayMicroseconds(BIT_DELAY / 2);
  
  // Wait until middle of first data bit
  delayMicroseconds(BIT_DELAY);
  
  // Read 8 data bits (LSB first)
  for (int bit = 0; bit < 8; bit++) {
    bool bitValue = digitalRead(RX_PIN);
    if (bitValue) {
      receivedChar |= (1 << bit);
    }
    delayMicroseconds(BIT_DELAY);
  }
  
  // Wait for stop bit to pass
  delayMicroseconds(BIT_DELAY);
  
  // Add to message buffer
  receivedMessage[msgIndex++] = receivedChar;
  
  // Check if we've received a complete message (newline)
  if (receivedChar == '\n' || msgIndex >= sizeof(receivedMessage) - 1) {
    receivedMessage[msgIndex] = '\0';
    Serial.print("Received message: ");
    Serial.println(receivedMessage);
    msgIndex = 0;
  }
  
  return receivedChar;
}