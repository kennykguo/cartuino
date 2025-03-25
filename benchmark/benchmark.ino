/*
 * Arduino Simplified Communication Protocol (Master)
 * Using explicit bit-banging with maximum reliability
 * D2 - Data (to DE1-SoC D0)
 * D3 - Clock (to DE1-SoC D1)
 * D5 - Sync (to DE1-SoC D2)
 */

// Pin definitions
#define DATA_PIN 2   // For data line
#define CLOCK_PIN 3  // For clock line
#define SYNC_PIN 5   // For sync/handshake line (D5)

// Communication parameters
#define BIT_PERIOD_MS 10    // Slow but reliable bit transfer (10ms per bit)
#define MSG_BUFFER_SIZE 32  // Buffer size for messages
char tx_buffer[MSG_BUFFER_SIZE];
int message_counter = 0;

// Protocol constants
#define START_BYTE 0xAA  // 10101010 pattern for sync

void setup() {
  Serial.begin(9600);
  
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
  
  Serial.println("\nSimplified Communication Protocol Ready");
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

// Send a byte (8 bits) MSB first
void sendByte(unsigned char byte) {
  Serial.print("TX: 0x");
  if (byte < 16) Serial.print("0");
  Serial.println(byte, HEX);
  
  // Send each bit, MSB first
  for (int i = 7; i >= 0; i--) {
    int bit = (byte >> i) & 0x01;
    sendBit(bit);
  }
  
  // Pause between bytes
  delay(10);
}

// Send a simple message with sync protocol
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
  Serial.println("Ending transmission");
  delay(50);
  digitalWrite(SYNC_PIN, LOW);
  
  // Turn off LED
  digitalWrite(LED_BUILTIN, LOW);
  
  Serial.println("Message sent successfully");
}

// Main loop
void loop() {
  static unsigned long lastTime = 0;
  unsigned long currentTime = millis();
  
  // Send message every 5 seconds
  if (currentTime - lastTime >= 5000) {
    Serial.println("\n----- SENDING NEW MESSAGE -----");
    sendMessage();
    lastTime = currentTime;
  }
  
  delay(10);
}