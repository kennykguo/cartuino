/*
 * Arduino Simple Communication Protocol (Master)
 * For communication with DE1-SoC FPGA
 * Uses D2 for data and D3 for clock, connecting to DE1-SoC D0 and D1
 */

// Pin definitions
#define CLOCK_PIN 3  // D3 for clock line - connects to DE1-SoC D1
#define DATA_PIN 2   // D2 for data line - connects to DE1-SoC D0

// Communication parameters
#define BIT_PERIOD_MS 50    // 50ms per bit for extreme reliability
#define MSG_BUFFER_SIZE 32  // Buffer size for messages
char tx_buffer[MSG_BUFFER_SIZE];
char rx_buffer[MSG_BUFFER_SIZE];

// Message counter and timing
int message_counter = 0;
unsigned long lastMessageTime = 0;
unsigned long messageSendInterval = 10000;  // 10 seconds between message attempts

// Protocol constants
#define START_BYTE 0xAA  // 10101010 pattern for synchronization

void setup() {
  // Setup USB serial for debugging output
  Serial.begin(9600);
  
  // Configure pins
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, INPUT);  // Start in input mode to avoid conflicts
  
  // Initialize pins to known states
  digitalWrite(CLOCK_PIN, HIGH);  // Idle state is HIGH
  
  // Configure built-in LED
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Initial LED flash to show we're ready
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
  
  Serial.println("\n\n");
  Serial.println("==============================================");
  Serial.println("Arduino Master Communication Protocol Ready");
  Serial.println("==============================================");
  Serial.println("Using D2 (data) and D3 (clock) to DE1-SoC D0/D1");
  Serial.println("Will attempt to send a message every 10 seconds");
  Serial.println("==============================================");
}

// Send a single bit with very explicit timing
void sendBit(int bit) {
  // Set data pin as output
  pinMode(DATA_PIN, OUTPUT);
  
  // Set data line to the bit value
  digitalWrite(DATA_PIN, bit ? HIGH : LOW);
  delay(10);  // Long setup time
  
  // Clock low
  digitalWrite(CLOCK_PIN, LOW);
  delay(BIT_PERIOD_MS / 2);
  
  // Clock high
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
  
  // Small pause between bytes
  delay(20);
}

// Reset communication bus to known state
void resetBus() {
  // Release data line first
  pinMode(DATA_PIN, INPUT);
  delay(20);
  
  // Reset clock to HIGH (idle state)
  digitalWrite(CLOCK_PIN, HIGH);
  delay(100);
  
  // Generate reset pattern - 8 clock pulses with data as input
  for (int i = 0; i < 8; i++) {
    digitalWrite(CLOCK_PIN, LOW);
    delay(30);
    digitalWrite(CLOCK_PIN, HIGH);
    delay(30);
  }
  
  // Another pause in idle state
  delay(100);
}

// Initialize communication
void initCommunication() {
  Serial.println("Initializing communication sequence...");
  
  // Reset the bus first
  resetBus();
  
  // Send strong synchronization pattern - 16 clock pulses
  for (int i = 0; i < 16; i++) {
    digitalWrite(CLOCK_PIN, LOW);
    delay(50);
    digitalWrite(CLOCK_PIN, HIGH);
    delay(50);
  }
  
  // Long pause in idle state
  delay(200);
  
  Serial.println("Initialization complete");
}

// Send a simple message - just the content with start byte
void sendMessage() {
  // Create test message
  sprintf(tx_buffer, "ARD_MSG_%d", message_counter++);
  int len = strlen(tx_buffer);
  
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
  
  // Send each byte of the message
  for (int i = 0; i < len; i++) {
    sendByte((unsigned char)tx_buffer[i]);
  }
  
  // Turn off LED
  digitalWrite(LED_BUILTIN, LOW);
  
  Serial.println("Message sent successfully");
}

// Perform a complete message send
void performMessageSend() {
  // Initialize communication
  initCommunication();
  
  // Send the message
  sendMessage();
  
  // Reset bus to clean state
  resetBus();
}

// Main loop
void loop() {
  unsigned long currentTime = millis();
  
  // If it's time to send a new message
  if (currentTime - lastMessageTime >= messageSendInterval) {
    Serial.println("\n\nStarting new message transmission");
    
    // Perform message send
    performMessageSend();
    
    // Update timing
    lastMessageTime = currentTime;
  }
  
  // Small delay
  delay(10);
}