/*
 * Arduino Simple Bit-Banged Communication Protocol (Master)
 * For communication with DE1-SoC FPGA
 * Uses D2 for data and D3 for clock, connecting to DE1-SoC D0 and D1
 */

// Pin definitions
#define CLOCK_PIN 3  // D3 for clock line - connects to DE1-SoC D1
#define DATA_PIN 2   // D2 for data line - connects to DE1-SoC D0

// Communication parameters
#define BIT_PERIOD_MS 40    // 40ms per bit - even slower for reliability
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
unsigned long messageSendInterval = 10000;  // Send message every 10 seconds

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
  Serial.println("Will send messages every 10 seconds");
  Serial.println("==============================================");
}

// Send a single bit (simplest approach)
void sendBit(int bit) {
  // First set data line
  digitalWrite(DATA_PIN, bit ? HIGH : LOW);
  delay(10);  // Give time for line to stabilize
  
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
  delay(5);  // Give time for mode change to take effect
  
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
  delay(20);
  
  // Send each bit, MSB first
  for (int i = 7; i >= 0; i--) {
    int bit = (byte >> i) & 0x01;
    sendBit(bit);
  }
  
  // Small pause between bytes for receiver to catch up
  delay(20);
}

// Receive a byte (8 bits) MSB first
unsigned char receiveByte() {
  unsigned char byte = 0;
  
  // Set as input before receiving
  pinMode(DATA_PIN, INPUT);
  delay(10);
  
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

// Send a simple message - just the raw content
void sendSimpleMessage(char *message) {
  int len = strlen(message);
  
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
  
  // Send each byte of the message
  for (int i = 0; i < len; i++) {
    sendByte((unsigned char)message[i]);
  }
  
  // Send end byte
  sendByte(END_BYTE);
  
  // Turn off LED
  digitalWrite(LED_BUILTIN, LOW);
  
  Serial.println("Message sent");
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
  delay(200);
  
  // Send attention sequence - 16 clock pulses with data low
  for (int i = 0; i < 16; i++) {
    digitalWrite(CLOCK_PIN, LOW);
    delay(60);
    digitalWrite(CLOCK_PIN, HIGH);
    delay(60);
  }
  
  // Another reset with longer delay
  digitalWrite(CLOCK_PIN, HIGH);
  digitalWrite(DATA_PIN, LOW);
  delay(300);
  
  Serial.println("Initialization complete");
}

// Main loop
void loop() {
  // Check if it's time to send a message based on interval
  unsigned long currentTime = millis();
  
  if (currentTime - lastMessageTime >= messageSendInterval) {
    Serial.println("\nTime to send a test message");
    
    // Initialize communication to get clean state
    initCommunication();
    
    // Create and send test message
    sprintf(tx_buffer, "ARD_MSG_%d", message_counter++);
    sendSimpleMessage(tx_buffer);
    
    // Update the last message time
    lastMessageTime = currentTime;
  }
  
  // Small delay
  delay(10);
}