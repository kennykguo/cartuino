/*
 * Arduino Communication with DE1-SoC
 * 
 * Uses D0 (data) and D1 (clock) pins for synchronized communication
 * D0 is bidirectional (data), D1 is input only (clock from DE1-SoC)
 */

// Pin definitions - matching DE1-SoC
#define DATA_PIN 0  // D0 for data (bidirectional)
#define CLOCK_PIN 1 // D1 for clock (input from DE1-SoC)

// Communication timing
#define BIT_DELAY_MS 50

// Message buffers
#define MSG_BUFFER_SIZE 64
char rx_buffer[MSG_BUFFER_SIZE];
int rx_buffer_pos = 0;
int message_counter = 0;

// Timestamp tracking
unsigned long lastActivityTime = 0;
unsigned long lastSendTime = 0;

void setup() {
  // Setup USB serial for debugging output
  Serial.begin(115200);
  
  // Configure pins - DATA starts as input, CLOCK always input
  pinMode(DATA_PIN, INPUT);
  pinMode(CLOCK_PIN, INPUT);
  
  // Configure built-in LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  // Wait for serial to initialize
  delay(1000);
  
  Serial.println("\n\n===========================");
  Serial.println("Arduino <-> DE1-SoC Communication");
  Serial.println("===========================");
  Serial.println("- Auto-sending test message every 5 seconds");
  Serial.println("- Monitoring for messages from DE1-SoC");
  Serial.println("===========================");
  
  // Print initial pin states
  Serial.print("Initial DATA pin state: ");
  Serial.println(digitalRead(DATA_PIN) ? "HIGH" : "LOW");
  Serial.print("Initial CLOCK pin state: ");
  Serial.println(digitalRead(CLOCK_PIN) ? "HIGH" : "LOW");
}

void loop() {
  static int lastClockState = -1;
  int clockState = digitalRead(CLOCK_PIN);
  
  // Detect clock transitions (DE1-SoC is signaling us)
  if (clockState != lastClockState) {
    Serial.print("CLOCK pin changed to: ");
    Serial.println(clockState ? "HIGH" : "LOW");
    lastClockState = clockState;
    lastActivityTime = millis();
    
    // If clock goes LOW, DE1-SoC is starting to send
    if (clockState == LOW) {
      Serial.println("DE1-SoC is sending data - receiving...");
      receiveMessage();
    }
  }
  
  // Auto-send a test message every 5 seconds
  if (millis() - lastSendTime > 5000) {
    Serial.println("\n--- Auto-sending test message ---");
    char message[MSG_BUFFER_SIZE];
    sprintf(message, "ARDUINO_MSG_%d", message_counter++);
    sendMessage(message);
    lastSendTime = millis();
  }
  
  // Periodically print pin states for debugging
  static unsigned long lastPinCheckTime = 0;
  if (millis() - lastPinCheckTime > 1000) {
    Serial.print("STATUS: DATA=");
    Serial.print(digitalRead(DATA_PIN) ? "HIGH" : "LOW");
    Serial.print(", CLOCK=");
    Serial.println(digitalRead(CLOCK_PIN) ? "HIGH" : "LOW");
    lastPinCheckTime = millis();
  }
  
  // Small delay
  delay(10);
}

// Send a message to DE1-SoC bit by bit
void sendMessage(char *message) {
  int len = strlen(message);
  
  Serial.print("Sending message: \"");
  Serial.print(message);
  Serial.print("\" (");
  Serial.print(len);
  Serial.println(" bytes)");
  
  // Turn on LED during transmission
  digitalWrite(LED_BUILTIN, HIGH);
  
  // Set DATA pin to output mode
  pinMode(DATA_PIN, OUTPUT);
  Serial.println("DATA pin set to OUTPUT mode");
  
  // Send each byte of the message
  for (int i = 0; i < len; i++) {
    byte b = message[i];
    Serial.print("Sending byte ");
    Serial.print(i+1);
    Serial.print("/");
    Serial.print(len);
    Serial.print(": 0x");
    if (b < 16) Serial.print("0");
    Serial.print(b, HEX);
    Serial.print(" ('");
    if (b >= 32 && b <= 126) Serial.print((char)b);
    else Serial.print(".");
    Serial.println("')");
    
    // Send 8 bits, MSB first
    for (int j = 7; j >= 0; j--) {
      int bit = (b >> j) & 0x01;
      
      // Set the bit value
      digitalWrite(DATA_PIN, bit ? HIGH : LOW);
      Serial.print("  Setting bit ");
      Serial.print(j);
      Serial.print(" to ");
      Serial.println(bit);
      
      // Need to wait for full bit period since we don't control the clock
      delay(BIT_DELAY_MS);
    }
    
    // Small delay between bytes
    delay(BIT_DELAY_MS);
  }
  
  // Return DATA pin to input mode
  pinMode(DATA_PIN, INPUT);
  Serial.println("DATA pin set back to INPUT mode");
  
  // Turn off LED
  digitalWrite(LED_BUILTIN, LOW);
  
  Serial.println("Message transmission complete");
}

// Receive a message from DE1-SoC
void receiveMessage() {
  rx_buffer_pos = 0;
  boolean receiving = true;
  unsigned long startTime = millis();
  
  // Ensure DATA pin is in input mode
  pinMode(DATA_PIN, INPUT);
  
  // First clock transition already detected before calling this function
  digitalWrite(LED_BUILTIN, HIGH); // Turn on LED while receiving
  
  // Process bytes until we get a timeout or buffer is full
  while (receiving && rx_buffer_pos < MSG_BUFFER_SIZE-1) {
    byte receivedByte = 0;
    
    // Read 8 bits (MSB first)
    for (int i = 7; i >= 0; i--) {
      // Wait for clock to go HIGH (end of previous bit)
      unsigned long bitStartTime = millis();
      while (digitalRead(CLOCK_PIN) == LOW) {
        // Timeout if clock stays LOW too long
        if (millis() - bitStartTime > BIT_DELAY_MS * 4) {
          Serial.println("Clock timeout waiting for HIGH - end of message?");
          receiving = false;
          break;
        }
      }
      
      if (!receiving) break;
      
      // Wait for clock to go LOW (bit ready to read)
      bitStartTime = millis();
      while (digitalRead(CLOCK_PIN) == HIGH) {
        // Timeout if clock stays HIGH too long
        if (millis() - bitStartTime > BIT_DELAY_MS * 4) {
          Serial.println("Clock timeout waiting for LOW - end of message?");
          receiving = false;
          break;
        }
      }
      
      if (!receiving) break;
      
      // Read the bit value
      int bit = digitalRead(DATA_PIN);
      Serial.print("Read bit ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(bit);
      
      // Store the bit
      if (bit) {
        receivedByte |= (1 << i);
      }
    }
    
    // If we got a full byte, add it to the buffer
    if (receiving) {
      rx_buffer[rx_buffer_pos++] = receivedByte;
      Serial.print("Received byte: 0x");
      if (receivedByte < 16) Serial.print("0");
      Serial.print(receivedByte, HEX);
      Serial.print(" ('");
      if (receivedByte >= 32 && receivedByte <= 126) Serial.print((char)receivedByte);
      else Serial.print(".");
      Serial.println("')");
    }
    
    // Check for end of message (clock stays HIGH for longer than expected)
    if (digitalRead(CLOCK_PIN) == HIGH) {
      unsigned long endCheckTime = millis();
      while (digitalRead(CLOCK_PIN) == HIGH) {
        if (millis() - endCheckTime > BIT_DELAY_MS * 3) {
          Serial.println("Detected end of message (CLOCK HIGH for extended period)");
          receiving = false;
          break;
        }
      }
    }
    
    // Overall timeout safety
    if (millis() - startTime > 5000) {
      Serial.println("Receive timeout - aborting");
      receiving = false;
    }
  }
  
  // Null-terminate the buffer
  rx_buffer[rx_buffer_pos] = '\0';
  
  // Print the complete message
  if (rx_buffer_pos > 0) {
    Serial.print("Complete message received: \"");
    Serial.print(rx_buffer);
    Serial.print("\" (");
    Serial.print(rx_buffer_pos);
    Serial.println(" bytes)");
  } else {
    Serial.println("No valid message received");
  }
  
  digitalWrite(LED_BUILTIN, LOW); // Turn off LED
}