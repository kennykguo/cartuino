// Feather 32u4 UART Debug Script
// Enhanced to troubleshoot one-way communication issues

// Setting to match the DE1-SoC's baud rate
#define BAUD_RATE 9600

// Buffers for received data
#define RX_BUFFER_SIZE 64
char rx_buffer[RX_BUFFER_SIZE];
int rx_count = 0;

// LED control constants
#define LED_ON_TIME 100    // LED blink time in ms
#define LED_RECEIVED 0     // LED pattern for received data
#define LED_TRANSMIT 1     // LED pattern for transmitted data
#define LED_ERROR 2        // LED pattern for errors

// Test mode
enum TestMode {
  MODE_NORMAL,       // Regular echo test
  MODE_HEX_DISPLAY,  // Display received bytes as hex
  MODE_BINARY_CHECK, // Check binary patterns
  MODE_LOOPBACK      // Internal loopback test
};

TestMode currentMode = MODE_HEX_DISPLAY;  // Default to hex display for debugging

// Tracking variables
unsigned long lastHeartbeatTime = 0;
unsigned long lastTransmitTime = 0;
unsigned long messageCount = 0;
unsigned long heartbeatCount = 0;
unsigned long errorCount = 0;
unsigned long rxTimeoutCount = 0;
boolean ledState = false;
unsigned long ledBlinkTime = 0;

// Signal quality metrics
unsigned long totalBytesReceived = 0;
unsigned long framesWithErrors = 0;

void setup() {
  // Set up serial communication for debugging with computer
  Serial.begin(BAUD_RATE);
  
  // Wait for serial connection to establish
  delay(3000);
  
  Serial.println("\n\n==================================");
  Serial.println("Feather 32u4 UART Debug Tool");
  Serial.println("==================================");
  
  // Set up hardware serial for DE1-SoC communication
  Serial1.begin(BAUD_RATE);
  
  // Configure LED pin for status indication
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Blink LED pattern to indicate setup and current mode
  blinkLEDPattern(currentMode);
  
  Serial.println("Setup complete");
  Serial.print("Operating mode: ");
  printCurrentMode();
  Serial.print("Baud rate: ");
  Serial.println(BAUD_RATE);
  Serial.println("Waiting for DE1-SoC communication...");
  Serial.println("==================================");
  
  // Print help information
  printHelp();
}

void loop() {
  // Process button inputs if available
  checkSerialCommands();
  
  // Send heartbeat message to Serial monitor periodically
  unsigned long currentTime = millis();
  if (currentTime - lastHeartbeatTime >= 5000) {  // Every 5 seconds
    heartbeatCount++;
    printHeartbeat();
    lastHeartbeatTime = currentTime;
  }
  
  // Send test message to DE1-SoC periodically
  if (currentTime - lastTransmitTime >= 5000) {  // Every 5 seconds
    sendTestMessage();
    lastTransmitTime = currentTime;
  }
  
  // Check for incoming data from DE1-SoC
  checkForIncomingData();
  
  // Process LED blinking
  updateLED(currentTime);
  
  // In loopback mode, pass data from Serial to Serial1
  if (currentMode == MODE_LOOPBACK && Serial.available() > 0) {
    char c = Serial.read();
    Serial1.write(c);
    Serial.print("Loopback Serial->Serial1: ");
    Serial.println(c);
  }
}

void printHeartbeat() {
  Serial.print("\nHeartbeat #");
  Serial.print(heartbeatCount);
  Serial.print(" - Messages received: ");
  Serial.print(messageCount);
  Serial.print(", Errors: ");
  Serial.print(errorCount);
  Serial.print(", Mode: ");
  printCurrentMode();
  
  // Print signal quality if we've received any data
  if (totalBytesReceived > 0) {
    float errorRate = (float)framesWithErrors / totalBytesReceived * 100.0;
    Serial.print("Signal quality: ");
    Serial.print(errorRate, 1);
    Serial.println("% error rate");
  }
}

void printCurrentMode() {
  switch (currentMode) {
    case MODE_NORMAL:
      Serial.println("NORMAL (echo)");
      break;
    case MODE_HEX_DISPLAY:
      Serial.println("HEX DISPLAY");
      break;
    case MODE_BINARY_CHECK:
      Serial.println("BINARY CHECK");
      break;
    case MODE_LOOPBACK:
      Serial.println("LOOPBACK");
      break;
  }
}

void sendTestMessage() {
  // Create different message types based on mode
  String message = "FEATHER_";
  
  switch (currentMode) {
    case MODE_NORMAL:
      message += "MSG_" + String(heartbeatCount);
      break;
    case MODE_HEX_DISPLAY:
      message += "HEX_" + String(heartbeatCount);
      break;
    case MODE_BINARY_CHECK:
      message += "BIN_" + String(heartbeatCount);
      // Also send some binary test patterns
      Serial1.write(0x55);  // 01010101
      Serial1.write(0xAA);  // 10101010
      break;
    case MODE_LOOPBACK:
      // Don't send messages in loopback mode
      return;
  }
  
  // Send the message
  Serial.print("Sending to DE1-SoC: ");
  Serial.println(message);
  Serial1.println(message);
  
  // Blink LED to indicate transmission
  blinkLED(LED_TRANSMIT);
}

void checkForIncomingData() {
  // Check if data is available
  if (Serial1.available() > 0) {
    // Turn on LED to indicate received data
    digitalWrite(LED_BUILTIN, HIGH);
    ledState = true;
    ledBlinkTime = millis();
    
    // Read data with a timeout
    rx_count = 0;
    unsigned long startTime = millis();
    
    // Read all available data or until buffer full
    while (Serial1.available() > 0 && rx_count < RX_BUFFER_SIZE - 1) {
      rx_buffer[rx_count] = Serial1.read();
      rx_count++;
      
      // Check for timeout
      if (millis() - startTime > 100) {  // 100ms timeout
        rxTimeoutCount++;
        Serial.println("Timeout while reading data");
        break;
      }
    }
    
    // Null terminate the buffer
    rx_buffer[rx_count] = '\0';
    
    // Process the received data
    processReceivedData();
    
    // Increment counters
    messageCount++;
    totalBytesReceived += rx_count;
  }
}

void processReceivedData() {
  // Process data based on current mode
  switch (currentMode) {
    case MODE_NORMAL:
      // Print as string if mostly printable
      Serial.print("Received from DE1-SoC: \"");
      for (int i = 0; i < rx_count; i++) {
        if (rx_buffer[i] >= 32 && rx_buffer[i] <= 126) {
          Serial.print((char)rx_buffer[i]);
        } else {
          Serial.print("\\x");
          Serial.print(rx_buffer[i], HEX);
        }
      }
      Serial.println("\"");
      
      // Echo back what we received
      Serial1.write(rx_buffer, rx_count);
      Serial.println("Message echoed back to DE1-SoC");
      break;
      
    case MODE_HEX_DISPLAY:
      // Display in hex format for debugging
      Serial.print("Received bytes (hex): ");
      for (int i = 0; i < rx_count; i++) {
        if (i > 0) Serial.print(" ");
        if (rx_buffer[i] < 0x10) Serial.print("0");
        Serial.print(rx_buffer[i], HEX);
      }
      Serial.println();
      
      // Also try to print as ASCII
      Serial.print("As ASCII: \"");
      for (int i = 0; i < rx_count; i++) {
        if (rx_buffer[i] >= 32 && rx_buffer[i] <= 126) {
          Serial.print((char)rx_buffer[i]);
        } else {
          Serial.print(".");
        }
      }
      Serial.println("\"");
      
      // Also print individual bits for debugging
      if (rx_count > 0) {
        Serial.print("First byte as bits: ");
        printBits(rx_buffer[0]);
        Serial.println();
      }
      
      // Echo back what we received
      Serial1.write(rx_buffer, rx_count);
      break;
      
    case MODE_BINARY_CHECK:
      // Check for specific binary patterns
      checkBinaryPatterns();
      break;
      
    case MODE_LOOPBACK:
      // In loopback mode, data is passed from Serial1 to Serial
      Serial.print("Loopback from Serial1 to Serial: ");
      Serial.write(rx_buffer, rx_count);
      Serial.println();
      break;
  }
}

void checkBinaryPatterns() {
  Serial.print("Binary pattern check: ");
  
  for (int i = 0; i < rx_count; i++) {
    byte b = rx_buffer[i];
    
    // Check for 0x55 (01010101)
    if (b == 0x55) {
      Serial.print("Found 0x55 pattern at position ");
      Serial.println(i);
    }
    // Check for 0xAA (10101010)
    else if (b == 0xAA) {
      Serial.print("Found 0xAA pattern at position ");
      Serial.println(i);
    }
    // Otherwise print the byte
    else {
      Serial.print("Other byte: 0x");
      if (b < 0x10) Serial.print("0");
      Serial.print(b, HEX);
      Serial.print(" (");
      printBits(b);
      Serial.println(")");
    }
  }
}

void printBits(byte b) {
  for (int bit = 7; bit >= 0; bit--) {
    Serial.print((b & (1 << bit)) ? '1' : '0');
  }
}

void blinkLED(int pattern) {
  switch (pattern) {
    case LED_RECEIVED:
      // Single blink for received data
      digitalWrite(LED_BUILTIN, HIGH);
      ledState = true;
      ledBlinkTime = millis();
      break;
      
    case LED_TRANSMIT:
      // Double blink for transmit
      digitalWrite(LED_BUILTIN, HIGH);
      delay(50);
      digitalWrite(LED_BUILTIN, LOW);
      delay(50);
      digitalWrite(LED_BUILTIN, HIGH);
      ledState = true;
      ledBlinkTime = millis();
      break;
      
    case LED_ERROR:
      // Triple fast blink for error
      for (int i = 0; i < 3; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(30);
        digitalWrite(LED_BUILTIN, LOW);
        delay(30);
      }
      ledState = false;
      break;
  }
}

void blinkLEDPattern(int mode) {
  // Blink LED in a pattern that indicates the current mode
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  
  // Blink (mode+1) times
  for (int i = 0; i <= mode; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  }
  
  delay(500);
}

void updateLED(unsigned long currentTime) {
  // Turn off LED after blink time if it's on
  if (ledState && (currentTime - ledBlinkTime > LED_ON_TIME)) {
    digitalWrite(LED_BUILTIN, LOW);
    ledState = false;
  }
}

void checkSerialCommands() {
  // Check for commands from Serial (computer)
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    // Process commands
    switch (cmd) {
      case '1':
        // Switch to normal mode
        currentMode = MODE_NORMAL;
        Serial.println("Switched to NORMAL mode");
        blinkLEDPattern(currentMode);
        break;
        
      case '2':
        // Switch to hex display mode
        currentMode = MODE_HEX_DISPLAY;
        Serial.println("Switched to HEX DISPLAY mode");
        blinkLEDPattern(currentMode);
        break;
        
      case '3':
        // Switch to binary check mode
        currentMode = MODE_BINARY_CHECK;
        Serial.println("Switched to BINARY CHECK mode");
        blinkLEDPattern(currentMode);
        break;
        
      case '4':
        // Switch to loopback mode
        currentMode = MODE_LOOPBACK;
        Serial.println("Switched to LOOPBACK mode");
        blinkLEDPattern(currentMode);
        break;
        
      case 't':
        // Send test message immediately
        sendTestMessage();
        break;
        
      case 'p':
        // Send a pattern of bytes useful for debugging
        Serial.println("Sending binary pattern test");
        Serial1.write(0x55); // 01010101
        Serial1.write(0xAA); // 10101010
        Serial1.write(0xFF); // 11111111
        Serial1.write(0x00); // 00000000
        Serial1.write('U');  // ASCII 'U' (0x55)
        Serial1.write('*');  // ASCII '*' (0x2A)
        Serial.println("Pattern sent");
        break;
        
      case 's':
        // Status report
        printHeartbeat();
        break;
        
      case 'r':
        // Reset counters
        messageCount = 0;
        errorCount = 0;
        totalBytesReceived = 0;
        framesWithErrors = 0;
        rxTimeoutCount = 0;
        Serial.println("Counters reset");
        break;
        
      case 'b':
        // Change baud rate
        Serial.println("Enter new baud rate (1=9600, 2=19200, 3=38400, 4=57600, 5=115200):");
        while (!Serial.available()) {
          // Wait for input
        }
        int baudChoice = Serial.read() - '0';
        int newBaud;
        
        switch (baudChoice) {
          case 1: newBaud = 9600; break;
          case 2: newBaud = 19200; break;
          case 3: newBaud = 38400; break;
          case 4: newBaud = 57600; break;
          case 5: newBaud = 115200; break;
          default: 
            Serial.println("Invalid choice, staying at current baud rate");
            return;
        }
        
        Serial.print("Changing to ");
        Serial.print(newBaud);
        Serial.println(" baud");
        Serial1.end();
        Serial1.begin(newBaud);
        break;
        
      case 'h':
        // Print help
        printHelp();
        break;
        
      default:
        // Ignore other characters (like newlines)
        break;
    }
    
    // Consume any remaining characters
    while (Serial.available()) {
      Serial.read();
    }
  }
}

void printHelp() {
  Serial.println("\nCommands:");
  Serial.println("1-4: Switch modes (1=Normal, 2=Hex, 3=Binary, 4=Loopback)");
  Serial.println("t: Send test message now");
  Serial.println("p: Send binary pattern test");
  Serial.println("s: Show status");
  Serial.println("r: Reset counters");
  Serial.println("b: Change baud rate");
  Serial.println("h: Show this help");
  
  Serial.println("\nMode descriptions:");
  Serial.println("- NORMAL: Basic echo mode, prints received data as text");
  Serial.println("- HEX DISPLAY: Shows received data in hex format");
  Serial.println("- BINARY CHECK: Tests for specific bit patterns");
  Serial.println("- LOOPBACK: Passes data between Serial and Serial1");
  
  Serial.println("\nLED patterns:");
  Serial.println("- Mode indicator: 1-4 blinks at startup");
  Serial.println("- Single blink: Data received");
  Serial.println("- Double blink: Data transmitted");
  Serial.println("- Triple fast blink: Error detected");
}