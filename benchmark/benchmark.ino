/*
 * Arduino UART and Manchester Decoding Script
 * For communication with DE1-SoC FPGA
 * 
 * Handles both standard UART and Manchester encoding
 */

// Pin definitions
#define RX_PIN 0  // Hardware UART RX pin (D0)
#define TX_PIN 1  // Hardware UART TX pin (D1)

// Pulse and pattern detection variables
unsigned long lastChangeTime = 0;
int lastPinState = HIGH;
int pulseCount = 0;
unsigned long pulseStartTime = 0;

// Manchester decoding variables
bool manchesterMode = false;
unsigned long lastRiseTime = 0;
unsigned long lastFallTime = 0;
int startSequenceCount = 0;
int bitPosition = 7;  // MSB first
byte currentByte = 0;
bool inStartSequence = false;
bool inEndSequence = false;

// UART state
bool uartActive = false;
unsigned long lastByteTime = 0;

void setup() {
  // Setup USB serial for debugging output
  Serial.begin(9600);
  
  // Setup hardware UART for DE1-SoC communication
  Serial1.begin(600);  // Slower baud rate for reliability
  Serial1.setTimeout(500);  // Longer timeout for slow baud
  
  // Configure direct pin monitoring in addition to Serial1
  pinMode(RX_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Initial LED state
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  
  Serial.println("\n\n");
  Serial.println("==============================================");
  Serial.println("Arduino DE1-SoC Multi-Protocol Receiver Ready");
  Serial.println("==============================================");
  Serial.println("Monitoring for:");
  Serial.println("1. UART data at 600 baud");
  Serial.println("2. Pulse patterns (100/300ms and 1s)");
  Serial.println("3. Manchester encoded data");
  Serial.println("==============================================");
}

void loop() {
  // PART 1: UART Communication Monitoring
  checkUartCommunication();
  
  // PART 2: Direct Pin State Monitoring
  checkPinStates();
}

void checkUartCommunication() {
  // Check for data on hardware Serial1 (UART)
  if (Serial1.available()) {
    byte c = Serial1.read();
    lastByteTime = millis();
    uartActive = true;
    
    // Flash LED briefly to indicate reception
    digitalWrite(LED_BUILTIN, HIGH);
    
    // Echo back data to DE1-SoC
    Serial1.write(c);
    
    // Print received byte in multiple formats
    Serial.print("UART Received: 0x");
    if (c < 16) Serial.print("0"); // Pad with leading zero
    Serial.print(c, HEX);
    Serial.print(" (binary: ");
    
    // Print binary representation
    for (int i = 7; i >= 0; i--) {
      Serial.print((c >> i) & 1);
    }
    
    Serial.print(", decimal: ");
    Serial.print(c);
    Serial.print(", char: '");
    
    // Only print the character if it's printable
    if (c >= 32 && c <= 126) {
      Serial.print((char)c);
    } else {
      // For non-printable characters, show a description
      switch(c) {
        case 0: Serial.print("NULL"); break;
        case 10: Serial.print("LF"); break;
        case 13: Serial.print("CR"); break;
        default: Serial.print("non-printable"); break;
      }
    }
    Serial.println("')");
    
    digitalWrite(LED_BUILTIN, LOW);
  }
  
  // If it's been 5 seconds since last UART activity, report it
  if (uartActive && (millis() - lastByteTime > 5000)) {
    Serial.println("UART: No activity for 5 seconds");
    uartActive = false;
  }
}

void checkPinStates() {
  // Read current pin state directly
  int currentPinState = digitalRead(RX_PIN);
  unsigned long now = millis();
  
  // If state changed, analyze it
  if (currentPinState != lastPinState) {
    unsigned long duration = now - lastChangeTime;
    lastChangeTime = now;
    
    // Toggle LED to show state change detection
    digitalWrite(LED_BUILTIN, currentPinState == LOW);
    
    // Print state change information
    Serial.print("Pin state changed to ");
    Serial.print(currentPinState ? "HIGH" : "LOW");
    Serial.print(" after ");
    Serial.print(duration);
    Serial.println("ms");
    
    // Check for Manchester encoding
    checkManchesterEncoding(currentPinState, duration, now);
    
    // Detect pattern 1: 5 pulses with 100ms LOW, 300ms HIGH
    detectShortLongPattern(currentPinState, duration);
    
    // Detect pattern 2: 1 second LOW, 1 second HIGH
    detectLongPattern(currentPinState, duration);
    
    // Update state
    lastPinState = currentPinState;
  }
}

void checkManchesterEncoding(int currentState, unsigned long duration, unsigned long now) {
  // Start sequence detection (3 pulses of ~100ms each)
  if (!manchesterMode && !inStartSequence) {
    if (duration >= 80 && duration <= 120) {
      inStartSequence = true;
      startSequenceCount = 1;
      Serial.println("Possible Manchester start sequence detected");
    }
  } 
  else if (!manchesterMode && inStartSequence) {
    if (duration >= 80 && duration <= 120) {
      startSequenceCount++;
      Serial.print("Manchester start sequence pulse ");
      Serial.println(startSequenceCount);
      
      if (startSequenceCount >= 6) { // 3 complete pulses (3 rises + 3 falls = 6 edges)
        manchesterMode = true;
        inStartSequence = false;
        bitPosition = 7; // MSB first
        currentByte = 0;
        Serial.println("Manchester encoding detected - decoding started");
      }
    } else {
      // Reset if timing is wrong
      inStartSequence = false;
      Serial.println("Manchester start sequence reset - incorrect timing");
    }
  }
  // End sequence detection (1 long pulse ~150ms)
  else if (manchesterMode && !inEndSequence) {
    if (duration >= 130 && duration <= 170) {
      inEndSequence = true;
      Serial.println("Possible Manchester end sequence detected");
    }
    // Bit decoding
    else if (duration >= 40 && duration <= 60) {
      // This is a bit transition - standard Manchester encoding:
      // HIGH-to-LOW = 1, LOW-to-HIGH = 0 (MSB first)
      if (bitPosition >= 0) {
        if (currentState == LOW) { // Just transitioned from HIGH to LOW
          // This is a '1' bit
          currentByte |= (1 << bitPosition);
          Serial.print("1");
        } else { // Just transitioned from LOW to HIGH
          // This is a '0' bit
          Serial.print("0");
          // We don't need to set the bit to 0 as it's already 0
        }
        
        bitPosition--;
        
        // If we've decoded all 8 bits
        if (bitPosition < 0) {
          Serial.print(" = 0x");
          if (currentByte < 16) Serial.print("0");
          Serial.print(currentByte, HEX);
          Serial.print(" ('");
          if (currentByte >= 32 && currentByte <= 126) {
            Serial.print((char)currentByte);
          } else {
            Serial.print("non-printable");
          }
          Serial.println("')");
          
          // Reset for next byte
          bitPosition = 7;
          currentByte = 0;
        }
      }
    } else {
      // Unexpected timing - might be between bytes or an error
      if (bitPosition != 7) {
        Serial.println("\nUnexpected timing during Manchester decoding");
      }
    }
  }
  else if (manchesterMode && inEndSequence) {
    // After end sequence, wait for the second edge
    if (duration >= 130 && duration <= 170) {
      Serial.println("Manchester end sequence complete");
      manchesterMode = false;
      inEndSequence = false;
    } else {
      // Not a proper end sequence - continue processing
      inEndSequence = false;
    }
  }
}

void detectShortLongPattern(int currentState, unsigned long duration) {
  // Looking for alternating 100ms LOW, 300ms HIGH pattern
  if (currentState == HIGH && duration >= 80 && duration <= 120) {
    // This was a ~100ms LOW pulse followed by going HIGH
    pulseCount++;
    Serial.print("Pattern 1 - Detected short LOW pulse #");
    Serial.println(pulseCount);
    
    if (pulseCount >= 5) {
      Serial.println("PATTERN DETECTED: 5 short-long pulses (100ms/300ms)");
      pulseCount = 0;
    }
  } 
  else if (currentState == LOW && duration >= 280 && duration <= 320) {
    // This was a ~300ms HIGH pulse followed by going LOW
    // We don't increment the counter here, just validate the pattern
    Serial.println("Pattern 1 - Detected long HIGH segment");
  }
  else if (duration > 150 && duration < 280) {
    // Pulse duration outside expected ranges, reset counter
    pulseCount = 0;
  }
}

void detectLongPattern(int currentState, unsigned long duration) {
  // Looking for ~1000ms LOW followed by ~1000ms HIGH
  if (currentState == HIGH && duration >= 900 && duration <= 1100) {
    // This was a ~1s LOW pulse followed by going HIGH
    Serial.println("PATTERN DETECTED: Long LOW pulse (~1s)");
  }
  else if (currentState == LOW && duration >= 900 && duration <= 1100) {
    // This was a ~1s HIGH pulse followed by going LOW
    Serial.println("PATTERN DETECTED: Long HIGH pulse (~1s)");
  }
}