/*
 * Arduino Dual-Mode UART and Pulse Detection Script
 * For communication with DE1-SoC FPGA
 * 
 * This script detects both standard UART communication
 * and direct pulse signaling as an alternative
 */

// Pin definitions
#define RX_PIN 0  // Hardware UART RX pin (D0)
#define TX_PIN 1  // Hardware UART TX pin (D1)

// Pulse detection variables
unsigned long lastChangeTime = 0;
int lastPinState = HIGH;
int pulseCount = 0;
bool patternDetected = false;

// UART state
bool uartActive = false;
unsigned long lastByteTime = 0;

void setup() {
  // Setup USB serial for debugging output
  Serial.begin(9600);
  
  // Setup hardware UART for DE1-SoC communication
  Serial1.begin(600);  // Very slow baud rate for reliability
  
  // Configure direct pin monitoring in addition to Serial1
  pinMode(RX_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Initial LED state
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  Serial1.setTimeout(100);  // 100ms timeout for UART operations
  Serial.println("\n\n");
  Serial.println("==============================================");
  Serial.println("Arduino DE1-SoC UART and Pulse Detection Ready");
  Serial.println("==============================================");
  Serial.println("Monitoring for:");
  Serial.println("1. UART data at 1200 baud");
  Serial.println("2. Pulse patterns (100/300ms and 1s)");
  Serial.println("==============================================");
}

void loop() {
  // PART 1: UART Communication Monitoring
  checkUartCommunication();
  
  // PART 2: Direct Pulse Pattern Detection
  checkPulsePatterns();
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

void checkPulsePatterns() {
  // Read current pin state directly
  int currentPinState = digitalRead(RX_PIN);
  unsigned long now = millis();
  
  // If state changed, analyze it
  if (currentPinState != lastPinState) {
    unsigned long duration = now - lastChangeTime;
    lastChangeTime = now;
    
    // Toggle LED to show state change detection
    if (currentPinState == LOW) {
      digitalWrite(LED_BUILTIN, HIGH);
    } else {
      digitalWrite(LED_BUILTIN, LOW);
    }
    
    // Print state change information
    Serial.print("Pin state changed to ");
    Serial.print(currentPinState ? "HIGH" : "LOW");
    Serial.print(" after ");
    Serial.print(duration);
    Serial.println("ms");
    
    // Detect pattern 1: 5 pulses with 100ms LOW, 300ms HIGH
    detectShortLongPattern(currentPinState, duration);
    
    // Detect pattern 2: 1 second LOW, 1 second HIGH
    detectLongPattern(currentPinState, duration);
    
    // Update state
    lastPinState = currentPinState;
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
  else if (duration > 150 && duration < 400) {
    // Pulse duration outside expected range, reset counter
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