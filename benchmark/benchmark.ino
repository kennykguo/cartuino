// Arduino version - copy this to a new Arduino sketch
const int DATA_PIN_OUT = 3;  // Use D3 for output to DE1-SoC
const int DATA_PIN_IN = 0;   // Use D0 for input from DE1-SoC

void setup() {
  Serial.begin(115200);
  pinMode(DATA_PIN_OUT, OUTPUT);
  pinMode(DATA_PIN_IN, INPUT);
  digitalWrite(DATA_PIN_OUT, LOW);
  
  delay(1000);
  Serial.println("Simplified Communication Protocol Ready");
  
  // Print initial pin state
  Serial.print("Initial input pin state: ");
  Serial.println(digitalRead(DATA_PIN_IN) ? "HIGH" : "LOW");
}

void loop() {
  static unsigned long lastSendTime = 0;
  static unsigned long lastPulseTime = 0;
  static int pulseCount = 0;
  static int lastPinState = -1;
  
  // Check for incoming signals from DE1-SoC
  int pinState = digitalRead(DATA_PIN_IN);
  if (pinState != lastPinState) {
    Serial.print("Pin state changed to: ");
    Serial.println(pinState ? "HIGH" : "LOW");
    lastPinState = pinState;
    
    // Toggle built-in LED based on pin state
    digitalWrite(LED_BUILTIN, pinState);
    
    // Count transitions to detect messages
    if (pinState == HIGH) {
      pulseCount++;
      Serial.print("Pulse count: ");
      Serial.println(pulseCount);
      
      // If we receive 5 pulses, consider it a "message"
      if (pulseCount >= 5) {
        Serial.println("MESSAGE RECEIVED FROM DE1-SOC!");
        pulseCount = 0;
        
        // Send a response
        sendPulsePattern();
      }
    }
    
    lastPulseTime = millis();
  }
  
  // Auto-send pulse pattern every 5 seconds
  if (millis() - lastSendTime > 5000) {
    Serial.println("Auto-sending pulse pattern");
    sendPulsePattern();
    lastSendTime = millis();
  }
  
  // Reset pulse counter if too much time passes between pulses
  if (pulseCount > 0 && millis() - lastPulseTime > 1000) {
    Serial.println("Pulse sequence timed out, resetting counter");
    pulseCount = 0;
  }
}

// Send a simple pattern of pulses
void sendPulsePattern() {
  // Send 10 pulses with clear timing
  for (int i = 0; i < 10; i++) {
    digitalWrite(DATA_PIN_OUT, HIGH);
    Serial.println("Sent: HIGH");
    delay(100);  // 100ms HIGH
    
    digitalWrite(DATA_PIN_OUT, LOW);
    Serial.println("Sent: LOW");
    delay(100);  // 100ms LOW
  }
  
  // End with LOW state
  digitalWrite(DATA_PIN_OUT, LOW);
  Serial.println("Pulse pattern complete");
}

