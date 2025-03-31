// Minimal Quadrature Encoder for Feather 32u4 (600PPR)
volatile int32_t encoderCount = 0;
volatile int8_t encoderDir = 0; // 1=CW, -1=CCW
uint8_t prevState = 0;

void setup() {
  Serial.begin(115200);
  pinMode(10, INPUT); // Green (A) - changed from INPUT_PULLUP
  pinMode(11, INPUT); // White (B) - changed from INPUT_PULLUP
  
  attachInterrupt(digitalPinToInterrupt(2), readEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), readEncoder, CHANGE);
}
void loop() {
  static int32_t lastCount = 0;
  
  // Fast poll without delays
  if(millis() % 50 == 0) { // Update every 50ms
    noInterrupts();
    int32_t currentCount = encoderCount;
    int8_t currentDir = encoderDir;
    interrupts();

    // Calculate angle (0-360°)
    float angle = fmod(currentCount * 0.15, 360.0);
    if(angle < 0) angle += 360.0;

    // Determine direction from count change
    String dir = (currentCount > lastCount) ? "CW" : 
                (currentCount < lastCount) ? "CCW" : "STOP";
    
    Serial.print("Count: ");
    Serial.print(currentCount);
    Serial.print("\tAngle: ");
    Serial.print(angle, 1);
    Serial.print("°\t");
    Serial.println(dir);
    
    lastCount = currentCount;
  }
}

// Optimized ISR with state machine
void readEncoder() {
  static uint32_t lastISR = 0;
  uint32_t now = micros();
  
  // Hardware debounce (100μs)
  if(now - lastISR < 100) return;
  lastISR = now;

  uint8_t currState = (digitalRead(2) << 1) | digitalRead(3);
  
  // State transition matrix for quadrature decoding
  static const int8_t transitions[16] = {
     0, -1,  1,  0, 
     1,  0,  0, -1, 
    -1,  0,  0,  1, 
     0,  1, -1,  0
  };

  int8_t delta = transitions[(prevState << 2) | currState];
  
  if(delta) {
    encoderCount += delta;
    encoderDir = delta > 0 ? 1 : -1;
    prevState = currState;
  }
}