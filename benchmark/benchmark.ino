// Simple UART Test for Feather 32u4 and DE1-SoC
// This is a minimal test to verify UART communication

#define BAUD_RATE 115200

// Test mode
enum TestMode {
  IDLE,
  TEST_ACTIVE
};

TestMode currentMode = IDLE;
unsigned long lastBlinkTime = 0;
unsigned long lastHeartbeatTime = 0;
unsigned long messageCount = 0;
unsigned long heartbeatCount = 0;

void setup() {
  // Set up serial communication for debugging with computer
  Serial.begin(BAUD_RATE);
  
  // Wait for serial connection to establish
  delay(3000);
  
  Serial.println("==================================");
  Serial.println("Feather 32u4 Simple UART Test");
  Serial.println("==================================");
  
  // Set up hardware serial for DE1-SoC communication
  Serial1.begin(BAUD_RATE);
  
  // Configure LED pin for status indication
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Blink LED to indicate setup is complete
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  }
  
  Serial.println("Setup complete");
  Serial.println("Waiting for DE1-SoC communication...");
  Serial.println("==================================");
}

void loop() {
  // Send heartbeat message to Serial monitor
  unsigned long currentTime = millis();
  if (currentTime - lastHeartbeatTime >= 5000) {  // Every 5 seconds
    heartbeatCount++;
    Serial.print("Heartbeat #");
    Serial.print(heartbeatCount);
    Serial.print(" - Messages received: ");
    Serial.println(messageCount);
    lastHeartbeatTime = currentTime;
    
    // Send a test message to DE1-SoC
    Serial.println("Sending test message to DE1-SoC...");
    Serial1.print("FEATHER_MSG_");
    Serial1.println(heartbeatCount);
  }
  
  // Check for incoming data from DE1-SoC
  if (Serial1.available() > 0) {
    // Turn on LED to indicate received data
    digitalWrite(LED_BUILTIN, HIGH);
    lastBlinkTime = currentTime;
    currentMode = TEST_ACTIVE;
    
    // Read all available data
    String message = "";
    while (Serial1.available() > 0) {
      char c = Serial1.read();
      message += c;
    }
    
    // Print received data to Serial monitor
    Serial.print("Received from DE1-SoC: '");
    Serial.print(message);
    Serial.println("'");
    
    // Echo back what we received
    Serial1.print("ECHO:");
    Serial1.print(message);
    
    // Increment message counter
    messageCount++;
  }
  
  // Turn off LED after a delay
  if (currentMode == TEST_ACTIVE && currentTime - lastBlinkTime >= 100) {
    digitalWrite(LED_BUILTIN, LOW);
    currentMode = IDLE;
  }
}