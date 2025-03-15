// Arduino UART Benchmark for DE1-SoC Connection
// Companion to the DE1-SoC FPGA benchmark code

// Feather 32u4 has hardware UART on pins D0 (RX) and D1 (TX)
// No need to redefine pins since we'll use the built-in Serial1
// DE1 SOC JP1 D1 TO ARDUINO PIN 0
// DE1 SOC JP1 D0 TO ARDUINO PIN 1


// Benchmark parameters
#define PACKET_SIZE 32  // Size of each test packet in bytes
#define BAUD_RATE 9600

// We'll use the hardware Serial1 for communication with DE1-SoC
// Serial will be used for debugging with the computer

// Buffer for incoming/outgoing data
byte dataBuffer[PACKET_SIZE];

// Test metrics
unsigned long packetCount = 0;
unsigned long errorCount = 0;
unsigned long startTime = 0;
unsigned long endTime = 0;

// Test mode
enum TestMode {
  STANDBY,
  LATENCY_TEST,
  THROUGHPUT_TEST,
  RELIABILITY_TEST
};

TestMode currentMode = STANDBY;

void setup() {
  // Set up serial communication for debugging with computer
  Serial.begin(BAUD_RATE);
  Serial.println("Feather 32u4 UART Benchmark Starting...");
  
  // Set up hardware serial for DE1-SoC communication
  Serial1.begin(BAUD_RATE);
  
  // Configure LED pin for status indication
  pinMode(LED_BUILTIN, OUTPUT);
  
  // No need to manually configure UART pins when using hardware Serial1
  
  Serial.println("Feather 32u4 UART Benchmark Ready");
}

void loop() {
  // Check for commands from DE1-SoC
  if (Serial1.available() > 0) {
    char cmd = Serial1.read();
    
    // If it's the start of a command
    if (cmd == 'B') {
      // Read the command type
      while (Serial1.available() < 2) {
        // Wait for rest of command
        delay(1);
      }
      
      // Skip comma
      Serial1.read();
      
      // Get test type
      char testType = Serial1.read();
      
      // Skip newline if present
      if (Serial1.peek() == '\n') {
        Serial1.read();
      }
      
      // Reset test metrics
      packetCount = 0;
      errorCount = 0;
      
      // Set test mode based on command
      switch (testType) {
        case 'L':
          currentMode = LATENCY_TEST;
          Serial.println("Starting latency test");
          break;
        case 'T':
          currentMode = THROUGHPUT_TEST;
          Serial.println("Starting throughput test");
          break;
        case 'R':
          currentMode = RELIABILITY_TEST;
          Serial.println("Starting reliability test");
          break;
        default:
          Serial.println("Unknown test type");
          currentMode = STANDBY;
          break;
      }
      
      // Send ready signal
      Serial1.write('R');
      
      // Record start time
      startTime = micros();
    }
    // Process based on current test mode
    else if (currentMode == LATENCY_TEST) {
      // For latency test, simply echo back the received byte
      Serial1.write(cmd);
      packetCount++;
      
      // Toggle LED for visual indication
      digitalWrite(LED_BUILTIN, packetCount % 2);
    }
  }
  
  // Handle throughput test - we just receive, no echo
  if (currentMode == THROUGHPUT_TEST) {
    if (Serial1.available() > 0) {
      // Read as much data as available
      while (Serial1.available() > 0) {
        byte data = Serial1.read();
        packetCount++;
      }
      
      // Toggle LED every 256 bytes
      if ((packetCount & 0xFF) == 0) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      }
    }
  }
  
  // Handle reliability test
  if (currentMode == RELIABILITY_TEST) {
    // Read a full packet and echo it back
    if (Serial1.available() >= PACKET_SIZE) {
      // Read the packet
      for (int i = 0; i < PACKET_SIZE; i++) {
        dataBuffer[i] = Serial1.read();
      }
      
      // Echo it back
      for (int i = 0; i < PACKET_SIZE; i++) {
        Serial1.write(dataBuffer[i]);
      }
      
      packetCount++;
      
      // Toggle LED on each packet
      digitalWrite(LED_BUILTIN, packetCount % 2);
    }
  }
  
  // Extended reliability test - handle partial packets
  if (currentMode == RELIABILITY_TEST && Serial1.available() > 0 && Serial1.available() < PACKET_SIZE) {
    // Wait a bit for more data
    delay(10);
    
    // If we still don't have a full packet, read what we can
    if (Serial1.available() < PACKET_SIZE) {
      int available = Serial1.available();
      
      // Read the partial packet
      for (int i = 0; i < available; i++) {
        dataBuffer[i] = Serial1.read();
      }
      
      // Echo back what we got
      for (int i = 0; i < available; i++) {
        Serial1.write(dataBuffer[i]);
      }
      
      // Count as an error since it's a partial packet
      errorCount++;
    }
  }
  
  // Auto-reset to standby if no activity for 5 seconds
  if (currentMode != STANDBY && (millis() - startTime > 5000)) {
    if (Serial1.available() == 0) {
      currentMode = STANDBY;
      
      // Print test summary to debug serial
      endTime = micros();
      Serial.println("Test completed or timed out");
      Serial.print("Packets processed: ");
      Serial.println(packetCount);
      Serial.print("Errors: ");
      Serial.println(errorCount);
      Serial.print("Time elapsed (us): ");
      Serial.println(endTime - startTime);
      
      if (packetCount > 0) {
        float bytesPerSecond = (packetCount * 1000000.0) / (endTime - startTime);
        Serial.print("Throughput (bytes/sec): ");
        Serial.println(bytesPerSecond);
      }
      
      // Turn off LED
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
}

// Extended test functions for detailed benchmarking

// Function to calculate checksum of a packet
byte calculateChecksum(byte *data, int length) {
  byte checksum = 0;
  for (int i = 0; i < length; i++) {
    checksum ^= data[i]; // XOR checksum
  }
  return checksum;
}

// Function to measure timing jitter
void measureJitter() {
  const int samples = 100;
  unsigned long times[samples];
  unsigned long lastTime = 0;
  
  // Collect timing samples of incoming bytes
  for (int i = 0; i < samples; i++) {
    // Wait for a byte
    while (Serial1.available() == 0);
    
    // Record time and read the byte
    unsigned long now = micros();
    Serial1.read();
    
    // Calculate inter-byte delay
    if (i > 0) {
      times[i-1] = now - lastTime;
    }
    
    lastTime = now;
    
    // Send acknowledgment
    Serial1.write('A');
  }
  
  // Calculate jitter statistics
  unsigned long sum = 0;
  unsigned long min_time = 0xFFFFFFFF;
  unsigned long max_time = 0;
  
  for (int i = 0; i < samples-1; i++) {
    sum += times[i];
    if (times[i] < min_time) min_time = times[i];
    if (times[i] > max_time) max_time = times[i];
  }
  
  unsigned long avg_time = sum / (samples - 1);
  unsigned long jitter = max_time - min_time;
  
  // Report statistics to debug serial
  Serial.print("Average inter-byte time (us): ");
  Serial.println(avg_time);
  Serial.print("Min time (us): ");
  Serial.println(min_time);
  Serial.print("Max time (us): ");
  Serial.println(max_time);
  Serial.print("Jitter (us): ");
  Serial.println(jitter);
}

//