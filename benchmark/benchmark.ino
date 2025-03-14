/*
 * IR Sensor Benchmark Script for Adafruit Feather 32u4
 * 
 * This script reads values from a single IR sensor and outputs them to the Serial Monitor
 * for tuning and testing purposes.
 * 
 * Connections:
 * - Red wire: Connect to VBUS (5V) or 3.3V on Feather board
 * - Yellow wire: Connect to any GND pin on Feather board
 * - Orange wire: Connect to analog input pin A0
 */

// Define the pin to which the IR sensor's orange wire is connected
#define IR_SENSOR_PIN A0

// Threshold for determining black vs white/reflective surface
// You can adjust this value based on your testing results
#define THRESHOLD 500

// LED pin for visual feedback
#define LED_PIN 13  // Built-in LED on the Feather 32u4 (labeled as D13)

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for serial port to connect (needed for native USB devices like the Feather)
  }
  
  // Set up sensor pin as input
  pinMode(IR_SENSOR_PIN, INPUT);
  
  // Set up LED pin as output (for visual feedback)
  pinMode(LED_PIN, OUTPUT);
  
  Serial.println("IR Sensor Benchmark for Feather 32u4");
  Serial.println("------------------------------------");
  Serial.println("Raw values will be displayed below.");
  Serial.println("Higher values typically indicate darker/less reflective surfaces.");
  Serial.println("Lower values typically indicate lighter/more reflective surfaces.");
  Serial.println();
  Serial.println("Current threshold set to: " + String(THRESHOLD));
  Serial.println("Reading...");
}

void loop() {
  // Read the raw analog value from the IR sensor
  int sensorValue = analogRead(IR_SENSOR_PIN);
  
  // Determine if the surface is "black" (less reflective) based on threshold
  bool isBlack = (sensorValue > THRESHOLD);
  
  // Track min/max values
  static int minValue = 1023;
  static int maxValue = 0;
  
  if (sensorValue < minValue) minValue = sensorValue;
  if (sensorValue > maxValue) maxValue = sensorValue;
  
  // Print header every 20 readings for clarity
  static int readingCount = 0;
  if (readingCount % 20 == 0) {
    Serial.println("\n--- IR SENSOR READINGS ---");
    Serial.println("Raw Value | Detection | Min | Max");
    Serial.println("--------------------------");
  }
  readingCount++;
  
  // Print formatted output
  Serial.print(sensorValue);
  Serial.print("\t| ");
  Serial.print(isBlack ? "BLACK" : "WHITE");
  Serial.print("\t| ");
  Serial.print(minValue);
  Serial.print(" | ");
  Serial.println(maxValue);
  
  // Visual feedback using LED
  digitalWrite(LED_PIN, isBlack);
  
  // Small delay to make the output readable
  delay(200);
}