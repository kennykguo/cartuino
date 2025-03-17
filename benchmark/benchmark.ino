void setup() {
  Serial.begin(9600);          // USB serial for debugging
  Serial1.begin(1200);         // Match DE1-SoC baud rate exactly
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println("DE1-SoC UART Communication Test");
}

void loop() {
  if (Serial1.available()) {
    byte c = Serial1.read();
    Serial1.write(c);  // Echo back
    
    // Flash LED to visually indicate reception
    digitalWrite(LED_BUILTIN, HIGH);
    
    // Print the received byte in multiple formats
    Serial.print("Received byte: 0x");
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
    
    // Only try to print the character if it's printable
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
}