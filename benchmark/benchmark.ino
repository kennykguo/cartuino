void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial.println("Echo test starting - will display hex values");
}

void loop() {
  if (Serial1.available()) {
    byte c = Serial1.read();
    Serial1.write(c);  // Echo back
    
    // Print the received byte in multiple formats
    Serial.print("Received byte: 0x");
    if (c < 16) Serial.print("0"); // Pad with leading zero
    Serial.print(c, HEX);
    Serial.print(" (decimal: ");
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
  }
}