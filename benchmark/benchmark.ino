void setup() {
  Serial.begin(115200);
  pinMode(0, OUTPUT); // D0 as output
  pinMode(LED_BUILTIN, OUTPUT);
  
  digitalWrite(0, LOW);
  digitalWrite(LED_BUILTIN, LOW);
  
  delay(1000);
  Serial.println("Pure Signal Test Started");
  Serial.println("D0 will toggle HIGH/LOW every second");
}

void loop() {
  // Toggle D0 HIGH
  digitalWrite(0, HIGH);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("D0: HIGH");
  delay(1000);
  
  // Toggle D0 LOW
  digitalWrite(0, LOW);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("D0: LOW");
  delay(1000);
}