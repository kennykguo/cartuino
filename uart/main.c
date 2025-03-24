/*
 * IMPROVED BIT-BANG UART PROTOCOL
 * 
 * This implementation uses more generous timing and simpler approach.
 */

/************** ARDUINO CODE **************/

// Arduino version - copy to a new sketch
#define RX_PIN 0 // D0 for receiving from DE1-SoC
#define TX_PIN 1 // D1 for transmitting to DE1-SoC

// Matching the DE1-SoC's 600 baud rate
#define BAUD_RATE 600
#define BIT_DELAY (1000000 / BAUD_RATE)

void setup() {
  Serial.begin(115200);
  pinMode(RX_PIN, INPUT);
  pinMode(TX_PIN, OUTPUT);
  digitalWrite(TX_PIN, HIGH); // Idle state for UART
  
  delay(1000);
  Serial.println("Improved UART Protocol Ready");
  Serial.println("Using bit-bang UART at 600 baud");
}

void loop() {
  static unsigned long lastSendTime = 0;
  
  // Check for incoming data on RX pin
  if (digitalRead(RX_PIN) == LOW) {
    // Start bit detected, wait a bit to center in the bit
    delayMicroseconds(BIT_DELAY / 2);
    
    // Read the byte
    byte receivedByte = receiveChar();
    
    // Print the received byte
    Serial.print("Received: 0x");
    if (receivedByte < 16) Serial.print("0");
    Serial.print(receivedByte, HEX);
    Serial.print(" '");
    if (receivedByte >= 32 && receivedByte <= 126) {
      Serial.print((char)receivedByte);
    } else {
      Serial.print(".");
    }
    Serial.println("'");
    
    // Echo it back
    delay(10); // Small delay before responding
    sendChar(receivedByte);
  }
  
  // Every 5 seconds, send a test message
  if (millis() - lastSendTime > 5000) {
    Serial.println("Sending test message...");
    sendString("ARDUINO_TEST");
    lastSendTime = millis();
  }
}

// Send a single character using software UART
void sendChar(byte c) {
  // Start bit (LOW)
  digitalWrite(TX_PIN, LOW);
  delayMicroseconds(BIT_DELAY);
  
  // Send 8 data bits (LSB first)
  for (int i = 0; i < 8; i++) {
    digitalWrite(TX_PIN, (c & (1 << i)) ? HIGH : LOW);
    delayMicroseconds(BIT_DELAY);
  }
  
  // Stop bit (HIGH) - extra long for reliability
  digitalWrite(TX_PIN, HIGH);
  delayMicroseconds(BIT_DELAY * 2);
}

// Send a string one character at a time
void sendString(const char* str) {
  while (*str) {
    sendChar(*str++);
    delay(5); // Small inter-character delay
  }
}

// Receive a single character using software UART
byte receiveChar() {
  byte val = 0;
  
  // Start bit already detected before calling this function
  
  // Read 8 data bits (LSB first)
  for (int i = 0; i < 8; i++) {
    delayMicroseconds(BIT_DELAY);
    if (digitalRead(RX_PIN)) {
      val |= (1 << i);
    }
  }
  
  // Wait for stop bit
  delayMicroseconds(BIT_DELAY);
  
  // Add extra delay to ensure we're clear of the stop bit
  delayMicroseconds(BIT_DELAY);
  
  return val;
}

/************** DE1-SoC CODE **************/

/*
// DE1-SoC version - integrate into your program

// UART settings
#define UART_RX_BIT 0x00000001  // D0 for receiving from Arduino
#define UART_TX_BIT 0x00000002  // D1 for transmitting to Arduino
#define UART_BAUD_RATE 600      // Very slow baud for reliability

// Calculate bit delay using system clock
#define BIT_DELAY (CLOCK_RATE / UART_BAUD_RATE)

// Buffer for received data
char rx_buffer[64];
int rx_buffer_pos = 0;

// Initialize UART
void init_uart() {
    // Set TX as output, RX as input
    *(JP1_ptr + 1) = UART_TX_BIT;  // Direction register
    
    // Set TX HIGH (idle state)
    *(JP1_ptr) |= UART_TX_BIT;
    
    printf("UART initialized at %d baud\n", UART_BAUD_RATE);
    printf("TX (D1): %s, RX (D0): %s\n",
           (*(JP1_ptr) & UART_TX_BIT) ? "HIGH" : "LOW",
           (*(JP1_ptr) & UART_RX_BIT) ? "HIGH" : "LOW");
}

// Send a character
void uart_tx_char(char c) {
    printf("Sending char: '%c' (0x%02X)\n", (c >= 32 && c <= 126) ? c : '.', c & 0xFF);
    
    // Start bit (LOW)
    *(JP1_ptr) &= ~UART_TX_BIT;
    bit_delay();
    
    // 8 data bits (LSB first)
    for (int i = 0; i < 8; i++) {
        if (c & (1 << i)) {
            *(JP1_ptr) |= UART_TX_BIT;   // HIGH
        } else {
            *(JP1_ptr) &= ~UART_TX_BIT;  // LOW
        }
        bit_delay();
    }
    
    // Stop bit (HIGH) - extra long for reliability
    *(JP1_ptr) |= UART_TX_BIT;
    bit_delay();
    bit_delay();  // Double stop bit
}

// Send a string
void uart_tx_string(const char* str) {
    printf("Sending string: \"%s\"\n", str);
    
    while (*str) {
        uart_tx_char(*str++);
        delay_ms(5);  // Small delay between chars
    }
    
    printf("String transmission complete\n");
}

// Read a character with timeout
int uart_rx_char(int timeout_ms) {
    int i, rx_data = 0;
    unsigned long start_time = *(TIMER_ptr);
    
    // Wait for start bit (RX going LOW)
    while ((*(JP1_ptr) & UART_RX_BIT)) {
        if (timeout_ms > 0 && 
            (*(TIMER_ptr) - start_time) > (timeout_ms * (CLOCK_RATE / 1000))) {
            return -1;  // Timeout
        }
    }
    
    printf("Start bit detected\n");
    
    // Wait half a bit time to sample in the middle of the bit
    delay_us(BIT_DELAY / 2);
    
    // Read 8 data bits (LSB first)
    for (i = 0; i < 8; i++) {
        delay_us(BIT_DELAY);
        if (*(JP1_ptr) & UART_RX_BIT) {
            rx_data |= (1 << i);
        }
    }
    
    // Wait for stop bit
    delay_us(BIT_DELAY);
    
    printf("Received char: '%c' (0x%02X)\n", 
          (rx_data >= 32 && rx_data <= 126) ? (char)rx_data : '.', rx_data);
    
    return rx_data;
}

// Helper for bit timing
void bit_delay() {
    volatile int i;
    for (i = 0; i < BIT_DELAY; i++);
}

// Helper for microsecond delay
void delay_us(int us) {
    volatile int i;
    for (i = 0; i < us * (CLOCK_RATE / 1000000); i++);
}
*/