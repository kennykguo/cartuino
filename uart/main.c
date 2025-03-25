#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "sys/alt_stdio.h"
#include "system.h"
#include "altera_avalon_pio_regs.h"

// Define pins (adjust these based on your actual hardware configuration)
#define TX_BASE PIO_0_BASE
#define RX_BASE PIO_1_BASE
#define KEYS_BASE PIO_2_BASE

#define TX_PIN_MASK 0x01
#define RX_PIN_MASK 0x01
#define KEY0_MASK 0x01

// Communication parameters
#define BIT_RATE 500  // 500 bps = 2ms per bit
#define BIT_DELAY_US 2000  // Microseconds per bit

// Test message
const char* TEST_MESSAGE = "Hello from DE1-SOC!";

// Receive buffer
char receivedMessage[100];
int msgIndex = 0;

// Function for more precise microsecond delay
void delay_us(int us) {
    // For a 100MHz clock, each cycle is 10ns, so 100 cycles = 1us
    // This is a simplified calculation and may need calibration
    volatile int cycles = us * 100;
    while (cycles--) {
        asm("nop");
    }
}

// Send a single byte
void sendByte(char c) {
    // Start bit (LOW)
    IOWR_ALTERA_AVALON_PIO_DATA(TX_BASE, 0);
    delay_us(BIT_DELAY_US);
    
    // Data bits (LSB first)
    for (int bit = 0; bit < 8; bit++) {
        int bitValue = (c >> bit) & 1;
        IOWR_ALTERA_AVALON_PIO_DATA(TX_BASE, bitValue ? TX_PIN_MASK : 0);
        delay_us(BIT_DELAY_US);
    }
    
    // Stop bit (HIGH)
    IOWR_ALTERA_AVALON_PIO_DATA(TX_BASE, TX_PIN_MASK);
    delay_us(BIT_DELAY_US);
    
    // Extra delay between bytes for reliability
    delay_us(BIT_DELAY_US);
}

// Send a complete message
void sendMessage(const char* message) {
    for (int i = 0; message[i] != '\0'; i++) {
        sendByte(message[i]);
    }
    // Send newline to mark end of message
    sendByte('\n');
}

// Receive a single byte
char receiveByte() {
    char receivedChar = 0;
    
    // We've already detected the start bit, now wait until middle of bit
    delay_us(BIT_DELAY_US / 2);
    
    // Wait until middle of first data bit
    delay_us(BIT_DELAY_US);
    
    // Read 8 data bits (LSB first)
    for (int bit = 0; bit < 8; bit++) {
        int bitValue = IORD_ALTERA_AVALON_PIO_DATA(RX_BASE) & RX_PIN_MASK;
        if (bitValue) {
            receivedChar |= (1 << bit);
        }
        delay_us(BIT_DELAY_US);
    }
    
    // Wait for stop bit to pass
    delay_us(BIT_DELAY_US);
    
    // Add to message buffer
    receivedMessage[msgIndex++] = receivedChar;
    
    // Check if we've received a complete message (newline)
    if (receivedChar == '\n' || msgIndex >= sizeof(receivedMessage) - 1) {
        receivedMessage[msgIndex] = '\0';
        printf("Received message: %s\n", receivedMessage);
        msgIndex = 0;
    }
    
    return receivedChar;
}

int main() {
    alt_putstr("NIOS V ready\n");
    
    // Initialize TX pin to idle state (HIGH)
    IOWR_ALTERA_AVALON_PIO_DATA(TX_BASE, TX_PIN_MASK);
    
    int prevKey0 = IORD_ALTERA_AVALON_PIO_DATA(KEYS_BASE) & KEY0_MASK;
    
    while (1) {
        // Check if KEY0 is pressed (active low)
        int key0 = IORD_ALTERA_AVALON_PIO_DATA(KEYS_BASE) & KEY0_MASK;
        
        if (!key0 && prevKey0) {
            // KEY0 was just pressed
            printf("Sending message\n");
            sendMessage(TEST_MESSAGE);
        }
        
        prevKey0 = key0;
        
        // Check for incoming data (start bit)
        if (!(IORD_ALTERA_AVALON_PIO_DATA(RX_BASE) & RX_PIN_MASK)) {
            receiveByte();
        }
        
        // Small delay to prevent tight polling
        usleep(100);
    }
    
    return 0;
}