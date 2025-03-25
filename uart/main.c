// DE1-SoC Simplified Communication Protocol
// Uses a simple bit-banging approach with generous timing

#include <stdio.h>
#include <string.h>
#include "address_map.h"

// JP1 Pin definitions
#define DATA_PIN_BIT 0x00000001  // Bit 0 (D0) for data line
#define CLOCK_PIN_BIT 0x00000002 // Bit 1 (D1) for clock line
#define CLOCK_RATE 100000000    // 100MHz DE1-SoC system clock

// Global variables
volatile int *JP1_ptr;      // Pointer to JP1 expansion port
volatile int *KEY_ptr;      // Pointer to pushbutton KEYs
volatile int *SW_ptr;       // Pointer to slider switches
volatile int *LEDR_ptr;     // Pointer to red LEDs
volatile int *TIMER_ptr;    // Pointer to interval timer

// Communication parameters
#define BIT_PERIOD_MS 50    // 50ms per bit - slower for reliability
#define MSG_BUFFER_SIZE 64  // Buffer size for messages
char tx_buffer[MSG_BUFFER_SIZE];
char rx_buffer[MSG_BUFFER_SIZE];
int rx_buffer_pos = 0;

// Message counter
int message_counter = 0;

// Protocol constants
#define START_BYTE 0xAA     // 10101010 pattern for synchronization
#define END_BYTE 0x55       // 01010101 pattern for end of message

// Debug flag - set to 1 to enable verbose debug output
#define DEBUG 1

// Simple delay in milliseconds
void delay_ms(int ms) {
    volatile int i;
    for (i = 0; i < ms * (CLOCK_RATE / 10000); i++);
}

// Initialize the JP1 port for communication
void init_jp1_communication() {
    // Set both pins as output initially
    *(JP1_ptr + 1) = DATA_PIN_BIT | CLOCK_PIN_BIT;
    
    // Set both pins high (idle state)
    *(JP1_ptr) |= DATA_PIN_BIT | CLOCK_PIN_BIT;
    
    // Add delay for stability
    delay_ms(500); // Longer startup delay
    
    printf("JP1 communication initialized\n");
    printf("DATA_PIN_BIT = 0x%08X (D0), CLOCK_PIN_BIT = 0x%08X (D1)\n", 
           DATA_PIN_BIT, CLOCK_PIN_BIT);
}

// Set the direction of the DATA pin (input or output)
void set_data_pin_direction(int is_output) {
    if (is_output) {
        // Set DATA pin as output
        *(JP1_ptr + 1) |= DATA_PIN_BIT;
    } else {
        // Set DATA pin as input
        *(JP1_ptr + 1) &= ~DATA_PIN_BIT;
    }
    
    // Give hardware time to respond
    delay_ms(5);
}

// Set the DATA pin value (when configured as output)
void set_data_pin(int high) {
    if (high) {
        *(JP1_ptr) |= DATA_PIN_BIT;
    } else {
        *(JP1_ptr) &= ~DATA_PIN_BIT;
    }
}

// Set the CLOCK pin value
void set_clock_pin(int high) {
    if (high) {
        *(JP1_ptr) |= CLOCK_PIN_BIT;
    } else {
        *(JP1_ptr) &= ~CLOCK_PIN_BIT;
    }
}

// Read the DATA pin value
int read_data_pin() {
    return (*(JP1_ptr) & DATA_PIN_BIT) ? 1 : 0;
}

// Send a byte bit by bit (MSB first)
// DE1-SOC controls the clock in this simplified protocol
void send_byte(unsigned char byte) {
    if (DEBUG) printf("Sending byte: 0x%02X\n", byte);
    
    // Ensure DATA pin is set as output
    set_data_pin_direction(1);
    
    // Send each bit MSB first
    for (int i = 7; i >= 0; i--) {
        // Set data bit
        int bit = (byte >> i) & 0x01;
        set_data_pin(bit);
        
        // Wait for data to stabilize
        delay_ms(5);
        
        // Signal bit is ready by toggling clock LOW
        set_clock_pin(0);
        delay_ms(BIT_PERIOD_MS);
        
        // Return clock HIGH to complete bit
        set_clock_pin(1);
        delay_ms(BIT_PERIOD_MS / 2);
        
        if (DEBUG) printf("Sent bit %d: %d\n", i, bit);
    }
}

// Send a message byte by byte
void send_message(char *message) {
    int len = strlen(message);
    
    // Turn on LED 0 during transmission
    *LEDR_ptr |= 0x1;
    
    printf("Sending message: \"%s\" (%d bytes)\n", message, len);
    
    // 1. Initial delay to ensure Arduino is ready
    delay_ms(500);
    
    // 2. Send start byte for synchronization
    send_byte(START_BYTE);
    delay_ms(100); // Extra delay between framing bytes and content
    
    // 3. Send length byte
    send_byte((unsigned char)len);
    delay_ms(100);
    
    // 4. Send each byte of the message
    for (int i = 0; i < len; i++) {
        send_byte((unsigned char)message[i]);
        delay_ms(20); // Small delay between data bytes
    }
    
    // 5. Send end byte
    delay_ms(100);
    send_byte(END_BYTE);
    
    // Extra delay to ensure completion
    delay_ms(500);
    
    // Turn off LED 0
    *LEDR_ptr &= ~0x1;
    
    printf("Message sent\n");
}

// Receive a byte bit by bit (MSB first)
// DE1-SOC controls the clock in this simplified protocol
int receive_byte() {
    unsigned char byte = 0;
    
    // Ensure DATA pin is set as input
    set_data_pin_direction(0);
    
    // Receive 8 bits, MSB first
    for (int i = 7; i >= 0; i--) {
        // Signal ready to receive by toggling clock LOW
        set_clock_pin(0);
        delay_ms(BIT_PERIOD_MS / 2);
        
        // Read data bit
        int bit = read_data_pin();
        byte |= (bit << i);
        
        // Complete bit read by returning clock HIGH
        set_clock_pin(1);
        delay_ms(BIT_PERIOD_MS / 2);
        
        if (DEBUG) printf("Received bit %d: %d\n", i, bit);
    }
    
    if (DEBUG) printf("Received byte: 0x%02X\n", byte);
    return byte;
}

// Receive a message byte by byte
int receive_message() {
    int byte_value, length;
    int i, success = 0;
    
    // Turn on LED 2 during reception
    *LEDR_ptr |= 0x4;
    
    printf("Waiting for message...\n");
    
    // Ensure DATA pin is input
    set_data_pin_direction(0);
    
    // Clock starts HIGH in idle state
    set_clock_pin(1);
    delay_ms(100);
    
    // 1. Look for start byte
    byte_value = receive_byte();
    if (byte_value != START_BYTE) {
        printf("Invalid start byte: 0x%02X, expected 0x%02X\n", byte_value, START_BYTE);
        *LEDR_ptr &= ~0x4; // Turn off LED 2
        return 0;
    }
    
    printf("Start byte detected\n");
    
    // 2. Receive length byte
    byte_value = receive_byte();
    length = byte_value;
    
    printf("Expecting message of %d bytes\n", length);
    
    // Ensure length is reasonable
    if (length >= MSG_BUFFER_SIZE || length <= 0) {
        printf("Invalid message length: %d bytes\n", length);
        *LEDR_ptr &= ~0x4; // Turn off LED 2
        return 0;
    }
    
    // 3. Receive each byte of the message
    rx_buffer_pos = 0;
    for (i = 0; i < length; i++) {
        byte_value = receive_byte();
        rx_buffer[rx_buffer_pos++] = byte_value;
    }
    
    // Null-terminate the string
    rx_buffer[rx_buffer_pos] = '\0';
    
    // 4. Receive end byte
    byte_value = receive_byte();
    
    if (byte_value != END_BYTE) {
        printf("Invalid end byte: 0x%02X, expected 0x%02X\n", byte_value, END_BYTE);
        success = 0;
    } else {
        printf("Message received successfully: \"%s\"\n", rx_buffer);
        success = 1;
    }
    
    // Turn off LED 2
    *LEDR_ptr &= ~0x4;
    
    return success;
}

// Main function
int main(void) {
    // Initialize pointers to I/O devices
    JP1_ptr = (int *)JP1_BASE;
    KEY_ptr = (int *)KEY_BASE;
    SW_ptr = (int *)SW_BASE;
    LEDR_ptr = (int *)LEDR_BASE;
    TIMER_ptr = (int *)TIMER_BASE;
    
    printf("\n\n===================================\n");
    printf("DE1-SoC Simplified Communication Started\n");
    printf("===================================\n");
    
    // Initialize JP1 for communication
    init_jp1_communication();
    
    // Main loop variables
    int key_value, old_key_value = 0;
    int sw_value;
    
    printf("Test Controls:\n");
    printf("- KEY0: Send test message to Arduino\n");
    printf("- KEY1: Wait to receive message from Arduino\n");
    printf("- SW0: Toggle between different test messages\n");
    printf("===================================\n\n");
    
    // Initial LED state
    *LEDR_ptr = 0;
    
    // Main loop
    while (1) {
        // Read key value for edge detection
        key_value = *KEY_ptr;
        
        // Read switch value
        sw_value = *SW_ptr;
        
        // KEY0: Send message to Arduino (active low)
        if (!(key_value & 0x1) && (old_key_value & 0x1)) {
            printf("\nKEY0 pressed - Sending test message\n");
            
            // Create test message based on switch setting
            if (sw_value & 0x1) {
                sprintf(tx_buffer, "DE1_MSG_%d_EXTENDED", message_counter++);
            } else {
                sprintf(tx_buffer, "DE1_MSG_%d", message_counter++);
            }
            
            send_message(tx_buffer);
        }
        
        // KEY1: Receive message from Arduino (active low)
        if (!(key_value & 0x2) && (old_key_value & 0x2)) {
            printf("\nKEY1 pressed - Waiting for message from Arduino\n");
            receive_message();
        }
        
        // Update old key value for edge detection
        old_key_value = key_value;
        
        // Small delay to prevent CPU hogging
        delay_ms(10);
    }
    
    return 0;
}