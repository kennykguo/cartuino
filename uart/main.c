#include <stdio.h>
#include <string.h>
#include "address_map.h"

// JP1 Pin definitions
#define DATA_PIN_BIT 0x00000001  // Bit 0 (D0) for data line from Arduino D2
#define CLOCK_PIN_BIT 0x00000002 // Bit 1 (D1) for clock line from Arduino D3
#define CLOCK_RATE 100000000     // 100MHz DE1-SoC system clock

// Global variables
volatile int *JP1_ptr;      // Pointer to JP1 expansion port
volatile int *KEY_ptr;      // Pointer to pushbutton KEYs
volatile int *LEDR_ptr;     // Pointer to red LEDs

// Communication parameters
#define BIT_PERIOD_MS 20    // 20ms per bit (matching Arduino)
#define MSG_BUFFER_SIZE 32  // Buffer size for messages
char rx_buffer[MSG_BUFFER_SIZE];

// Protocol constants
#define START_BYTE 0xAA  // 10101010 pattern for synchronization
#define END_BYTE 0x55    // 01010101 pattern for end of message

// Simple delay in milliseconds
void delay_ms(int ms) {
    volatile int i;
    for (i = 0; i < ms * (CLOCK_RATE / 10000); i++);
}

// Initialize the JP1 port for communication
void init_jp1_communication() {
    // Set both pins as input initially (slave mode)
    *(JP1_ptr + 1) = 0x00000000;
    
    printf("DE1-SoC Slave Communication Initialized\n");
    printf("Using D0 (data) and D1 (clock) from Arduino D2/D3\n");
    printf("JP1 direction register value: 0x%08X\n", *(JP1_ptr + 1));
    printf("JP1 data register value: 0x%08X\n", *(JP1_ptr));
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
    
    // Small delay for pin state to stabilize
    delay_ms(5);
}

// Set the DATA pin value (when configured as output)
void set_data_pin(int high) {
    if (high) {
        *(JP1_ptr) |= DATA_PIN_BIT;
    } else {
        *(JP1_ptr) &= ~DATA_PIN_BIT;
    }
    
    // Small delay for value to stabilize
    delay_ms(5);
}

// Read the DATA pin value
int read_data_pin() {
    return (*(JP1_ptr) & DATA_PIN_BIT) ? 1 : 0;
}

// Read the CLOCK pin value
int read_clock_pin() {
    return (*(JP1_ptr) & CLOCK_PIN_BIT) ? 1 : 0;
}

// Wait for clock edge with timeout protection
int wait_for_clock_edge(int wait_for_high) {
    int i;
    const int MAX_WAIT = 1000000;  // Arbitrary timeout value
    
    // Wait for desired clock state
    for (i = 0; i < MAX_WAIT; i++) {
        int clock_state = read_clock_pin();
        if ((wait_for_high && clock_state) || (!wait_for_high && !clock_state)) {
            return 1;  // Success
        }
    }
    
    printf("Clock timeout waiting for %s\n", wait_for_high ? "HIGH" : "LOW");
    return 0;  // Timeout
}

// Receive a single bit with explicit clock edge detection
int receive_bit() {
    int bit;
    
    // Make sure DATA pin is set as input
    set_data_pin_direction(0);
    
    // Wait for clock to go LOW
    if (!wait_for_clock_edge(0)) {
        printf("Timeout waiting for clock LOW during bit receive\n");
        return 0;  // Default to 0 on timeout
    }
    
    // Read data bit
    bit = read_data_pin();
    
    // Wait for clock to go HIGH
    if (!wait_for_clock_edge(1)) {
        printf("Timeout waiting for clock HIGH during bit receive\n");
    }
    
    return bit;
}

// Receive a byte (8 bits) MSB first with edge detection
unsigned char receive_byte() {
    unsigned char byte = 0;
    
    // Receive 8 bits, MSB first
    for (int i = 7; i >= 0; i--) {
        int bit = receive_bit();
        byte |= (bit << i);
    }
    
    printf("Received byte: 0x%02X\n", byte);
    return byte;
}

// Wait for start byte with timeout
int wait_for_start_byte(int max_attempts) {
    unsigned char byte;
    
    for (int i = 0; i < max_attempts; i++) {
        byte = receive_byte();
        if (byte == START_BYTE) {
            printf("Valid start byte detected (0x%02X)\n", byte);
            return 1;  // Start byte found
        } else {
            printf("Attempt %d: Invalid start byte: 0x%02X (expected 0x%02X)\n", 
                   i+1, byte, START_BYTE);
        }
    }
    
    printf("Failed to detect valid start byte after %d attempts\n", max_attempts);
    return 0;  // Start byte not found
}

// Receive a simple message
int receive_simple_message() {
    unsigned char byte, length;
    int success = 0;
    
    // Turn on LED 0 during reception
    *LEDR_ptr |= 0x1;
    
    printf("Waiting for message from Arduino...\n");
    
    // Try to detect start byte
    if (!wait_for_start_byte(3)) {
        printf("Failed to detect start byte sequence\n");
        *LEDR_ptr &= ~0x1;
        return 0;
    }
    
    // Receive length byte
    length = receive_byte();
    
    printf("Expecting message of %d bytes\n", length);
    
    // Ensure length is reasonable
    if (length >= MSG_BUFFER_SIZE) {
        printf("Message too long: %d bytes\n", length);
        *LEDR_ptr &= ~0x1;
        return 0;
    }
    
    // Receive message bytes
    int pos = 0;
    for (int i = 0; i < length; i++) {
        byte = receive_byte();
        rx_buffer[pos++] = byte;
    }
    
    // Null-terminate the string
    rx_buffer[pos] = '\0';
    
    // Receive end byte
    byte = receive_byte();
    
    // Verify end byte
    if (byte != END_BYTE) {
        printf("Invalid end byte: 0x%02X (expected 0x%02X)\n", byte, END_BYTE);
        success = 0;
    } else {
        printf("Message received successfully: \"%s\"\n", rx_buffer);
        success = 1;
    }
    
    // Turn off LED 0
    *LEDR_ptr &= ~0x1;
    
    return success;
}

// Main function
int main(void) {
    // Initialize pointers to I/O devices
    JP1_ptr = (int *)JP1_BASE;
    KEY_ptr = (int *)KEY_BASE;
    LEDR_ptr = (int *)LEDR_BASE;
    
    printf("\n\n===================================\n");
    printf("DE1-SoC Slave Communication Started\n");
    printf("===================================\n");
    
    // Initialize JP1 for communication
    init_jp1_communication();
    
    // Main loop variables
    int key_value, old_key_value = 0;
    
    // Initial LED state
    *LEDR_ptr = 0;
    
    printf("Press KEY0 to attempt to receive a message\n");
    printf("===================================\n\n");
    
    // Main loop
    while (1) {
        // Read key value for edge detection
        key_value = *KEY_ptr;
        
        // KEY0: Initiate message reception on request
        if ((key_value & 0x1) && !(old_key_value & 0x1)) {
            printf("\nKEY0 pressed - Listening for Arduino message\n");
            
            // Ensure data pin is set as input
            set_data_pin_direction(0);
            
            // Wait to receive a message
            if (receive_simple_message()) {
                printf("Message successfully received: %s\n", rx_buffer);
                
                // Turn on LED 1 to indicate success
                *LEDR_ptr |= 0x2;
                delay_ms(1000);
                *LEDR_ptr &= ~0x2;
            } else {
                printf("Message reception failed\n");
                
                // Flash LED 3 to indicate failure
                for (int i = 0; i < 3; i++) {
                    *LEDR_ptr |= 0x8;
                    delay_ms(200);
                    *LEDR_ptr &= ~0x8;
                    delay_ms(200);
                }
            }
        }
        
        // Update old key value for edge detection
        old_key_value = key_value;
        
        // Small delay to prevent CPU hogging
        delay_ms(10);
    }
    
    return 0;
}