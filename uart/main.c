// DE1-SoC Asynchronous Communication Protocol - Simplified Version
// Uses software UART on a single pin (D0)

#include <stdio.h>
#include <string.h>
#include "address_map.h"

// JP1 Pin definitions
#define DATA_PIN_BIT 0x00000001  // Bit 0 (D0) for data line
#define CLOCK_RATE 100000000     // 100MHz DE1-SoC system clock

// Global variables
volatile int *JP1_ptr;      // Pointer to JP1 expansion port
volatile int *KEY_ptr;      // Pointer to pushbutton KEYs
volatile int *LEDR_ptr;     // Pointer to red LEDs

// Communication parameters
#define BIT_PERIOD_MS 5     // 5ms per bit (200 bps) - slow but reliable
#define MSG_BUFFER_SIZE 32  // Buffer size for messages
char tx_buffer[MSG_BUFFER_SIZE];
char rx_buffer[MSG_BUFFER_SIZE];

// Message counter
int message_counter = 0;

// Protocol constants
#define START_SEQUENCE 0xAA // 10101010 pattern for synchronization
#define END_SEQUENCE 0x55   // 01010101 pattern for end of message

// Simple delay in milliseconds
void delay_ms(int ms) {
    volatile int i;
    for (i = 0; i < ms * (CLOCK_RATE / 10000); i++);
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
}

// Set the DATA pin value (when configured as output)
void set_data_pin(int high) {
    if (high) {
        *(JP1_ptr) |= DATA_PIN_BIT;
    } else {
        *(JP1_ptr) &= ~DATA_PIN_BIT;
    }
}

// Read the DATA pin value
int read_data_pin() {
    return (*(JP1_ptr) & DATA_PIN_BIT) ? 1 : 0;
}

// Initialize the JP1 port for communication
void init_jp1_communication() {
    printf("Initializing JP1 communication...\n");
    
    // Set DATA pin as output initially to ensure a known state
    set_data_pin_direction(1);
    
    // Set DATA pin HIGH (idle state for serial)
    set_data_pin(1);
    
    // Hold for a moment to ensure line is stable
    delay_ms(100);
    
    // Now set to input for receiving
    set_data_pin_direction(0);
    
    printf("JP1 initialized, DATA_PIN_BIT = 0x%08X (D0)\n", DATA_PIN_BIT);
}

// Send a byte using software UART approach
void send_byte(unsigned char byte) {
    printf("Sending byte: 0x%02X\n", byte);
    
    // Set DATA pin as output
    set_data_pin_direction(1);
    
    // Send start bit (always 0)
    set_data_pin(0);
    delay_ms(BIT_PERIOD_MS);
    
    // Send 8 data bits, LSB first
    for (int i = 0; i < 8; i++) {
        int bit = (byte >> i) & 0x01;
        set_data_pin(bit);
        delay_ms(BIT_PERIOD_MS);
    }
    
    // Send stop bit (always 1)
    set_data_pin(1);
    delay_ms(BIT_PERIOD_MS);
    
    // Keep line in idle state
    set_data_pin(1);
}

// Receive a byte using software UART approach
// Returns -1 on error
int receive_byte(int timeout_ms) {
    unsigned char byte = 0;
    int start_bit_found = 0;
    int timeout_counter = 0;
    
    // Set DATA pin as input
    set_data_pin_direction(0);
    
    // Wait for start bit (logic 0)
    while (!start_bit_found) {
        if (read_data_pin() == 0) {
            // Potential start bit detected, wait for half a bit period to verify
            delay_ms(BIT_PERIOD_MS / 2);
            
            // Check if it's still low (valid start bit)
            if (read_data_pin() == 0) {
                start_bit_found = 1;
                printf("Start bit detected\n");
            }
        }
        
        // Check for timeout
        timeout_counter++;
        delay_ms(1);
        if (timeout_counter > timeout_ms) {
            printf("Timeout waiting for start bit\n");
            return -1;
        }
    }
    
    // Start bit found, wait for the remaining half of start bit
    delay_ms(BIT_PERIOD_MS / 2);
    
    // Read 8 data bits
    for (int i = 0; i < 8; i++) {
        // Sample in the middle of each bit
        delay_ms(BIT_PERIOD_MS);
        if (read_data_pin()) {
            byte |= (1 << i);  // LSB first
        }
    }
    
    // Wait for and verify stop bit
    delay_ms(BIT_PERIOD_MS);
    if (read_data_pin() != 1) {
        printf("Invalid stop bit\n");
        return -1;
    }
    
    printf("Received byte: 0x%02X\n", byte);
    return byte;
}

// Send a message with simple framing
void send_message(char *message) {
    int len = strlen(message);
    
    // Turn on LED 0 during transmission
    *LEDR_ptr |= 0x1;
    
    printf("Sending message: \"%s\" (%d bytes)\n", message, len);
    
    // Send start sequence byte
    send_byte(START_SEQUENCE);
    
    // Send length byte
    send_byte(len);
    
    // Send message bytes
    for (int i = 0; i < len; i++) {
        send_byte(message[i]);
    }
    
    // Send end sequence byte
    send_byte(END_SEQUENCE);
    
    // Set pin back to input to listen for response
    set_data_pin_direction(0);
    
    // Turn off LED 0
    *LEDR_ptr &= ~0x1;
    
    printf("Message sent, waiting for response...\n");
}

// Receive a message with simple framing
int receive_message(int timeout_ms) {
    int byte_val;
    int len;
    
    // Turn on LED 1 during reception
    *LEDR_ptr |= 0x2;
    
    printf("Waiting for message...\n");
    
    // Wait for and verify start sequence
    byte_val = receive_byte(timeout_ms);
    if (byte_val != START_SEQUENCE) {
        printf("Invalid start sequence: 0x%02X, expected 0x%02X\n", byte_val, START_SEQUENCE);
        *LEDR_ptr &= ~0x2; // Turn off LED 1
        return 0;
    }
    
    // Receive length byte
    byte_val = receive_byte(timeout_ms);
    if (byte_val < 0) {
        printf("Error receiving length byte\n");
        *LEDR_ptr &= ~0x2; // Turn off LED 1
        return 0;
    }
    len = byte_val;
    
    printf("Expecting message of %d bytes\n", len);
    
    // Ensure length is reasonable
    if (len >= MSG_BUFFER_SIZE) {
        printf("Message too long: %d bytes\n", len);
        *LEDR_ptr &= ~0x2; // Turn off LED 1
        return 0;
    }
    
    // Receive each byte of the message
    for (int i = 0; i < len; i++) {
        byte_val = receive_byte(timeout_ms);
        if (byte_val < 0) {
            printf("Error receiving data byte %d\n", i);
            *LEDR_ptr &= ~0x2; // Turn off LED 1
            return 0;
        }
        rx_buffer[i] = byte_val;
    }
    
    // Null-terminate the string
    rx_buffer[len] = '\0';
    
    // Receive end sequence
    byte_val = receive_byte(timeout_ms);
    if (byte_val != END_SEQUENCE) {
        printf("Invalid end sequence: 0x%02X, expected 0x%02X\n", byte_val, END_SEQUENCE);
        *LEDR_ptr &= ~0x2; // Turn off LED 1
        return 0;
    }
    
    printf("Message received successfully: \"%s\"\n", rx_buffer);
    
    // Turn off LED 1
    *LEDR_ptr &= ~0x2;
    
    return 1;
}

// Main function
int main(void) {
    // Initialize pointers to I/O devices
    JP1_ptr = (int *)JP1_BASE;
    KEY_ptr = (int *)KEY_BASE;
    LEDR_ptr = (int *)LEDR_BASE;
    
    printf("\n\n===================================\n");
    printf("DE1-SoC Simple Communication Protocol\n");
    printf("===================================\n");
    
    // Initialize JP1 for communication
    init_jp1_communication();
    
    // Main loop variables
    int key_value, old_key_value = 0;
    
    printf("Test Controls:\n");
    printf("- KEY0: Send test message to Arduino\n");
    printf("===================================\n\n");
    
    // Initial LED state
    *LEDR_ptr = 0;
    
    // Main loop
    while (1) {
        // Read key value for edge detection
        key_value = *KEY_ptr;
        
        // KEY0: Send message to Arduino
        if ((key_value & 0x1) && !(old_key_value & 0x1)) {
            printf("\nKEY0 pressed - Sending test message\n");
            
            // Create test message
            sprintf(tx_buffer, "DE1_MSG_%d", message_counter++);
            
            // Send the message
            send_message(tx_buffer);
            
            // Wait for response with timeout
            if (receive_message(2000)) {  // 2 second timeout
                printf("Response received: %s\n", rx_buffer);
            } else {
                printf("No valid response received\n");
            }
        }
        
        // Update old key value for edge detection
        old_key_value = key_value;
        
        // Small delay to prevent CPU hogging
        delay_ms(10);
    }
    
    return 0;
}