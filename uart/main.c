#include <stdio.h>
#include <string.h>
#include "address_map.h"

// JP1 Pin definitions - using different pins from original
#define DATA_PIN_BIT 0x00000001  // Bit 0 (D0) for data line
#define CLOCK_PIN_BIT 0x00000002 // Bit 1 (D1) for clock line (from Arduino)
#define CLOCK_RATE 100000000     // 100MHz DE1-SoC system clock

// Global variables
volatile int *JP1_ptr;      // Pointer to JP1 expansion port
volatile int *KEY_ptr;      // Pointer to pushbutton KEYs
volatile int *LEDR_ptr;     // Pointer to red LEDs

// Communication parameters
#define MSG_BUFFER_SIZE 32  // Buffer size for messages
char tx_buffer[MSG_BUFFER_SIZE];
char rx_buffer[MSG_BUFFER_SIZE];

// Message counter
int message_counter = 0;

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
    // Set DATA pin as input by default, CLOCK pin always as input (slave mode)
    *(JP1_ptr + 1) = 0;  // All pins as input initially
    
    printf("JP1 communication initialized\n");
    printf("DATA_PIN_BIT = 0x%08X (D2), CLOCK_PIN_BIT = 0x%08X (D3)\n", 
           DATA_PIN_BIT, CLOCK_PIN_BIT);
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
    delay_ms(1);
}

// Set the DATA pin value (when configured as output)
void set_data_pin(int high) {
    if (high) {
        *(JP1_ptr) |= DATA_PIN_BIT;
    } else {
        *(JP1_ptr) &= ~DATA_PIN_BIT;
    }
    
    // Small delay for value to stabilize
    delay_ms(1);
}

// Read the DATA pin value with multiple samples for stability
int read_data_pin() {
    // Read multiple times to ensure stable reading
    int sample1 = (*(JP1_ptr) & DATA_PIN_BIT) ? 1 : 0;
    delay_ms(1);
    int sample2 = (*(JP1_ptr) & DATA_PIN_BIT) ? 1 : 0;
    
    // Return the majority value
    return (sample1 == sample2) ? sample1 : (*(JP1_ptr) & DATA_PIN_BIT) ? 1 : 0;
}

// Read the CLOCK pin value with multiple samples for stability
int read_clock_pin() {
    // Read multiple times to ensure stable reading
    int sample1 = (*(JP1_ptr) & CLOCK_PIN_BIT) ? 1 : 0;
    delay_ms(1);
    int sample2 = (*(JP1_ptr) & CLOCK_PIN_BIT) ? 1 : 0;
    
    // Return the majority value
    return (sample1 == sample2) ? sample1 : (*(JP1_ptr) & CLOCK_PIN_BIT) ? 1 : 0;
}

// Receive a single bit by waiting for clock transitions with robust edge detection
int receive_bit() {
    int bit;
    int current_clock, prev_clock;
    int timeout_counter = 0;
    const int MAX_TIMEOUT = 1000000;  // Arbitrary large value for timeout
    
    // Make sure DATA pin is set as input
    set_data_pin_direction(0);
    
    // Read initial clock state with a small delay for stability
    delay_ms(1);
    prev_clock = read_clock_pin();
    
    // Wait for clock to transition from HIGH to LOW
    while (1) {
        current_clock = read_clock_pin();
        
        if (prev_clock == 1 && current_clock == 0) {
            // Clock transition detected, wait a small moment for data to stabilize
            delay_ms(1);
            
            // Read the data bit
            bit = read_data_pin();
            break;
        }
        
        prev_clock = current_clock;
        
        // Timeout prevention
        timeout_counter++;
        if (timeout_counter > MAX_TIMEOUT) {
            printf("Timeout waiting for clock HIGH to LOW transition\n");
            // Default to reading current data value on timeout
            bit = read_data_pin();
            break;
        }
    }
    
    // Reset timeout counter
    timeout_counter = 0;
    
    // Wait for clock to transition back from LOW to HIGH
    while (read_clock_pin() == 0) {
        timeout_counter++;
        if (timeout_counter > MAX_TIMEOUT) {
            printf("Timeout waiting for clock LOW to HIGH transition\n");
            break;
        }
    }
    
    return bit;
}

// Receive a byte (8 bits) MSB first
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

// Send a single bit by monitoring clock transitions with timeout protection
void send_bit(int bit) {
    int timeout_counter = 0;
    const int MAX_TIMEOUT = 1000000;  // Arbitrary large value for timeout
    
    // Set data line to the desired bit value
    set_data_pin_direction(1);  // Set as output
    set_data_pin(bit);
    
    // Make sure the data is stable before clock edge
    delay_ms(1);
    
    // Wait for clock to transition from HIGH to LOW
    while (read_clock_pin() == 1) {
        timeout_counter++;
        if (timeout_counter > MAX_TIMEOUT) {
            printf("Timeout waiting for clock HIGH to LOW transition during send\n");
            break;
        }
    }
    
    // Reset timeout counter
    timeout_counter = 0;
    
    // Wait for clock to transition back from LOW to HIGH
    while (read_clock_pin() == 0) {
        timeout_counter++;
        if (timeout_counter > MAX_TIMEOUT) {
            printf("Timeout waiting for clock LOW to HIGH transition during send\n");
            break;
        }
    }
    
    // Hold the data bit stable for a moment after the clock edge
    delay_ms(1);
}

// Send a byte (8 bits) MSB first
void send_byte(unsigned char byte) {
    printf("Sending byte: 0x%02X\n", byte);
    
    // Send each bit, MSB first
    for (int i = 7; i >= 0; i--) {
        int bit = (byte >> i) & 0x01;
        send_bit(bit);
    }
}

// Receive a message with basic framing and improved robustness
int receive_message() {
    unsigned char byte, length, checksum = 0, calculated_checksum = 0;
    int success = 0;
    
    // Turn on LED 0 during reception
    *LEDR_ptr |= 0x1;
    
    printf("Waiting for message from Arduino...\n");
    
    // Clear the data pin direction first for clean start
    set_data_pin_direction(0);
    delay_ms(50);
    
    // Try multiple times to get a valid start byte
    int retry_count = 0;
    const int MAX_RETRIES = 5;
    
    while (retry_count < MAX_RETRIES) {
        // Wait for start byte
        byte = receive_byte();
        if (byte == START_BYTE) {
            printf("Valid start byte detected: 0x%02X\n", byte);
            break;
        } else {
            printf("Invalid start byte on attempt %d: 0x%02X, expected 0x%02X\n", 
                   retry_count + 1, byte, START_BYTE);
            retry_count++;
            
            if (retry_count >= MAX_RETRIES) {
                printf("Exceeded maximum retries for start byte\n");
                *LEDR_ptr &= ~0x1;
                return 0;
            }
            
            // Small delay before next attempt
            delay_ms(50);
        }
    }
    
    // Receive length byte
    length = receive_byte();
    calculated_checksum ^= length;
    
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
        calculated_checksum ^= byte;
    }
    
    // Null-terminate the string
    rx_buffer[pos] = '\0';
    
    // Receive checksum
    checksum = receive_byte();
    
    // Receive end byte
    byte = receive_byte();
    
    // Verify checksum and end byte
    if (byte != END_BYTE) {
        printf("Invalid end byte: 0x%02X, expected 0x%02X\n", byte, END_BYTE);
        success = 0;
    } else if (checksum != calculated_checksum) {
        printf("Checksum error: received 0x%02X, calculated 0x%02X\n", checksum, calculated_checksum);
        success = 0;
    } else {
        printf("Message received successfully: \"%s\"\n", rx_buffer);
        success = 1;
    }
    
    // Turn off LED 0
    *LEDR_ptr &= ~0x1;
    
    return success;
}

// Send a message with basic framing
void send_message(char *message) {
    int len = strlen(message);
    unsigned char checksum = 0;
    
    // Turn on LED 1 during transmission
    *LEDR_ptr |= 0x2;
    
    printf("Sending message: \"%s\" (%d bytes)\n", message, len);
    
    // Send start byte
    send_byte(START_BYTE);
    
    // Send length byte
    send_byte((unsigned char)len);
    checksum ^= len;
    
    // Send each byte of the message
    for (int i = 0; i < len; i++) {
        send_byte((unsigned char)message[i]);
        checksum ^= message[i];
    }
    
    // Send checksum
    send_byte(checksum);
    
    // Send end byte
    send_byte(END_BYTE);
    
    // Turn off LED 1
    *LEDR_ptr &= ~0x2;
    
    printf("Message sent successfully\n");
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
    
    // Main loop
    while (1) {
        // Read key value for edge detection
        key_value = *KEY_ptr;
        
        // KEY0: Initiate message exchange on request
        if ((key_value & 0x1) && !(old_key_value & 0x1)) {
            printf("\nKEY0 pressed - Waiting for Arduino message\n");
            
            // Wait to receive a message from Arduino
            if (receive_message()) {
                // Prepare response message
                sprintf(tx_buffer, "DE1_MSG_%d", message_counter++);
                
                // Important delay to allow Arduino to switch to receive mode
                delay_ms(300);
                
                // Reset the data pin state explicitly
                set_data_pin_direction(0);  // First set as input to reset
                delay_ms(50);
                set_data_pin_direction(1);  // Then set as output for sending
                
                // Send response message
                send_message(tx_buffer);
            }
        }
        
        // Update old key value for edge detection
        old_key_value = key_value;
        
        // Small delay to prevent CPU hogging
        delay_ms(10);
    }
    
    return 0;
}