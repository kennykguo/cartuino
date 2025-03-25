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
char tx_buffer[MSG_BUFFER_SIZE];
char rx_buffer[MSG_BUFFER_SIZE];

// Message counter and state
int message_counter = 0;
int awaiting_response = 0;
int clock_edge_detected = 0;
int prev_clock_state = 1;  // Assume HIGH initially

// Protocol constants
#define START_BYTE 0xAA  // 10101010 pattern for synchronization
#define END_BYTE 0x55    // 01010101 pattern for end of message
#define ACK_BYTE 0xCC    // 11001100 pattern for acknowledgment

// Simple delay in milliseconds - improved for better accuracy
void delay_ms(int ms) {
    volatile int i;
    // Adjusted calibration factor for more accurate timing
    // The division factor is reduced to create longer delays
    for (i = 0; i < ms * (CLOCK_RATE / 8000); i++);
}

// Initialize the JP1 port for communication
void init_jp1_communication() {
    // Set both pins as input initially (slave mode)
    *(JP1_ptr + 1) = 0x00000000;
    
    printf("DE1-SoC Slave Communication Initialized\n");
    printf("Using D0 (data) and D1 (clock) from Arduino D2/D3\n");
    printf("Running in asynchronous bidirectional mode\n");
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
    
    // Increased delay for pin state to stabilize
    delay_ms(10);  // Increased from 2ms to 10ms
}

// Set the DATA pin value (when configured as output)
void set_data_pin(int high) {
    if (high) {
        *(JP1_ptr) |= DATA_PIN_BIT;
    } else {
        *(JP1_ptr) &= ~DATA_PIN_BIT;
    }
    
    // Increased delay for value to stabilize
    delay_ms(5);  // Increased from 2ms to 5ms
}

// Read the DATA pin value
int read_data_pin() {
    return (*(JP1_ptr) & DATA_PIN_BIT) ? 1 : 0;
}

// Read the CLOCK pin value
int read_clock_pin() {
    return (*(JP1_ptr) & CLOCK_PIN_BIT) ? 1 : 0;
}

// Check if a clock edge has been detected (HIGH to LOW transition)
// Now with debouncing for more reliable detection
int detect_clock_edge() {
    int current_clock = read_clock_pin();
    int edge_detected = 0;
    
    if (prev_clock_state == 1 && current_clock == 0) {
        // Confirm it's a real edge with a small delay and second read
        delay_ms(1);
        current_clock = read_clock_pin();
        if (current_clock == 0) {  // Still LOW after delay
            edge_detected = 1;
        }
    }
    
    prev_clock_state = current_clock;
    return edge_detected;
}

// Wait for clock edge with timeout protection and multiple samples
// Improved version with more samples and longer timeout
int wait_for_clock_edge(int wait_for_high) {
    int i, j;
    const int MAX_WAIT = 2000000;  // Increased timeout
    const int SAMPLE_COUNT = 5;    // More samples for better reliability
    
    // Wait for desired clock state
    for (i = 0; i < MAX_WAIT; i++) {
        int consecutive_matches = 0;
        
        // Take multiple samples
        for (j = 0; j < SAMPLE_COUNT; j++) {
            int clock_state = read_clock_pin();
            if ((wait_for_high && clock_state) || (!wait_for_high && !clock_state)) {
                consecutive_matches++;
            } else {
                consecutive_matches = 0;
                break;
            }
            
            // Small delay between samples
            delay_ms(1);
        }
        
        // If we got enough consecutive matches, we've detected a stable edge
        if (consecutive_matches >= SAMPLE_COUNT) {
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
    
    // Wait a bit for data to stabilize
    delay_ms(2);
    
    // Read data bit
    bit = read_data_pin();
    
    // Wait for clock to go HIGH
    if (!wait_for_clock_edge(1)) {
        printf("Timeout waiting for clock HIGH during bit receive\n");
    }
    
    return bit;
}

// Send a bit by waiting for clock transitions from master
void send_bit(int bit) {
    // Set data line to bit value
    set_data_pin_direction(1);  // Set as output
    set_data_pin(bit);
    
    // Wait for clock to go LOW
    if (!wait_for_clock_edge(0)) {
        printf("Timeout waiting for clock LOW during bit send\n");
        return;
    }
    
    // Data is now being read by master, wait for clock to go HIGH
    if (!wait_for_clock_edge(1)) {
        printf("Timeout waiting for clock HIGH during bit send\n");
    }
    
    // Hold data for a moment after clock edge
    delay_ms(5);  // Increased to ensure data is stable
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

// Send a byte (8 bits) MSB first
void send_byte(unsigned char byte) {
    printf("Sending byte: 0x%02X\n", byte);
    
    // Send each bit, MSB first
    for (int i = 7; i >= 0; i--) {
        int bit = (byte >> i) & 0x01;
        send_bit(bit);
    }
}

// Receive a message and automatically send a response
int receive_message() {
    unsigned char byte, length, checksum = 0, calculated_checksum = 0;
    int success = 0;
    
    // Turn on LED 0 during reception
    *LEDR_ptr |= 0x1;
    
    printf("Receiving message from Arduino...\n");
    
    // Try to detect start byte with retries
    int max_retries = 5;
    int retry_count = 0;
    int found_start = 0;
    
    while (retry_count < max_retries && !found_start) {
        byte = receive_byte();
        if (byte == START_BYTE) {
            printf("Valid start byte detected (0x%02X)\n", byte);
            found_start = 1;
        } else {
            printf("Attempt %d: Invalid start byte: 0x%02X (expected 0x%02X)\n", 
                   retry_count + 1, byte, START_BYTE);
            retry_count++;
            delay_ms(50);  // Longer delay between retry attempts
        }
    }
    
    if (!found_start) {
        printf("Failed to detect valid start byte after %d attempts\n", max_retries);
        *LEDR_ptr &= ~0x1;
        return 0;
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
        printf("Invalid end byte: 0x%02X (expected 0x%02X)\n", byte, END_BYTE);
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
    
    if (success) {
        // Turn on LED 1 to indicate successful reception
        *LEDR_ptr |= 0x2;
        
        // Prepare automatic response
        sprintf(tx_buffer, "DE1_MSG_%d", message_counter++);
        printf("Preparing response: \"%s\"\n", tx_buffer);
        
        // Longer delay to allow Arduino to switch to receive mode
        delay_ms(300);  // Increased from 200ms to 300ms
        
        // Send automatic response
        send_response(tx_buffer);
        
        // Turn off LED 1
        *LEDR_ptr &= ~0x2;
    } else {
        // Flash LED 3 to indicate error
        for (int i = 0; i < 3; i++) {
            *LEDR_ptr |= 0x8;
            delay_ms(100);
            *LEDR_ptr &= ~0x8;
            delay_ms(100);
        }
    }
    
    return success;
}

// Send response message - with improved synchronization
void send_response(char *message) {
    int len = strlen(message);
    unsigned char checksum = 0;
    
    // Turn on LED 2 during transmission
    *LEDR_ptr |= 0x4;
    
    printf("Sending response: \"%s\" (%d bytes)\n", message, len);
    
    // Add synchronization pulses to help Arduino detect the start of transmission
    set_data_pin_direction(1);  // Set as output
    
    // Force data line to specific pattern to help synchronization
    for (int i = 0; i < 5; i++) {
        set_data_pin(1);
        delay_ms(10);
        set_data_pin(0);
        delay_ms(10);
    }
    
    // Final sync pulse
    set_data_pin(1);
    delay_ms(50);
    
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
    
    // Wait for ACK with longer timeout
    delay_ms(50);  // Extra delay before switching to receive mode
    
    // Set data pin as input to receive ACK
    set_data_pin_direction(0);
    
    // Wait for ACK
    unsigned char response = receive_byte();
    if (response == ACK_BYTE) {
        printf("Received ACK - response successfully delivered\n");
    } else {
        printf("Did not receive ACK - got 0x%02X instead\n", response);
    }
    
    // Turn off LED 2
    *LEDR_ptr &= ~0x4;
}

// Check for clock activity
void check_for_communication() {
    // Check for a falling edge on the clock line
    if (detect_clock_edge()) {
        printf("\nClock activity detected - preparing to receive message\n");
        clock_edge_detected = 1;
    }
    
    // If we detected a clock edge, start message reception process
    if (clock_edge_detected) {
        // Reset the flag
        clock_edge_detected = 0;
        
        // Receive the message and send response
        if (receive_message()) {
            printf("Communication cycle completed successfully\n");
        } else {
            printf("Communication cycle failed\n");
        }
    }
}

// Main function
int main(void) {
    // Initialize pointers to I/O devices
    JP1_ptr = (int *)JP1_BASE;
    KEY_ptr = (int *)KEY_BASE;
    LEDR_ptr = (int *)LEDR_BASE;
    
    printf("\n\n===================================\n");
    printf("DE1-SoC Asynchronous Communication\n");
    printf("===================================\n");
    
    // Initialize JP1 for communication
    init_jp1_communication();
    
    // Main loop variables
    int key_value, old_key_value = 0;
    
    // Initial LED state
    *LEDR_ptr = 0;
    
    printf("Operating in automatic mode - no KEY press needed\n");
    printf("KEY0 can still be used to manually trigger reception\n");
    printf("===================================\n\n");
    
    // Main loop
    while (1) {
        // Read key value for edge detection
        key_value = *KEY_ptr;
        
        // Check for incoming communication (asynchronous detection)
        check_for_communication();
        
        // KEY0: Manual trigger for message reception (still supported)
        if ((key_value & 0x1) && !(old_key_value & 0x1)) {
            printf("\nKEY0 pressed - Manual reception trigger\n");
            
            // Force message reception process
            if (receive_message()) {
                printf("Manual communication cycle completed successfully\n");
            } else {
                printf("Manual communication cycle failed\n");
            }
        }
        
        // Update old key value for edge detection
        old_key_value = key_value;
        
        // Small delay to prevent CPU hogging
        delay_ms(5);
    }
    
    return 0;
}