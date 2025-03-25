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
#define BIT_PERIOD_MS 50    // 50ms per bit (matching Arduino)
#define MSG_BUFFER_SIZE 32  // Buffer size for messages
char rx_buffer[MSG_BUFFER_SIZE];

// State tracking
int message_counter = 0;
int clock_edge_detected = 0;
int prev_clock_state = 1;  // Assume HIGH initially
int reset_sequence_detected = 0;

// Protocol constants
#define START_BYTE 0xAA  // 10101010 pattern for synchronization

// Simple delay in milliseconds
void delay_ms(int ms) {
    volatile int i;
    for (i = 0; i < ms * (CLOCK_RATE / 10000); i++);
}

// Initialize the JP1 port for communication
void init_jp1_communication() {
    // Start with both pins as input
    *(JP1_ptr + 1) = 0x00000000;
    
    printf("DE1-SoC Simple Receiver Initialized\n");
    printf("Using D0 (data) and D1 (clock) from Arduino D2/D3\n");
    printf("Running in asynchronous receive-only mode\n");
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

// Read the DATA pin value
int read_data_pin() {
    return (*(JP1_ptr) & DATA_PIN_BIT) ? 1 : 0;
}

// Read the CLOCK pin value
int read_clock_pin() {
    return (*(JP1_ptr) & CLOCK_PIN_BIT) ? 1 : 0;
}

// Check if a clock edge has been detected (HIGH to LOW transition)
int detect_clock_edge() {
    int current_clock = read_clock_pin();
    int edge_detected = 0;
    
    if (prev_clock_state == 1 && current_clock == 0) {
        edge_detected = 1;
    }
    
    prev_clock_state = current_clock;
    return edge_detected;
}

// Wait for clock to be in specified state with timeout
int wait_for_clock_state(int wait_for_high, int timeout_ms) {
    int i;
    const int SAMPLES_PER_MS = CLOCK_RATE / 10000;
    const int MAX_SAMPLES = timeout_ms * SAMPLES_PER_MS;
    
    for (i = 0; i < MAX_SAMPLES; i++) {
        int clock_state = read_clock_pin();
        if ((wait_for_high && clock_state) || (!wait_for_high && !clock_state)) {
            return 1;  // Success - clock is in desired state
        }
    }
    
    printf("Clock timeout waiting for %s (after %d ms)\n", 
           wait_for_high ? "HIGH" : "LOW", timeout_ms);
    return 0;  // Timeout
}

// Receive a single bit with generous timeout
int receive_bit() {
    int bit;
    
    // Make sure DATA pin is set as input
    set_data_pin_direction(0);
    
    // Wait for clock to go LOW (up to 200ms)
    if (!wait_for_clock_state(0, 200)) {
        printf("Timeout waiting for clock LOW during bit receive\n");
        return 0;  // Default to 0 on timeout
    }
    
    // Read the data bit with a small delay for stability
    delay_ms(5);
    bit = read_data_pin();
    
    // Wait for clock to go HIGH (up to 200ms)
    if (!wait_for_clock_state(1, 200)) {
        printf("Timeout waiting for clock HIGH during bit receive\n");
    }
    
    return bit;
}

// Receive a byte (8 bits) MSB first
unsigned char receive_byte() {
    unsigned char byte = 0;
    
    // Set data pin as input mode
    set_data_pin_direction(0);
    
    // Receive 8 bits, MSB first
    for (int i = 7; i >= 0; i--) {
        int bit = receive_bit();
        byte |= (bit << i);
    }
    
    printf("Received byte: 0x%02X\n", byte);
    return byte;
}

// Wait for initialization sequence
void wait_for_initialization() {
    int clock_pulses = 0;
    int consecutive_highs = 0;
    
    printf("Waiting for initialization sequence...\n");
    
    // Wait for a stable HIGH clock for at least 100ms
    while (consecutive_highs < 50) {  // 50*2ms = 100ms of HIGH
        if (read_clock_pin()) {
            consecutive_highs++;
        } else {
            consecutive_highs = 0;
            clock_pulses++;  // Count the pulses for reporting
        }
        delay_ms(2);
    }
    
    printf("Detected initialization sequence with %d clock pulses\n", clock_pulses);
}

// Receive a message from the Arduino
int receive_message() {
    unsigned char byte, length;
    int success = 0;
    
    // Turn on LED 0 during reception
    *LEDR_ptr |= 0x1;
    
    printf("Receiving message from Arduino...\n");
    
    // Wait for and verify start byte with retries
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
        }
    }
    
    if (!found_start) {
        printf("Failed to detect valid start byte after %d attempts\n", max_retries);
        *LEDR_ptr &= ~0x1;
        return 0;
    }
    
    // Receive length byte
    length = receive_byte();
    
    printf("Expecting message of %d bytes\n", length);
    
    // Ensure length is reasonable
    if (length >= MSG_BUFFER_SIZE || length == 0) {
        printf("Invalid message length: %d bytes\n", length);
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
    
    printf("Message received successfully: \"%s\"\n", rx_buffer);
    success = 1;
    
    // Turn off LED 0
    *LEDR_ptr &= ~0x1;
    
    // Turn on LED 1 to indicate successful reception
    if (success) {
        *LEDR_ptr |= 0x2;
        delay_ms(500);
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

// Detect sequence of clock pulses indicating reset/initialization
void detect_reset_sequence() {
    static int clock_pulse_count = 0;
    static unsigned long last_pulse_time = 0;
    unsigned long current_time = 0;  // Simulated time counter
    
    // Detect a falling edge on the clock
    if (detect_clock_edge()) {
        current_time = clock_pulse_count * 10;  // Simple counter instead of real time
        
        // If we've seen a pulse recently (within ~100ms), count it as part of a sequence
        if (current_time - last_pulse_time < 150) {
            clock_pulse_count++;
            
            // If we've seen enough pulses in sequence, consider it a reset
            if (clock_pulse_count >= 6) {
                printf("\nReset/initialization sequence detected (%d pulses)\n", 
                       clock_pulse_count);
                reset_sequence_detected = 1;
                clock_pulse_count = 0;
            }
        } else {
            // Too much time between pulses, reset counter
            clock_pulse_count = 1;
        }
        
        last_pulse_time = current_time;
    }
}

// Check for communication activity
void check_for_communication() {
    // Check for reset sequence (multiple clock pulses in a row)
    detect_reset_sequence();
    
    // If reset sequence detected, prepare for message reception
    if (reset_sequence_detected) {
        reset_sequence_detected = 0;
        
        // Wait for full initialization sequence
        wait_for_initialization();
        
        // Receive the message
        if (receive_message()) {
            printf("Communication cycle completed successfully\n");
        } else {
            printf("Communication cycle failed - message reception error\n");
        }
    }
    
    // Also detect single falling edges as potential start of communication
    if (detect_clock_edge()) {
        clock_edge_detected = 1;
    }
    
    // If we detected a single edge (but not a reset sequence)
    if (clock_edge_detected && !reset_sequence_detected) {
        clock_edge_detected = 0;
        
        // This could be the start of a message without the init sequence
        printf("\nClock activity detected - checking for message\n");
        
        // Try to receive a message
        if (receive_message()) {
            printf("Direct message reception successful\n");
        } else {
            printf("Direct message reception failed\n");
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
    printf("DE1-SoC Simple Receiver\n");
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
                printf("Manual message reception successful\n");
            } else {
                printf("Manual message reception failed\n");
            }
        }
        
        // Update old key value for edge detection
        old_key_value = key_value;
        
        // Small delay to prevent CPU hogging
        delay_ms(5);
    }
    
    return 0;
}