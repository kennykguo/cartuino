#include <stdio.h>
#include <string.h>
#include "address_map.h"

// JP1 Pin definitions
#define DATA_PIN_BIT 0x00000001  // Bit 0 (D0) for data line from Arduino D2
#define CLOCK_PIN_BIT 0x00000002 // Bit 1 (D1) for clock line from Arduino D3
#define SYNC_PIN_BIT 0x00000004  // Bit 2 (D2) for sync line from Arduino D4
#define CLOCK_RATE 100000000     // 100MHz DE1-SoC system clock

// Global variables
volatile int *JP1_ptr;      // Pointer to JP1 expansion port
volatile int *KEY_ptr;      // Pointer to pushbutton KEYs
volatile int *LEDR_ptr;     // Pointer to red LEDs

// Communication parameters
#define BIT_PERIOD_MS 5     // Fast bit transfer (5ms per bit)
#define MSG_BUFFER_SIZE 32  // Buffer size for messages
char tx_buffer[MSG_BUFFER_SIZE];
char rx_buffer[MSG_BUFFER_SIZE];

// Message counter and state tracking
int message_counter = 0;
int sync_detected = 0;
int prev_sync_state = 0;

// Protocol constants
#define START_BYTE 0xAA  // 10101010 pattern for synchronization
#define END_BYTE 0x55    // 01010101 pattern for end of message
#define CMD_ACK 0xCC     // Acknowledge receipt
#define CMD_NACK 0x33    // Negative acknowledge
#define CMD_RESPONSE 0xF0 // Response follows

// Simple delay in milliseconds
void delay_ms(int ms) {
    volatile int i;
    for (i = 0; i < ms * (CLOCK_RATE / 10000); i++);
}

// Even shorter delay in microseconds
void delay_us(int us) {
    volatile int i;
    for (i = 0; i < us * (CLOCK_RATE / 10000000); i++);
}

// Initialize the JP1 port for communication
void init_jp1_communication() {
    // Start with all pins as input
    *(JP1_ptr + 1) = 0x00000000;
    
    printf("DE1-SoC Bidirectional Communication Initialized\n");
    printf("Using D0(data), D1(clock), D2(sync) from Arduino\n");
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
    
    // Brief delay for pin state to stabilize
    delay_us(100);
}

// Set the DATA pin value (when configured as output)
void set_data_pin(int high) {
    if (high) {
        *(JP1_ptr) |= DATA_PIN_BIT;
    } else {
        *(JP1_ptr) &= ~DATA_PIN_BIT;
    }
    
    // Brief delay for value to stabilize
    delay_us(100);
}

// Read the DATA pin value
int read_data_pin() {
    return (*(JP1_ptr) & DATA_PIN_BIT) ? 1 : 0;
}

// Read the CLOCK pin value
int read_clock_pin() {
    return (*(JP1_ptr) & CLOCK_PIN_BIT) ? 1 : 0;
}

// Read the SYNC pin value
int read_sync_pin() {
    return (*(JP1_ptr) & SYNC_PIN_BIT) ? 1 : 0;
}

// Check for SYNC line rising edge
int detect_sync_edge() {
    int current_sync = read_sync_pin();
    int edge_detected = 0;
    
    if (!prev_sync_state && current_sync) {
        edge_detected = 1;
        // Wait until SYNC is stable
        int stable_count = 0;
        while (stable_count < 5) {
            delay_ms(1);
            if (read_sync_pin())
                stable_count++;
            else
                stable_count = 0;
        }
    }
    
    prev_sync_state = current_sync;
    return edge_detected;
}

// Wait for clock to be in specified state with timeout
int wait_for_clock_state(int wait_for_high, int timeout_us) {
    int i;
    const int SAMPLES_PER_US = CLOCK_RATE / 10000000;
    const int MAX_SAMPLES = timeout_us * SAMPLES_PER_US;
    
    for (i = 0; i < MAX_SAMPLES; i++) {
        int clock_state = read_clock_pin();
        if ((wait_for_high && clock_state) || (!wait_for_high && !clock_state)) {
            return 1;  // Success - clock is in desired state
        }
    }
    
    printf("Clock timeout waiting for %s (after %d us)\n", 
           wait_for_high ? "HIGH" : "LOW", timeout_us);
    return 0;  // Timeout
}

// Receive a single bit 
int receive_bit() {
    int bit;
    
    // Make sure DATA pin is set as input
    set_data_pin_direction(0);
    
    // Wait for clock to go LOW (max 10ms)
    if (!wait_for_clock_state(0, 10000)) {
        return 0;  // Default to 0 on timeout
    }
    
    // Read data bit
    bit = read_data_pin();
    
    // Wait for clock to go HIGH (max 10ms)
    wait_for_clock_state(1, 10000);
    
    return bit;
}

// Send a bit
void send_bit(int bit) {
    // Set data pin as output and set value
    set_data_pin_direction(1);
    set_data_pin(bit);
    
    // Wait for clock to go LOW
    if (!wait_for_clock_state(0, 10000)) {
        return;
    }
    
    // Keep data stable while clock is LOW
    
    // Wait for clock to go HIGH
    wait_for_clock_state(1, 10000);
}

// Receive a byte (8 bits) MSB first
unsigned char receive_byte() {
    unsigned char byte = 0;
    
    // Receive 8 bits, MSB first
    for (int i = 7; i >= 0; i--) {
        int bit = receive_bit();
        byte |= (bit << i);
    }
    
    printf("RX: 0x%02X\n", byte);
    return byte;
}

// Send a byte (8 bits) MSB first
void send_byte(unsigned char byte) {
    printf("TX: 0x%02X\n", byte);
    
    // Send each bit, MSB first
    for (int i = 7; i >= 0; i--) {
        int bit = (byte >> i) & 0x01;
        send_bit(bit);
    }
}

// Receive a message from the Arduino
int receive_message(int send_response) {
    unsigned char byte, length;
    int success = 0;
    int retries = 0;
    
    // Turn on LED 0 during reception
    *LEDR_ptr |= 0x1;
    
    printf("Preparing to receive message...\n");
    
    // Make sure data pin is in input mode
    set_data_pin_direction(0);
    
    // Pause for synchronization
    delay_ms(30);
    
    printf("Receiving message...\n");
    
    // Try to detect start byte with retries
    while (retries < 3) {
        byte = receive_byte();
    if (byte != START_BYTE) {
        printf("Invalid start byte: 0x%02X (expected 0x%02X) - retry %d\n", byte, START_BYTE, retries+1);
        retries++;
        // Short delay before retry
        delay_ms(10);
    } else {
        break; // Found valid start byte
    }
    }
    
    // Check if we found a valid start byte
    if (retries >= 3) {
        printf("Failed to find valid start byte after retries\n");
        *LEDR_ptr &= ~0x1;
        return 0;
    }
    
    // Receive length byte
    length = receive_byte();
    
    printf("Expecting %d bytes\n", length);
    
    // Ensure length is reasonable
    if (length >= MSG_BUFFER_SIZE || length == 0) {
        printf("Invalid message length: %d bytes\n", length);
        *LEDR_ptr &= ~0x1;
        
        // Send NACK
        send_byte(CMD_NACK);
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
    
    if (byte != END_BYTE) {
        printf("Invalid end byte: 0x%02X (expected 0x%02X)\n", byte, END_BYTE);
        success = 0;
        
        // Send NACK
        send_byte(CMD_NACK);
    } else {
        printf("Message received: \"%s\"\n", rx_buffer);
        success = 1;
        
        // Send response command or ACK
        if (send_response) {
            send_byte(CMD_RESPONSE);
        } else {
            send_byte(CMD_ACK);
        }
    }
    
    // Turn off LED 0
    *LEDR_ptr &= ~0x1;
    
    return success;
}

// Send a response message
void send_response() {
    // Turn on LED 1 during transmission
    *LEDR_ptr |= 0x2;
    
    // Prepare response message
    sprintf(tx_buffer, "DE1_MSG_%d", message_counter++);
    int len = strlen(tx_buffer);
    
    printf("Sending response: \"%s\"\n", tx_buffer);
    
    // Send start byte
    send_byte(START_BYTE);
    
    // Send length byte
    send_byte((unsigned char)len);
    
    // Send each byte of the message
    for (int i = 0; i < len; i++) {
        send_byte((unsigned char)tx_buffer[i]);
    }
    
    // Send end byte
    send_byte(END_BYTE);
    
    // Wait for ACK
    unsigned char response = receive_byte();
    
    if (response == CMD_ACK) {
        printf("Received ACK - response successfully delivered\n");
    } else {
        printf("Did not receive ACK - got 0x%02X instead\n", response);
    }
    
    // Turn off LED 1
    *LEDR_ptr &= ~0x2;
}

// Check for SYNC line to detect incoming communication
void check_for_sync() {
    // Monitor SYNC line for rising edge
    if (detect_sync_edge()) {
        printf("\nSync detected - preparing to receive message\n");
        sync_detected = 1;
    }
    
    // If SYNC detected, receive message and optionally send response
    if (sync_detected) {
        // Reset flag
        sync_detected = 0;
        
        // Process message with automatic response
        if (receive_message(1)) {
            // Send response message
            send_response();
            printf("Bidirectional exchange completed\n");
        } else {
            printf("Failed to receive message\n");
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
    printf("DE1-SoC Fast Bidirectional Communication\n");
    printf("===================================\n");
    
    // Initialize JP1 for communication
    init_jp1_communication();
    
    // Main loop variables
    int key_value, old_key_value = 0;
    
    // Initial LED state
    *LEDR_ptr = 0;
    
    printf("Running in automatic handshake mode\n");
    printf("KEY0 can still be used to manually send a message\n");
    printf("===================================\n\n");
    
    // Main loop
    while (1) {
        // Read key value for edge detection
        key_value = *KEY_ptr;
        
        // Continuously check for SYNC signal
        check_for_sync();
        
        // KEY0: Manually initiate message transmission
        if ((key_value & 0x1) && !(old_key_value & 0x1)) {
            printf("\nKEY0 pressed - Sending message\n");
            
            // Send a test message
            sprintf(tx_buffer, "DE1_MSG_%d", message_counter++);
            int len = strlen(tx_buffer);
            
            // Turn on LED 2 during transmission
            *LEDR_ptr |= 0x4;
            
            printf("Sending message: \"%s\"\n", tx_buffer);
            
            // Send start byte
            send_byte(START_BYTE);
            
            // Send length byte
            send_byte((unsigned char)len);
            
            // Send each byte of the message
            for (int i = 0; i < len; i++) {
                send_byte((unsigned char)tx_buffer[i]);
            }
            
            // Send end byte
            send_byte(END_BYTE);
            
            // Turn off LED 2
            *LEDR_ptr &= ~0x4;
            
            printf("Message sent\n");
        }
        
        // Update old key value for edge detection
        old_key_value = key_value;
        
        // Small delay to prevent CPU hogging
        delay_ms(1);
    }
    
    return 0;
}