// DE1-SoC Asynchronous Communication Protocol
// Uses software UART-like protocol with single data line

#include <stdio.h>
#include <string.h>
#include "address_map.h"

// JP1 Pin definitions
#define DATA_PIN_BIT 0x00000001  // Bit 0 (D0) for data line
#define CLOCK_RATE 100000000     // 100MHz DE1-SoC system clock

// Global variables
volatile int *JP1_ptr;      // Pointer to JP1 expansion port
volatile int *KEY_ptr;      // Pointer to pushbutton KEYs
volatile int *SW_ptr;       // Pointer to slider switches
volatile int *LEDR_ptr;     // Pointer to red LEDs
volatile int *TIMER_ptr;    // Pointer to interval timer

// Communication parameters
#define BIT_PERIOD_MS 2     // 2ms per bit (500 bps) - slow but reliable
#define MSG_BUFFER_SIZE 64  // Buffer size for messages
char tx_buffer[MSG_BUFFER_SIZE];
char rx_buffer[MSG_BUFFER_SIZE];
int rx_buffer_pos = 0;

// Message counter
int message_counter = 0;

// Protocol constants
#define START_SEQUENCE 0xAA // 10101010 pattern for synchronization
#define END_SEQUENCE 0x55   // 01010101 pattern for end of message
#define ACK_BYTE 0xCC       // 11001100 pattern for acknowledgment
#define NACK_BYTE 0x33      // 00110011 pattern for negative acknowledgment

// Timeout constants
#define MAX_WAIT_TIME_MS 1000

// Simple delay in milliseconds
void delay_ms(int ms) {
    volatile int i;
    for (i = 0; i < ms * (CLOCK_RATE / 10000); i++);
}

// Initialize the JP1 port for communication
void init_jp1_communication() {
    // Set DATA pin as input by default
    *(JP1_ptr + 1) &= ~DATA_PIN_BIT;  // Direction register
    
    // Add delay for stability
    delay_ms(10);
    
    printf("JP1 communication initialized\n");
    printf("DATA_PIN_BIT = 0x%08X (D0)\n", DATA_PIN_BIT);
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

// Send a single bit - hold for BIT_PERIOD_MS
void send_bit(int bit) {
    // Set the data value and hold for the full bit period
    set_data_pin(bit);
    delay_ms(BIT_PERIOD_MS);
}

// Send a byte (8 bits) LSB first, with start and stop bits
void send_byte(unsigned char byte) {
    // Set DATA pin as output
    set_data_pin_direction(1);
    
    printf("Sending byte: 0x%02X\n", byte);
    
    // Send start bit (always 0)
    send_bit(0);
    
    // Send 8 data bits, LSB first
    for (int i = 0; i < 8; i++) {
        int bit = (byte >> i) & 0x01;
        send_bit(bit);
    }
    
    // Send stop bit (always 1)
    send_bit(1);
    
    // Add extra idle time between bytes for stability
    delay_ms(BIT_PERIOD_MS);
}

// Receive a byte (8 bits) LSB first, with start and stop bits
// Returns 0xFF on error
unsigned char receive_byte() {
    unsigned char byte = 0;
    int bit;
    int timeout_count = 0;
    
    // Set DATA pin as input
    set_data_pin_direction(0);
    
    // Wait for start bit (logic 0)
    while (read_data_pin() == 1) {
        delay_ms(1);
        timeout_count++;
        if (timeout_count > MAX_WAIT_TIME_MS) {
            printf("Timeout waiting for start bit\n");
            return 0xFF; // Indicate error
        }
    }
    
    // Start bit detected, wait for half bit period to sample in the middle
    delay_ms(BIT_PERIOD_MS / 2);
    
    // Verify it's still a valid start bit
    if (read_data_pin() != 0) {
        printf("Invalid start bit\n");
        return 0xFF; // Indicate error
    }
    
    // Wait for the remainder of the start bit
    delay_ms(BIT_PERIOD_MS / 2);
    
    // Read 8 data bits
    for (int i = 0; i < 8; i++) {
        // Wait for half bit period to sample in the middle
        delay_ms(BIT_PERIOD_MS / 2);
        
        // Read bit
        bit = read_data_pin();
        if (bit) {
            byte |= (1 << i);
        }
        
        // Wait for remainder of bit period
        delay_ms(BIT_PERIOD_MS / 2);
    }
    
    // Wait for half bit period to check stop bit in the middle
    delay_ms(BIT_PERIOD_MS / 2);
    
    // Check for valid stop bit (logic 1)
    if (read_data_pin() != 1) {
        printf("Invalid stop bit\n");
        return 0xFF; // Indicate error
    }
    
    // Wait for remainder of stop bit
    delay_ms(BIT_PERIOD_MS / 2);
    
    printf("Received byte: 0x%02X\n", byte);
    return byte;
}

// Send a message with protocol framing
int send_message(char *message) {
    int len = strlen(message);
    unsigned char checksum = 0;
    
    // Turn on LED 0 during transmission
    *LEDR_ptr |= 0x1;
    
    printf("Sending message: \"%s\" (%d bytes)\n", message, len);
    
    // 1. Send start sequence for synchronization
    send_byte(START_SEQUENCE);
    
    // 2. Send length byte
    send_byte((unsigned char)len);
    checksum ^= len; // XOR for simple checksum
    
    // 3. Send each byte of the message
    for (int i = 0; i < len; i++) {
        send_byte((unsigned char)message[i]);
        checksum ^= message[i]; // Update checksum
    }
    
    // 4. Send checksum
    send_byte(checksum);
    
    // 5. Send end sequence
    send_byte(END_SEQUENCE);
    
    // 6. Wait for ACK/NACK from receiver
    unsigned char response = receive_byte();
    
    // Turn off LED 0
    *LEDR_ptr &= ~0x1;
    
    if (response == ACK_BYTE) {
        printf("Received ACK - message successfully delivered\n");
        return 1;
    } else if (response == 0xFF) {
        printf("Error or timeout waiting for acknowledgment\n");
        return 0;
    } else {
        printf("Did not receive ACK - got 0x%02X instead\n", response);
        return 0;
    }
}

// Receive a message with protocol framing
int receive_message() {
    unsigned char byte, length, checksum = 0, calculated_checksum = 0;
    int i, success = 0;
    
    // Turn on LED 2 during reception
    *LEDR_ptr |= 0x4;
    
    printf("Waiting for message...\n");
    
    // 1. Wait for and verify start sequence
    byte = receive_byte();
    if (byte == 0xFF) {
        printf("Error receiving start byte\n");
        *LEDR_ptr &= ~0x4; // Turn off LED 2
        return 0;
    }
    
    if (byte != START_SEQUENCE) {
        printf("Invalid start sequence: 0x%02X, expected 0x%02X\n", byte, START_SEQUENCE);
        *LEDR_ptr &= ~0x4; // Turn off LED 2
        return 0;
    }
    
    // 2. Receive length byte
    length = receive_byte();
    if (length == 0xFF) {
        printf("Error receiving length byte\n");
        *LEDR_ptr &= ~0x4; // Turn off LED 2
        return 0;
    }
    calculated_checksum ^= length;
    
    printf("Expecting message of %d bytes\n", length);
    
    // Ensure length is reasonable
    if (length >= MSG_BUFFER_SIZE) {
        printf("Message too long: %d bytes\n", length);
        *LEDR_ptr &= ~0x4; // Turn off LED 2
        return 0;
    }
    
    // 3. Receive each byte of the message
    rx_buffer_pos = 0;
    for (i = 0; i < length; i++) {
        byte = receive_byte();
        if (byte == 0xFF) {
            printf("Error receiving data byte %d\n", i);
            *LEDR_ptr &= ~0x4; // Turn off LED 2
            return 0;
        }
        rx_buffer[rx_buffer_pos++] = byte;
        calculated_checksum ^= byte;
    }
    
    // Null-terminate the string
    rx_buffer[rx_buffer_pos] = '\0';
    
    // 4. Receive checksum
    checksum = receive_byte();
    if (checksum == 0xFF) {
        printf("Error receiving checksum byte\n");
        *LEDR_ptr &= ~0x4; // Turn off LED 2
        return 0;
    }
    
    // 5. Receive end sequence
    byte = receive_byte();
    if (byte == 0xFF) {
        printf("Error receiving end sequence byte\n");
        *LEDR_ptr &= ~0x4; // Turn off LED 2
        return 0;
    }
    
    if (byte != END_SEQUENCE) {
        printf("Invalid end sequence: 0x%02X, expected 0x%02X\n", byte, END_SEQUENCE);
        success = 0;
    } else if (checksum != calculated_checksum) {
        printf("Checksum error: received 0x%02X, calculated 0x%02X\n", checksum, calculated_checksum);
        success = 0;
    } else {
        printf("Message received successfully: \"%s\"\n", rx_buffer);
        success = 1;
    }
    
    // 6. Send ACK or NACK based on success
    set_data_pin_direction(1); // Switch to output mode for ACK/NACK
    if (success) {
        send_byte(ACK_BYTE);
    } else {
        send_byte(NACK_BYTE);
    }
    
    // Turn off LED 2
    *LEDR_ptr &= ~0x4;
    
    return success;
}

// Check if start bit detected (polling for incoming data)
int check_for_start_bit() {
    // Set DATA pin as input
    set_data_pin_direction(0);
    
    // Check if data line is low (potential start bit)
    return (read_data_pin() == 0);
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
    printf("DE1-SoC Asynchronous Serial Communication Started\n");
    printf("===================================\n");
    
    // Initialize JP1 for communication
    init_jp1_communication();
    
    // Main loop variables
    int key_value, old_key_value = 0;
    int sw_value;
    
    printf("Test Controls:\n");
    printf("- KEY0: Send test message to Arduino\n");
    printf("- KEY1: Force receive message from Arduino\n");
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
        
        // Check for incoming data (non-blocking)
        if (check_for_start_bit()) {
            if (receive_message()) {
                printf("Received message from Arduino: %s\n", rx_buffer);
                
                // Auto-reply with acknowledgment
                sprintf(tx_buffer, "DE1_ACK_%d", message_counter++);
                delay_ms(100); // Small delay before responding
                send_message(tx_buffer);
            }
        }
        
        // KEY0: Send message to Arduino
        if ((key_value & 0x1) && !(old_key_value & 0x1)) {
            printf("\nKEY0 pressed - Sending test message\n");
            
            // Create test message based on switch setting
            if (sw_value & 0x1) {
                sprintf(tx_buffer, "DE1_MSG_%d_EXTENDED", message_counter++);
            } else {
                sprintf(tx_buffer, "DE1_MSG_%d", message_counter++);
            }
            
            send_message(tx_buffer);
        }
        
        // KEY1: Force receive message from Arduino
        if ((key_value & 0x2) && !(old_key_value & 0x2)) {
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