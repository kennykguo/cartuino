// DE1-SoC Simple Asynchronous Communication Protocol
// For communication with Arduino Feather 32u4

#include <stdio.h>
#include <string.h>
#include "address_map.h"

// Pin definitions - Using JP1 expansion port
#define DATA_PIN_BIT 0x00000001  // Bit 0 (D0) for data line
#define HANDSHAKE_PIN_BIT 0x00000002 // Bit 1 (D1) for handshake line
#define CLOCK_RATE 100000000     // 100MHz DE1-SoC system clock

// Global variables
volatile int *JP1_ptr;      // Pointer to JP1 expansion port
volatile int *KEY_ptr;      // Pointer to pushbutton KEYs
volatile int *SW_ptr;       // Pointer to slider switches
volatile int *LEDR_ptr;     // Pointer to red LEDs
volatile int *TIMER_ptr;    // Pointer to interval timer

// Communication parameters
#define BIT_DELAY_MS 50     // Bit transmission delay (slower for reliability)
#define START_BITS 2        // Number of start bits (1-1)
#define STOP_BITS 2         // Number of stop bits (0-0)
#define TIMEOUT_MS 5000     // Timeout in milliseconds
#define MSG_BUFFER_SIZE 64  // Buffer size for messages

char tx_buffer[MSG_BUFFER_SIZE];
char rx_buffer[MSG_BUFFER_SIZE];
int message_counter = 0;

// Simple delay in milliseconds
void delay_ms(int ms) {
    volatile int i;
    for (i = 0; i < ms * (CLOCK_RATE / 10000); i++);
}

// Initialize the JP1 port for communication
void init_jp1_communication() {
    // Set both pins as inputs initially
    *(JP1_ptr + 1) = 0x00000000;
    
    // Add delay for stability
    delay_ms(10);
    
    printf("JP1 communication initialized\n");
    printf("DATA_PIN_BIT = 0x%08X (D0), HANDSHAKE_PIN_BIT = 0x%08X (D1)\n", 
           DATA_PIN_BIT, HANDSHAKE_PIN_BIT);
}

// Set pin direction (input = 0, output = 1)
void set_pin_direction(int pin_bit, int is_output) {
    if (is_output) {
        *(JP1_ptr + 1) |= pin_bit;
    } else {
        *(JP1_ptr + 1) &= ~pin_bit;
    }
}

// Set pin value (when configured as output)
void set_pin(int pin_bit, int high) {
    if (high) {
        *(JP1_ptr) |= pin_bit;
    } else {
        *(JP1_ptr) &= ~pin_bit;
    }
}

// Read pin value
int read_pin(int pin_bit) {
    return (*(JP1_ptr) & pin_bit) ? 1 : 0;
}

// Wait for handshake pin to reach desired state with timeout
// Returns 1 on success, 0 on timeout
int wait_for_handshake(int desired_state, int timeout_ms) {
    int start_time = 0;
    
    while (read_pin(HANDSHAKE_PIN_BIT) != desired_state) {
        delay_ms(1);
        start_time++;
        if (start_time >= timeout_ms) {
            printf("Handshake timeout waiting for %d state\n", desired_state);
            return 0;
        }
    }
    return 1;
}

// Send a single bit
void send_bit(int bit) {
    // Set the data pin to the bit value
    set_pin(DATA_PIN_BIT, bit);
    
    // Signal that data is ready by setting handshake high
    set_pin(HANDSHAKE_PIN_BIT, 1);
    
    // Wait for receiver to acknowledge by setting their handshake pin high
    if (!wait_for_handshake(1, TIMEOUT_MS)) {
        printf("Timeout waiting for bit acknowledgment\n");
        return;
    }
    
    // Reset handshake line to low to complete the bit transmission
    set_pin(HANDSHAKE_PIN_BIT, 0);
    
    // Wait for receiver to reset their handshake pin
    if (!wait_for_handshake(0, TIMEOUT_MS)) {
        printf("Timeout waiting for bit completion\n");
        return;
    }
    
    // Small delay before next bit
    delay_ms(BIT_DELAY_MS);
}

// Receive a single bit
int receive_bit() {
    int bit;
    
    // Wait for sender to signal data is ready
    if (!wait_for_handshake(1, TIMEOUT_MS)) {
        printf("Timeout waiting for bit start\n");
        return -1; // Error condition
    }
    
    // Read the data bit
    bit = read_pin(DATA_PIN_BIT);
    
    // Acknowledge receipt by setting handshake high
    set_pin(HANDSHAKE_PIN_BIT, 1);
    
    // Wait for sender to reset their handshake
    if (!wait_for_handshake(0, TIMEOUT_MS)) {
        printf("Timeout waiting for bit reset\n");
        return -1; // Error condition
    }
    
    // Reset our handshake line
    set_pin(HANDSHAKE_PIN_BIT, 0);
    
    return bit;
}

// Send a byte, LSB first with start and stop bits
void send_byte(unsigned char byte) {
    // Set pins as outputs
    set_pin_direction(DATA_PIN_BIT, 1);
    set_pin_direction(HANDSHAKE_PIN_BIT, 1);
    
    printf("Sending byte: 0x%02X\n", byte);
    
    // Send start bits (1-1)
    for (int i = 0; i < START_BITS; i++) {
        send_bit(1);
    }
    
    // Send 8 data bits, LSB first
    for (int i = 0; i < 8; i++) {
        int bit = (byte >> i) & 0x01;
        send_bit(bit);
    }
    
    // Send stop bits (0-0)
    for (int i = 0; i < STOP_BITS; i++) {
        send_bit(0);
    }
}

// Receive a byte with start and stop bits
// Returns -1 on error
int receive_byte() {
    int start_bits[START_BITS];
    int data_bits[8];
    int stop_bits[STOP_BITS];
    unsigned char byte = 0;
    
    // Set data pin as input, handshake as output
    set_pin_direction(DATA_PIN_BIT, 0);
    set_pin_direction(HANDSHAKE_PIN_BIT, 1);
    set_pin(HANDSHAKE_PIN_BIT, 0);  // Initialize to low
    
    // Read start bits
    for (int i = 0; i < START_BITS; i++) {
        start_bits[i] = receive_bit();
        if (start_bits[i] != 1) {
            printf("Invalid start bit %d: %d\n", i, start_bits[i]);
            return -1;
        }
    }
    
    // Read 8 data bits, LSB first
    for (int i = 0; i < 8; i++) {
        data_bits[i] = receive_bit();
        if (data_bits[i] < 0) {
            printf("Error receiving data bit %d\n", i);
            return -1;
        }
        byte |= (data_bits[i] << i);
    }
    
    // Read stop bits
    for (int i = 0; i < STOP_BITS; i++) {
        stop_bits[i] = receive_bit();
        if (stop_bits[i] != 0) {
            printf("Invalid stop bit %d: %d\n", i, stop_bits[i]);
            return -1;
        }
    }
    
    printf("Received byte: 0x%02X\n", byte);
    return byte;
}

// Calculate simple checksum
unsigned char calculate_checksum(char *data, int length) {
    unsigned char checksum = 0;
    for (int i = 0; i < length; i++) {
        checksum ^= data[i];  // XOR checksum
    }
    return checksum;
}

// Send a message
void send_message(char *message) {
    int len = strlen(message);
    unsigned char checksum = calculate_checksum(message, len);
    
    // Turn on LED 0 during transmission
    *LEDR_ptr |= 0x1;
    
    printf("Sending message: \"%s\" (%d bytes)\n", message, len);
    
    // Send length byte
    send_byte((unsigned char)len);
    
    // Send each character
    for (int i = 0; i < len; i++) {
        send_byte((unsigned char)message[i]);
    }
    
    // Send checksum
    send_byte(checksum);
    
    // Wait for acknowledgment (ACK/NACK)
    int response = receive_byte();
    if (response == 0xCC) {  // ACK
        printf("Message successfully delivered (ACK received)\n");
    } else {
        printf("Message delivery failed (NACK or invalid response: 0x%02X)\n", response);
    }
    
    // Turn off LED 0
    *LEDR_ptr &= ~0x1;
}

// Receive a message
int receive_message() {
    unsigned char length, checksum, calculated_checksum;
    int i, success = 0;
    
    // Turn on LED 2 during reception
    *LEDR_ptr |= 0x4;
    
    printf("Waiting to receive message...\n");
    
    // Get length byte
    int len_byte = receive_byte();
    if (len_byte < 0 || len_byte >= MSG_BUFFER_SIZE) {
        printf("Invalid message length: %d\n", len_byte);
        send_byte(0x33);  // NACK
        *LEDR_ptr &= ~0x4;
        return 0;
    }
    
    length = (unsigned char)len_byte;
    printf("Expecting message of %d bytes\n", length);
    
    // Get message data
    for (i = 0; i < length; i++) {
        int byte = receive_byte();
        if (byte < 0) {
            printf("Error receiving message data\n");
            send_byte(0x33);  // NACK
            *LEDR_ptr &= ~0x4;
            return 0;
        }
        rx_buffer[i] = (char)byte;
    }
    
    // Null-terminate the string
    rx_buffer[length] = '\0';
    
    // Get checksum
    int checksum_byte = receive_byte();
    if (checksum_byte < 0) {
        printf("Error receiving checksum\n");
        send_byte(0x33);  // NACK
        *LEDR_ptr &= ~0x4;
        return 0;
    }
    
    checksum = (unsigned char)checksum_byte;
    calculated_checksum = calculate_checksum(rx_buffer, length);
    
    if (checksum != calculated_checksum) {
        printf("Checksum error: received 0x%02X, calculated 0x%02X\n", 
               checksum, calculated_checksum);
        send_byte(0x33);  // NACK
        success = 0;
    } else {
        printf("Message received successfully: \"%s\"\n", rx_buffer);
        send_byte(0xCC);  // ACK
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
    printf("DE1-SoC Asynchronous Communication Started\n");
    printf("===================================\n");
    
    // Initialize JP1 for communication
    init_jp1_communication();
    
    // Main loop variables
    int key_value, old_key_value = 0;
    int sw_value;
    
    printf("Controls:\n");
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
        
        // KEY0: Send message to Arduino (on press)
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
        
        // KEY1: Receive message from Arduino (on press)
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