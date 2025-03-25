#include <stdio.h>
#include <string.h>
#include "address_map.h"

// JP1 Pin definitions
#define DATA_PIN_BIT 0x00000001  // Bit 0 (D0) for data line from Arduino D2
#define CLOCK_PIN_BIT 0x00000002 // Bit 1 (D1) for clock line from Arduino D3
#define READY_PIN_BIT 0x00000004 // Bit 2 (D2) for ready/request line from Arduino D4
#define CLOCK_RATE 100000000     // 100MHz DE1-SoC system clock

// Global variables
volatile int *JP1_ptr;      // Pointer to JP1 expansion port
volatile int *KEY_ptr;      // Pointer to pushbutton KEYs
volatile int *LEDR_ptr;     // Pointer to red LEDs

// Communication parameters
#define BIT_PERIOD_US 5000  // 5ms per bit (in microseconds)
#define MSG_BUFFER_SIZE 32  // Buffer size for messages
char tx_buffer[MSG_BUFFER_SIZE];
char rx_buffer[MSG_BUFFER_SIZE];

// Message counter and state
int message_counter = 0;
int ready_state = 0;        // Tracked state of ready line

// Protocol constants
#define START_BYTE 0xAA  // 10101010 pattern for synchronization
#define END_BYTE 0x55    // 01010101 pattern for end of message
#define ACK_BYTE 0xCC    // 11001100 pattern for acknowledgment

// Simple delay in microseconds (more precise than milliseconds)
void delay_us(int us) {
    volatile int i;
    // Calibrated for microsecond precision
    for (i = 0; i < us * (CLOCK_RATE / 10000000); i++);
}

// Initialize the JP1 port for communication
void init_jp1_communication() {
    // Set pins as input initially (slave mode)
    *(JP1_ptr + 1) = 0x00000000;
    
    printf("DE1-SoC Slave Communication Initialized\n");
    printf("Using D0 (data), D1 (clock), D2 (ready/request)\n");
    printf("JP1 direction register value: 0x%08X\n", *(JP1_ptr + 1));
    printf("JP1 data register value: 0x%08X\n", *(JP1_ptr));
}

// Set the direction of a specific pin
void set_pin_direction(int pin_bit, int is_output) {
    if (is_output) {
        *(JP1_ptr + 1) |= pin_bit;   // Set as output
    } else {
        *(JP1_ptr + 1) &= ~pin_bit;  // Set as input
    }
    delay_us(10);  // Brief delay for stability
}

// Set a pin's value (when configured as output)
void set_pin_value(int pin_bit, int high) {
    if (high) {
        *(JP1_ptr) |= pin_bit;       // Set high
    } else {
        *(JP1_ptr) &= ~pin_bit;      // Set low
    }
    delay_us(10);  // Brief delay for stability
}

// Read a pin's value
int read_pin_value(int pin_bit) {
    return (*(JP1_ptr) & pin_bit) ? 1 : 0;
}

// Check if Arduino is requesting our attention
int arduino_requesting_attention() {
    // Set READY pin as input to read Arduino's state
    set_pin_direction(READY_PIN_BIT, 0);
    
    // If Arduino has READY_PIN HIGH, it's requesting attention
    return read_pin_value(READY_PIN_BIT);
}

// Acknowledge Arduino's request by pulling READY line LOW
void acknowledge_request() {
    // Set READY pin as output to acknowledge
    set_pin_direction(READY_PIN_BIT, 1);
    set_pin_value(READY_PIN_BIT, 0);
    delay_us(10);
}

// Reset READY line to normal state
void reset_ready_line() {
    set_pin_direction(READY_PIN_BIT, 0); // Set as input
    delay_us(10);
}

// Request attention from Arduino
int request_attention(int timeout_ms) {
    int i;
    const int POLL_INTERVAL_US = 100;
    int max_iterations = (timeout_ms * 1000) / POLL_INTERVAL_US;
    
    // Set READY pin as output and pull it LOW to request attention
    set_pin_direction(READY_PIN_BIT, 1);
    set_pin_value(READY_PIN_BIT, 0);
    
    // Wait for Arduino to acknowledge by setting READY pin as INPUT_PULLUP
    // which will cause the pin to read as HIGH
    for (i = 0; i < max_iterations; i++) {
        if (read_pin_value(READY_PIN_BIT)) {
            // Arduino has acknowledged
            return 1; // Success
        }
        delay_us(POLL_INTERVAL_US);
    }
    
    printf("Timeout waiting for Arduino to acknowledge\n");
    reset_ready_line();
    return 0; // Timeout
}

// Send a single bit
void send_bit(int bit) {
    // Set DATA pin as output
    set_pin_direction(DATA_PIN_BIT, 1);
    
    // Set data bit
    set_pin_value(DATA_PIN_BIT, bit);
    
    // Brief delay for data to stabilize
    delay_us(200);
    
    // Pull clock LOW to indicate bit is ready
    set_pin_value(CLOCK_PIN_BIT, 0);
    delay_us(BIT_PERIOD_US / 2);
    
    // Pull clock HIGH to complete bit transmission
    set_pin_value(CLOCK_PIN_BIT, 1);
    delay_us(BIT_PERIOD_US / 2);
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

// Receive a single bit
int receive_bit() {
    int bit;
    
    // Set DATA pin as input
    set_pin_direction(DATA_PIN_BIT, 0);
    
    // Wait for clock to go LOW (Arduino signals bit is ready)
    while (read_pin_value(CLOCK_PIN_BIT) != 0);
    
    // Brief delay for data to stabilize
    delay_us(100);
    
    // Read data bit
    bit = read_pin_value(DATA_PIN_BIT);
    
    // Wait for clock to go HIGH (bit transmission complete)
    while (read_pin_value(CLOCK_PIN_BIT) == 0);
    
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
    
    printf("RX: 0x%02X\n", byte);
    return byte;
}

// Send a complete message to Arduino
int send_message(char *message) {
    int len = strlen(message);
    unsigned char checksum = 0;
    int success = 0;
    
    // Turn on LED 0 during transmission
    *LEDR_ptr |= 0x1;
    
    printf("Sending message: \"%s\" (%d bytes)\n", message, len);
    
    // Request attention from Arduino
    if (!request_attention(500)) {
        *LEDR_ptr &= ~0x1; // Turn off LED
        return 0;
    }
    
    // Set CLOCK pin as output
    set_pin_direction(CLOCK_PIN_BIT, 1);
    set_pin_value(CLOCK_PIN_BIT, 1); // Start with clock HIGH
    
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
    
    // Wait for ACK from Arduino
    set_pin_direction(DATA_PIN_BIT, 0); // Set as input
    unsigned char response = receive_byte();
    
    if (response == ACK_BYTE) {
        printf("Received ACK - message successfully delivered\n");
        success = 1;
    } else {
        printf("Did not receive ACK - got 0x%02X instead\n", response);
        success = 0;
    }
    
    // Reset ready line
    reset_ready_line();
    
    // Turn off LED 0
    *LEDR_ptr &= ~0x1;
    
    return success;
}

// Receive a complete message from Arduino
int receive_message() {
    unsigned char byte, length, checksum = 0, calculated_checksum = 0;
    int success = 0;
    
    // Turn on LED 1 during reception
    *LEDR_ptr |= 0x2;
    
    printf("Receiving message from Arduino...\n");
    
    // Acknowledge Arduino's request
    acknowledge_request();
    
    // Wait for start byte
    byte = receive_byte();
    if (byte != START_BYTE) {
        printf("Invalid start byte: 0x%02X (expected 0x%02X)\n", byte, START_BYTE);
        reset_ready_line();
        *LEDR_ptr &= ~0x2; // Turn off LED
        return 0;
    }
    
    // Receive length byte
    length = receive_byte();
    calculated_checksum ^= length;
    
    printf("Expecting message of %d bytes\n", length);
    
    // Validate length
    if (length >= MSG_BUFFER_SIZE || length == 0) {
        printf("Invalid message length: %d\n", length);
        reset_ready_line();
        *LEDR_ptr &= ~0x2; // Turn off LED
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
    
    // Verify message
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
    
    // Send ACK or NACK
    set_pin_direction(DATA_PIN_BIT, 1); // Set as output
    send_byte(success ? ACK_BYTE : 0x33);
    
    // Reset ready line
    reset_ready_line();
    
    // Turn off LED 1
    *LEDR_ptr &= ~0x2;
    
    return success;
}

// Check if Arduino is requesting to send us data
void check_for_arduino_communication() {
    if (arduino_requesting_attention()) {
        printf("\nArduino requesting attention - receiving message\n");
        if (receive_message()) {
            printf("Communication cycle completed successfully\n");
            
            // Turn on LED 3 briefly to indicate success
            *LEDR_ptr |= 0x8;
            delay_us(100000); // 100ms
            *LEDR_ptr &= ~0x8;
        } else {
            printf("Communication cycle failed\n");
            
            // Flash LED 4 to indicate error
            for (int i = 0; i < 3; i++) {
                *LEDR_ptr |= 0x10;
                delay_us(50000);
                *LEDR_ptr &= ~0x10;
                delay_us(50000);
            }
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
    printf("DE1-SoC Bidirectional Communication\n");
    printf("===================================\n");
    
    // Initialize JP1 for communication
    init_jp1_communication();
    
    // Main loop variables
    int key_value, old_key_value = 0;
    
    // Initial LED state
    *LEDR_ptr = 0;
    
    printf("Operating in bidirectional mode with handshake\n");
    printf("KEY0: Send test message to Arduino\n");
    printf("===================================\n\n");
    
    // Set CLOCK pin as input initially (Arduino is master for clock)
    set_pin_direction(CLOCK_PIN_BIT, 0);
    
    // Main loop
    while (1) {
        // Read key value for edge detection
        key_value = *KEY_ptr;
        
        // Check for incoming communication from Arduino
        check_for_arduino_communication();
        
        // KEY0: Manual trigger for message sending
        if ((key_value & 0x1) && !(old_key_value & 0x1)) {
            printf("\nKEY0 pressed - Sending test message to Arduino\n");
            
            // Prepare a test message
            sprintf(tx_buffer, "DE1_MSG_%d", message_counter++);
            
            // Send the message
            if (send_message(tx_buffer)) {
                printf("Manual communication cycle completed successfully\n");
            } else {
                printf("Manual communication cycle failed\n");
            }
        }
        
        // Update old key value for edge detection
        old_key_value = key_value;
        
        // Small delay to prevent CPU hogging
        delay_us(1000); // 1ms polling interval
    }
    
    return 0;
}