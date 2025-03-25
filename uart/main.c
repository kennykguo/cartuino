#include <stdio.h>
#include <string.h>
#include "address_map.h"

// Pin definitions
#define DATA_PIN_BIT 0x00000001  // Bit 0 (D0) for data from Arduino D2
#define CLOCK_PIN_BIT 0x00000002 // Bit 1 (D1) for clock from Arduino D3
#define SYNC_PIN_BIT 0x00000004  // Bit 2 (D2) for sync from Arduino D5
#define CLOCK_RATE 100000000     // 100MHz DE1-SoC system clock

// Global variables
volatile int *JP1_ptr;      // Pointer to JP1 expansion port
volatile int *KEY_ptr;      // Pointer to pushbutton KEYs
volatile int *LEDR_ptr;     // Pointer to red LEDs

// Communication parameters
#define BIT_PERIOD_MS 5     // Match Arduino's timing
#define MSG_BUFFER_SIZE 32  // Buffer size for messages
#define START_BYTE 0xAA     // 10101010 pattern for synchronization

// Buffers and counters
char rx_buffer[MSG_BUFFER_SIZE];
char tx_buffer[MSG_BUFFER_SIZE];
int message_counter = 0;

// Simple delay in milliseconds
void delay_ms(int ms) {
    volatile int i;
    for (i = 0; i < ms * (CLOCK_RATE / 100000); i++);
}

// Brief microsecond delay
void delay_us(int us) {
    volatile int i;
    for (i = 0; i < us * (CLOCK_RATE / 1000000000); i++);
}

// Initialize the JP1 port for communication
void init_jp1_communication() {
    // Set all pins as input initially
    *(JP1_ptr + 1) = 0x00000000;
    
    printf("DE1-SoC Fast Bidirectional Protocol\n");
    printf("Using D0(data), D1(clock), D2(sync) from Arduino\n");
    printf("JP1 direction: 0x%08X, data: 0x%08X\n", 
           *(JP1_ptr + 1), *(JP1_ptr));
}

// Read pin values
int read_data_pin() {
    return (*(JP1_ptr) & DATA_PIN_BIT) ? 1 : 0;
}

int read_clock_pin() {
    return (*(JP1_ptr) & CLOCK_PIN_BIT) ? 1 : 0;
}

int read_sync_pin() {
    return (*(JP1_ptr) & SYNC_PIN_BIT) ? 1 : 0;
}

// Set data pin direction and value
void set_data_pin_direction(int is_output) {
    if (is_output) {
        *(JP1_ptr + 1) |= DATA_PIN_BIT;
    } else {
        *(JP1_ptr + 1) &= ~DATA_PIN_BIT;
    }
    delay_us(500);
}

void set_data_pin(int high) {
    if (high) {
        *(JP1_ptr) |= DATA_PIN_BIT;
    } else {
        *(JP1_ptr) &= ~DATA_PIN_BIT;
    }
    delay_us(500);
}

// Wait for clock transition (optimized for speed)
int wait_for_clock_state(int desired_state, int timeout_ms) {
    int i;
    const int SAMPLES_PER_MS = 100;
    const int MAX_SAMPLES = timeout_ms * SAMPLES_PER_MS;
    
    for (i = 0; i < MAX_SAMPLES; i++) {
        int clock_state = read_clock_pin();
        if ((desired_state && clock_state) || (!desired_state && !clock_state)) {
            return 1; // Success
        }
        delay_us(10);
    }
    
    printf("TIMEOUT waiting for CLOCK=%d\n", desired_state);
    return 0;
}

// Receive a single bit
int receive_bit() {
    int bit;
    
    // Make sure DATA pin is set as input
    set_data_pin_direction(0);
    
    // Wait for clock HIGH (initial state)
    if (!wait_for_clock_state(1, 10)) {
        return 0;
    }
    
    // Wait for clock LOW (active edge)
    if (!wait_for_clock_state(0, 10)) {
        return 0;
    }
    
    // Read the data bit while clock is LOW
    bit = read_data_pin();
    
    // Wait for clock to return HIGH
    wait_for_clock_state(1, 10);
    
    return bit;
}

// Send a single bit with retry
void send_bit(int bit) {
    int retries = 3;
    
    // Set DATA pin as output with the bit value
    set_data_pin_direction(1);
    set_data_pin(bit);
    
    // Try multiple times to wait for clock
    while (retries > 0) {
        // Wait for clock to go LOW with shorter timeout
        if (wait_for_clock_state(0, 10)) {
            // Success! Wait for clock HIGH to complete bit
            wait_for_clock_state(1, 10);
            return;
        }
        
        // Clock didn't go LOW, retry
        retries--;
        
        // Briefly pulse data line to get Arduino's attention
        if (retries > 0) {
            set_data_pin(!bit);
            delay_us(500);
            set_data_pin(bit);
            delay_us(500);
        }
    }
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

// Wait for and verify distinct preamble pattern
int wait_for_start_pattern() {
    unsigned char bytes[4];
    printf("Looking for preamble pattern...\n");
    
    // Collect up to 10 bytes looking for the pattern
    for (int i = 0; i < 10; i++) {
        // Shift the array to make room for new byte
        for (int j = 0; j < 3; j++) {
            bytes[j] = bytes[j+1];
        }
        
        // Read new byte
        bytes[3] = receive_byte();
        
        // Check for distinctive pattern: 0xAA, 0x55, 0xAA, 0xAA
        if (bytes[0] == 0xAA && bytes[1] == 0x55 && 
            bytes[2] == 0xAA && bytes[3] == 0xAA) {
            printf("Found preamble pattern!\n");
            return 1;
        }
    }
    
    printf("Failed to find preamble pattern\n");
    return 0;
}

// Monitor the SYNC line for activity
int check_for_sync() {
    static int prev_sync = 0;
    int current_sync = read_sync_pin();
    
    // Check for LOW to HIGH transition
    if (!prev_sync && current_sync) {
        printf("\nSYNC detected\n");
        prev_sync = current_sync;
        
        // Quick verification that SYNC is stable
        for (int i = 0; i < 5; i++) {
            delay_us(500);
            if (!read_sync_pin()) {
                return 0;
            }
        }
        
        return 1;
    }
    
    prev_sync = current_sync;
    return 0;
}

// Receive a message from the Arduino
int receive_message() {
    unsigned char length;
    
    // Turn on LED 0 during reception
    *LEDR_ptr |= 0x1;
    
    printf("Starting reception\n");
    
    // Wait for start pattern
    if (!wait_for_start_pattern()) {
        printf("Failed to find valid start pattern\n");
        *LEDR_ptr &= ~0x1;
        *LEDR_ptr |= 0x8;  // Error LED
        delay_ms(1);
        *LEDR_ptr &= ~0x8;
        return 0;
    }
    
    // We've already received the start byte as part of the pattern
    // Receive length byte directly
    length = receive_byte();
    printf("Message length: %d bytes\n", length);
    
    // Validate length
    if (length == 0 || length >= MSG_BUFFER_SIZE) {
        printf("Invalid message length\n");
        *LEDR_ptr &= ~0x1;
        return 0;
    }
    
    // Receive message bytes
    for (int i = 0; i < length; i++) {
        byte = receive_byte();
        rx_buffer[i] = (char)byte;
    }
    rx_buffer[length] = '\0'; // Null-terminate
    
    printf("Message received: \"%s\"\n", rx_buffer);
    
    // Success indicator
    *LEDR_ptr &= ~0x1;
    *LEDR_ptr |= 0x2;
    delay_ms(1);
    *LEDR_ptr &= ~0x2;
    
    return 1;
}

// Send response message back to Arduino
void send_response() {
    // Turn on LED 4 during response transmission
    *LEDR_ptr |= 0x10;
    
    // Create response message
    sprintf(tx_buffer, "DE1_MSG_%d", message_counter++);
    int len = strlen(tx_buffer);
    
    printf("Response: \"%s\"\n", tx_buffer);
    
    // Signal Arduino we have a response
    set_data_pin_direction(1);
    set_data_pin(1);  // Pull DATA high
    
    // Wait for acknowledge from Arduino (clock pulse)
    if (!wait_for_clock_state(0, 20)) {
        printf("No acknowledge\n");
        set_data_pin_direction(0);
        *LEDR_ptr &= ~0x10;
        return;
    }
    
    if (!wait_for_clock_state(1, 20)) {
        printf("Incomplete clock cycle\n");
        set_data_pin_direction(0);
        *LEDR_ptr &= ~0x10;
        return;
    }
    
    // Send multiple start bytes for reliability
    for (int i = 0; i < 3; i++) {
        send_byte(START_BYTE);
    }
    
    // Send length byte
    send_byte((unsigned char)len);
    
    // Send each byte of the message
    for (int i = 0; i < len; i++) {
        send_byte((unsigned char)tx_buffer[i]);
    }
    
    printf("Response sent\n");
    
    // Return to input mode
    set_data_pin_direction(0);
    *LEDR_ptr &= ~0x10;
}

// Main function
int main(void) {
    // Initialize pointers to I/O devices
    JP1_ptr = (int *)JP1_BASE;
    KEY_ptr = (int *)KEY_BASE;
    LEDR_ptr = (int *)LEDR_BASE;
    
    printf("\n\n===================================\n");
    printf("DE1-SoC Fast Protocol Started\n");
    printf("===================================\n");
    
    // Initialize JP1 for communication
    init_jp1_communication();
    
    // Initial LED state
    *LEDR_ptr = 0;
    
    printf("Ready - monitoring SYNC line\n");
    printf("===================================\n\n");
    
    // Main loop
    while (1) {
        // Check for SYNC line activity
        if (check_for_sync()) {
            // SYNC detected, receive message
            delay_ms(3); // Brief delay for preparation
            if (receive_message()) {
                // If message received successfully, send response
                delay_ms(5);
                send_response();
                printf("Exchange completed\n");
            }
        }
        
        // Minimal delay to prevent CPU hogging
        delay_us(100);
    }
    
    return 0;
}