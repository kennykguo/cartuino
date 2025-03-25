// Brief microsecond delay
void delay_us(int us) {
    volatile int i;
    for (i = 0; i < us * (CLOCK_RATE / 1000000000); i++);
}#include <stdio.h>
#include <string.h>
#include "address_map.h"

// JP1 Pin definitions
#define DATA_PIN_BIT 0x00000001  // Bit 0 (D0) for data from Arduino D2
#define CLOCK_PIN_BIT 0x00000002 // Bit 1 (D1) for clock from Arduino D3
#define SYNC_PIN_BIT 0x00000004  // Bit 2 (D2) for sync from Arduino D5
#define CLOCK_RATE 100000000     // 100MHz DE1-SoC system clock

// Global variables
volatile int *JP1_ptr;      // Pointer to JP1 expansion port
volatile int *KEY_ptr;      // Pointer to pushbutton KEYs
volatile int *LEDR_ptr;     // Pointer to red LEDs

// Communication parameters
#define BIT_PERIOD_MS 1     // Fast bit transfer (1ms per bit)
#define MSG_BUFFER_SIZE 32  // Buffer size for messages
char rx_buffer[MSG_BUFFER_SIZE];
char tx_buffer[MSG_BUFFER_SIZE];
int message_counter = 0;

// Protocol constants
#define START_BYTE 0xAA  // 10101010 pattern for synchronization

// Debug flags
#define DEBUG_PIN_VALUES 0 // Set to 1 to print pin values continuously

// Simple delay in milliseconds
void delay_ms(int ms) {
    volatile int i;
    for (i = 0; i < ms * (CLOCK_RATE / 100000); i++);
}

// Initialize the JP1 port for communication
void init_jp1_communication() {
    // Set all pins as input initially
    *(JP1_ptr + 1) = 0x00000000;
    
    printf("DE1-SoC Bidirectional Communication Initialized\n");
    printf("Using D0(data), D1(clock), D2(sync) from Arduino\n");
    printf("JP1 direction register: 0x%08X, data register: 0x%08X\n", 
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
    delay_ms(1); // Allow time for change to take effect
}

void set_data_pin(int high) {
    if (high) {
        *(JP1_ptr) |= DATA_PIN_BIT;
    } else {
        *(JP1_ptr) &= ~DATA_PIN_BIT;
    }
    delay_ms(1); // Allow time for change to take effect
}

// Print the current state of all pins (for debugging)
void print_pin_states() {
    printf("Pins: DATA=%d, CLOCK=%d, SYNC=%d\n", 
           read_data_pin(), read_clock_pin(), read_sync_pin());
}

// Wait for clock transition (optimized for speed)
int wait_for_clock_state(int desired_state, int timeout_ms) {
    int i;
    const int SAMPLES_PER_MS = 100; // Check more frequently
    const int MAX_SAMPLES = timeout_ms * SAMPLES_PER_MS;
    
    for (i = 0; i < MAX_SAMPLES; i++) {
        int clock_state = read_clock_pin();
        if ((desired_state && clock_state) || (!desired_state && !clock_state)) {
            return 1; // Success
        }
        delay_us(10); // Check every 10μs
    }
    
    printf("TIMEOUT waiting for CLOCK=%d after %dms\n", desired_state, timeout_ms);
    return 0; // Timeout
}

// Receive a single bit
int receive_bit() {
    int bit;
    
    // Make sure DATA pin is set as input
    set_data_pin_direction(0);
    
    // Wait for clock HIGH (initial state)
    if (!wait_for_clock_state(1, 50)) {
        printf("Failed waiting for clock HIGH\n");
        return -1; // Error
    }
    
    // Wait for clock LOW (active edge)
    if (!wait_for_clock_state(0, 50)) {
        printf("Failed waiting for clock LOW\n");
        return -1; // Error
    }
    
    // Read the data bit while clock is LOW
    bit = read_data_pin();
    
    // Wait for clock to return HIGH
    wait_for_clock_state(1, 50); // Ignoring result here to continue
    
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
        if (wait_for_clock_state(0, 20)) {
            // Success! Clock went LOW
            
            // Keep data value stable while clock is LOW
            
            // Wait for clock to return HIGH
            wait_for_clock_state(1, 20);
            return; // Success
        }
        
        // Clock didn't go LOW, retry
        retries--;
        printf("Retrying bit send, %d retries left\n", retries);
        
        // Briefly pulse data line to get Arduino's attention
        set_data_pin(!bit);
        delay_ms(1);
        set_data_pin(bit);
        delay_ms(1);
    }
    
    printf("Failed to send bit after retries\n");
}

// Receive a byte (8 bits) MSB first
unsigned char receive_byte() {
    unsigned char byte = 0;
    int bit;
    
    // Receive 8 bits, MSB first
    for (int i = 7; i >= 0; i--) {
        bit = receive_bit();
        if (bit < 0) {
            printf("Error receiving bit %d\n", i);
            return 0xFF; // Error value
        }
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

// Wait for and verify the start byte with fast timing
int wait_for_start_byte() {
    unsigned char byte;
    
    // Look for multiple consecutive start bytes (more reliable)
    int start_bytes_found = 0;
    int required_matches = 2; // Need to see at least 2 consecutive start bytes
    
    while(start_bytes_found < required_matches) {
        byte = receive_byte();
        
        if (byte == START_BYTE) {
            start_bytes_found++;
            if (start_bytes_found >= required_matches) {
                printf("Valid start byte sequence detected\n");
                return 1;
            }
        } else {
            // Reset counter if we don't see consecutive start bytes
            if (start_bytes_found > 0) {
                printf("Broken start byte sequence\n");
            }
            start_bytes_found = 0;
        }
        
        // Limit total attempts
        if (start_bytes_found == 0) {
            static int attempt = 0;
            attempt++;
            printf("Attempt %d: Invalid start byte: 0x%02X\n", attempt, byte);
            
            if (attempt >= 5) {
                printf("Failed to find valid start byte sequence\n");
                return 0;
            }
        }
    }
    
    return 0;
}

// Monitor the SYNC line for activity
int check_for_sync() {
    static int prev_sync = 0;
    int current_sync = read_sync_pin();
    
    // Check for LOW to HIGH transition
    if (!prev_sync && current_sync) {
        printf("\nSYNC line HIGH detected\n");
        prev_sync = current_sync;
        
        // Quick verification that SYNC is stable
        for (int i = 0; i < 5; i++) {
            delay_us(500);
            if (!read_sync_pin()) {
                printf("SYNC not stable\n");
                return 0;
            }
        }
        
        printf("SYNC verified\n");
        return 1;
    }
    
    prev_sync = current_sync;
    return 0;
}

// Receive a message and prepare to send response
int receive_message() {
    unsigned char byte, length;
    int i, success = 0;
    
    // Turn on LED 0 during reception
    *LEDR_ptr |= 0x1;
    
    printf("Starting message reception\n");
    
    // Wait for start byte
    if (!wait_for_start_byte()) {
        printf("Failed to find valid start byte\n");
        *LEDR_ptr &= ~0x1; // Turn off LED 0
        *LEDR_ptr |= 0x8;  // Turn on error LED
        delay_ms(1);
        *LEDR_ptr &= ~0x8; // Turn off error LED
        return 0;
    }
    
    // Receive length byte
    length = receive_byte();
    printf("Message length: %d bytes\n", length);
    
    // Validate length
    if (length == 0 || length >= MSG_BUFFER_SIZE) {
        printf("Invalid message length: %d\n", length);
        *LEDR_ptr &= ~0x1;
        *LEDR_ptr |= 0x8;
        delay_ms(1);
        *LEDR_ptr &= ~0x8;
        return 0;
    }
    
    // Receive message bytes
    for (i = 0; i < length; i++) {
        byte = receive_byte();
        rx_buffer[i] = (char)byte;
    }
    rx_buffer[i] = '\0'; // Null-terminate
    
    printf("Message received: \"%s\"\n", rx_buffer);
    
    // Success indicator
    *LEDR_ptr &= ~0x1;  // Turn off LED 0
    *LEDR_ptr |= 0x2;   // Turn on success LED
    delay_ms(1);
    *LEDR_ptr &= ~0x2;  // Turn off success LED
    
    success = 1;
    return success;
}

// Send response message back to Arduino with fast timing
void send_response() {
    // Turn on LED 4 during response transmission
    *LEDR_ptr |= 0x10;
    
    printf("Preparing response\n");
    
    // Create response message
    sprintf(tx_buffer, "DE1_MSG_%d", message_counter++);
    int len = strlen(tx_buffer);
    
    // Signal Arduino we have a response
    set_data_pin_direction(1);
    set_data_pin(1);  // Pull DATA high
    
    printf("Signaling response ready\n");
    
    // Wait for acknowledge from Arduino (clock pulse)
    if (!wait_for_clock_state(0, 20)) {
        printf("No acknowledge from Arduino\n");
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
    
    // Send start bytes - send multiple for reliability
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
    
    // Turn off LED
    *LEDR_ptr &= ~0x10;
    
    // Return to input mode
    set_data_pin_direction(0);
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
    
    // Initial LED state
    *LEDR_ptr = 0;
    
    printf("Monitoring SYNC line for activity\n");
    printf("===================================\n\n");
    
    // Main loop
    while (1) {
        // Check for SYNC line activity
        if (check_for_sync()) {
            printf("SYNC detected, beginning bidirectional exchange\n");
            
            // SYNC detected, receive message
            delay_ms(50); // Allow time for Arduino to prepare
            if (receive_message()) {
                // If message received successfully, send response
                delay_ms(100); // Brief pause before response
                send_response();
                printf("Bidirectional exchange completed\n");
            } else {
                printf("Failed to receive message, no response sent\n");
            }
            
            // Ensure data pin is back to input mode
            set_data_pin_direction(0);
        }
        
        // Optional: periodically print pin states for debugging
        if (DEBUG_PIN_VALUES) {
            static int counter = 0;
            if (++counter >= 500) { // Every ~500ms
                print_pin_states();
                counter = 0;
            }
        }
        
        // Small delay
        delay_ms(1);
    }
    
    return 0;
}