// DE1-SoC Simple Communication Protocol
// For reliable communication with Arduino

#include <stdio.h>
#include <string.h>
#include "address_map.h"

// JP1 Pin definitions
#define DATA_PIN_BIT 0x00000001  // Bit 0 (D0) for data
#define CLOCK_PIN_BIT 0x00000002 // Bit 1 (D1) for clock

// System clock definition
#define CLOCK_RATE 100000000    // 100MHz DE1-SoC system clock

// Global variables
volatile int *JP1_ptr;      // Pointer to JP1 expansion port
volatile int *KEY_ptr;      // Pointer to pushbutton KEYs
volatile int *SW_ptr;       // Pointer to slider switches
volatile int *LEDR_ptr;     // Pointer to red LEDs
volatile int *TIMER_ptr;    // Pointer to interval timer

// Communication parameters
#define BIT_DELAY_MS 50     // 50ms per bit for reliable communication
#define MSG_BUFFER_SIZE 64  // Buffer size for messages
char tx_buffer[MSG_BUFFER_SIZE];
char rx_buffer[MSG_BUFFER_SIZE];
int rx_buffer_pos = 0;

// Message counter
int message_counter = 0;

// Simple delay in milliseconds
void delay_ms(int ms) {
    volatile int i;
    for (i = 0; i < ms * (CLOCK_RATE / 10000); i++);
}

// Initialize the JP1 port for communication
void init_jp1_communication() {
    // Set DATA pin as input by default, CLOCK pin as output
    *(JP1_ptr + 1) = CLOCK_PIN_BIT;  // Direction register
    
    // Set CLOCK pin high (idle state)
    *(JP1_ptr) |= CLOCK_PIN_BIT;
    
    // Add delay for stability
    delay_ms(10);
    
    printf("JP1 communication initialized\n");
    printf("DATA_PIN_BIT = 0x%08X (D0), CLOCK_PIN_BIT = 0x%08X (D1)\n", 
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
}

// Set the DATA pin value (when configured as output)
void set_data_pin(int high) {
    if (high) {
        *(JP1_ptr) |= DATA_PIN_BIT;
    } else {
        *(JP1_ptr) &= ~DATA_PIN_BIT;
    }
}

// Set the CLOCK pin value
void set_clock_pin(int high) {
    if (high) {
        *(JP1_ptr) |= CLOCK_PIN_BIT;
    } else {
        *(JP1_ptr) &= ~CLOCK_PIN_BIT;
    }
}

// Read the DATA pin value
int read_data_pin() {
    return (*(JP1_ptr) & DATA_PIN_BIT) ? 1 : 0;
}

// Send a message byte-by-byte with explicit clock signaling
void send_message(char *message) {
    int len = strlen(message);
    
    // Turn on LED 0 during transmission
    *LEDR_ptr |= 0x1;
    
    printf("Sending message: \"%s\" (%d bytes)\n", message, len);
    
    // Set DATA pin as output
    set_data_pin_direction(1);
    printf("DATA pin set to OUTPUT mode\n");
    
    // Signal start of transmission by toggling clock
    set_clock_pin(0);  // CLOCK LOW to signal start
    printf("CLOCK pin set LOW (signaling start)\n");
    delay_ms(BIT_DELAY_MS * 2);  // Double delay for start signal
    
    // Send each byte of the message
    for (int i = 0; i < len; i++) {
        unsigned char byte = message[i];
        printf("Sending byte %d/%d: 0x%02X ('%c')\n", 
               i+1, len, byte, (byte >= 32 && byte <= 126) ? byte : '?');
        
        // Send 8 bits, MSB first
        for (int j = 7; j >= 0; j--) {
            int bit = (byte >> j) & 0x01;
            
            // Set the bit value
            set_data_pin(bit);
            printf("  Setting bit %d to %d\n", j, bit);
            
            // Toggle clock to signal bit is ready
            set_clock_pin(0);  // CLOCK LOW
            delay_ms(BIT_DELAY_MS / 2);
            
            set_clock_pin(1);  // CLOCK HIGH
            delay_ms(BIT_DELAY_MS / 2);
        }
        
        // Small delay between bytes
        delay_ms(BIT_DELAY_MS);
    }
    
    // End transmission signal
    set_clock_pin(0);  // CLOCK LOW
    printf("CLOCK pin set LOW (signaling end)\n");
    delay_ms(BIT_DELAY_MS * 2);  // Double delay for end signal
    
    set_clock_pin(1);  // Return CLOCK to idle HIGH state
    printf("CLOCK pin set HIGH (idle)\n");
    
    // Set DATA pin back to input mode for receiving
    set_data_pin_direction(0);
    printf("DATA pin set back to INPUT mode\n");
    
    // Turn off LED 0
    *LEDR_ptr &= ~0x1;
    
    printf("Message transmission complete\n");
}

// Monitor for Arduino data activity - add to main loop
void monitor_arduino() {
    static int last_data_value = -1;
    int data_value = read_data_pin();
    
    // If DATA pin changes, log it
    if (data_value != last_data_value) {
        printf("DATA pin changed to: %s\n", data_value ? "HIGH" : "LOW");
        last_data_value = data_value;
        
        // Toggle LED 1 to show data activity
        *LEDR_ptr ^= 0x2;
    }
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
    printf("DE1-SoC Simple Communication Started\n");
    printf("===================================\n");
    
    // Initialize JP1 for communication
    init_jp1_communication();
    
    // Main loop variables
    int key_value, old_key_value = 0;
    int sw_value;
    unsigned long last_send_time = 0;
    unsigned long current_time;
    int auto_send_enabled = 1; // Enable automatic sending
    
    printf("Test Controls:\n");
    printf("- KEY0: Send test message to Arduino\n");
    printf("- KEY2: Toggle automatic sending (every 3 seconds)\n");
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
        
        // Get current time (approximation)
        current_time = *(TIMER_ptr);
        
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
            last_send_time = current_time;
        }
        
        // KEY2: Toggle auto-send mode
        if ((key_value & 0x4) && !(old_key_value & 0x4)) {
            auto_send_enabled = !auto_send_enabled;
            printf("\nKEY2 pressed - Auto-send mode is now %s\n", 
                   auto_send_enabled ? "ENABLED" : "DISABLED");
            
            // Update LED to show auto-send status
            if (auto_send_enabled) {
                *LEDR_ptr |= 0x8;   // Set LED 3
            } else {
                *LEDR_ptr &= ~0x8;  // Clear LED 3
            }
        }
        
        // Auto-send test message if enabled (every 3 seconds)
        if (auto_send_enabled && (current_time - last_send_time > CLOCK_RATE * 3)) {
            printf("\nAuto-sending test message\n");
            
            // Simple test message
            strcpy(tx_buffer, "Hello Arduino");
            
            send_message(tx_buffer);
            last_send_time = current_time;
        }
        
        // Update old key value for edge detection
        old_key_value = key_value;
        
        // Monitor for Arduino activity
        monitor_arduino();
        
        // Small delay to prevent CPU hogging
        delay_ms(5);
    }
    
    return 0;
}