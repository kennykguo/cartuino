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
    set_pin_value(READY_PIN_