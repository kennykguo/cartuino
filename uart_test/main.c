// Simple DE1-SoC UART Test
// Tests basic UART communication with Feather 32u4 via JP1 expansion port

#include <stdio.h>
#include <string.h>
#include "address_map.h"

// JP1 UART Configuration
#define UART_RX_BIT 0x00000001  // Bit 0 for RX (D0)
#define UART_TX_BIT 0x00000002  // Bit 1 for TX (D1)
#define UART_BAUD_RATE 115200
#define CLOCK_RATE 100000000    // 100MHz DE1-SoC system clock
#define BIT_PERIOD (CLOCK_RATE / UART_BAUD_RATE)

// Global variables
volatile int *JP1_ptr;      // Pointer to JP1 expansion port for UART
volatile int *KEY_ptr;      // Pointer to pushbutton KEYs
volatile int *SW_ptr;       // Pointer to slider switches
volatile int *LEDR_ptr;     // Pointer to red LEDs
volatile int *TIMER_ptr;    // Pointer to interval timer

// Buffer for received data
#define RX_BUFFER_SIZE 64
char rx_buffer[RX_BUFFER_SIZE];
int rx_buffer_pos = 0;

// Test message counter
int message_counter = 0;

// Initialize the JP1 port for UART communication
void init_jp1_uart() {
    // Set direction for D0 (RX) as input, D1 (TX) as output
    *(JP1_ptr + 1) = UART_TX_BIT;  // D1 as output, D0 as input
    
    // Set TX pin high (idle state for UART)
    *(JP1_ptr) = UART_TX_BIT;
    
    printf("JP1 UART initialized at %d baud\n", UART_BAUD_RATE);
}

// Delay loop for accurate bit timing
void bit_delay() {
    volatile int i;
    for (i = 0; i < BIT_PERIOD; i++);
}

// Delay for half a bit period
void half_bit_delay() {
    volatile int i;
    for (i = 0; i < BIT_PERIOD/2; i++);
}

// Send a single byte over the software UART
void uart_tx_byte(unsigned char data) {
    int i;
    int tx_data = *(JP1_ptr);
    
    // Start bit (LOW)
    tx_data &= ~UART_TX_BIT;  // Clear TX bit
    *(JP1_ptr) = tx_data;
    bit_delay();
    
    // Data bits (LSB first)
    for (i = 0; i < 8; i++) {
        if (data & (1 << i)) {
            // Data bit is 1
            tx_data |= UART_TX_BIT;
        } else {
            // Data bit is 0
            tx_data &= ~UART_TX_BIT;
        }
        *(JP1_ptr) = tx_data;
        bit_delay();
    }
    
    // Stop bit (HIGH)
    tx_data |= UART_TX_BIT;
    *(JP1_ptr) = tx_data;
    bit_delay();
    
    // Extra delay to ensure proper framing
    bit_delay();
}

// Send a string over the software UART
void uart_tx_string(char *str) {
    char *p = str;
    while (*p) {
        uart_tx_byte(*p++);
    }
    printf("Sent: \"%s\"\n", str);
}

// Wait for and receive one byte over the software UART
// Returns -1 on timeout or framing error
int uart_rx_byte(int timeout_ms) {
    int i;
    unsigned char rx_data = 0;
    int current_bit;
    
    // Maximum wait cycles for timeout
    int max_wait = timeout_ms * (CLOCK_RATE / 1000);
    int wait_count = 0;
    
    // Wait for start bit (falling edge on RX)
    while ((*(JP1_ptr) & UART_RX_BIT)) {
        wait_count++;
        if (timeout_ms > 0 && wait_count > max_wait) {
            return -1;  // Timeout
        }
    }
    
    // Confirm it's a start bit after half a bit time
    half_bit_delay();
    if ((*(JP1_ptr) & UART_RX_BIT) != 0) {
        return -1;  // False start bit (framing error)
    }
    
    // Skip the rest of the start bit
    half_bit_delay();
    
    // Sample in the middle of each bit
    for (i = 0; i < 8; i++) {
        bit_delay();
        current_bit = (*(JP1_ptr) & UART_RX_BIT) ? 1 : 0;
        rx_data |= (current_bit << i);  // LSB first
    }
    
    // Wait for stop bit
    bit_delay();
    if ((*(JP1_ptr) & UART_RX_BIT) == 0) {
        return -1;  // Missing stop bit (framing error)
    }
    
    return rx_data;
}

// Read a complete line (up to newline or buffer full)
// Returns number of bytes read, or -1 on error
int read_line(int timeout_ms) {
    int c;
    rx_buffer_pos = 0;
    
    while (rx_buffer_pos < RX_BUFFER_SIZE - 1) {
        c = uart_rx_byte(timeout_ms);
        
        if (c < 0) {
            if (rx_buffer_pos > 0) {
                // If we've already read some data, return it
                break;
            }
            return -1;  // Error or timeout with no data
        }
        
        rx_buffer[rx_buffer_pos++] = (char)c;
        
        // Stop at newline
        if (c == '\n' || c == '\r') {
            break;
        }
    }
    
    // Null terminate
    rx_buffer[rx_buffer_pos] = '\0';
    
    return rx_buffer_pos;
}

// Simple delay in milliseconds
void delay_ms(int ms) {
    volatile int i;
    for (i = 0; i < ms * (CLOCK_RATE / 10000); i++);
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
    printf("DE1-SoC Simple UART Test Started\n");
    printf("===================================\n");
    
    // Initialize JP1 for UART
    init_jp1_uart();
    
    // Main loop variables
    unsigned long last_tx_time = 0;
    unsigned long current_time;
    char tx_message[40];
    int key_value, old_key_value = 0;
    int rx_length;
    
    printf("Press KEY0 to send a test message\n");
    printf("===================================\n\n");
    
    // Main loop
    while (1) {
        // Read current time
        current_time = *(TIMER_ptr);
        
        // Read key value
        key_value = *KEY_ptr;
        
        // If KEY0 is pressed (value transitions from 0 to 1)
        if ((key_value & 0x1) && !(old_key_value & 0x1)) {
            printf("KEY0 pressed - Sending test message\n");
            
            // Turn on LED 0
            *LEDR_ptr |= 0x1;
            
            // Create and send test message
            sprintf(tx_message, "DE1SOC_MSG_%d\n", message_counter++);
            uart_tx_string(tx_message);
            
            // Wait a bit before turning off LED
            delay_ms(200);
            
            // Turn off LED 0
            *LEDR_ptr &= ~0x1;
        }
        
        // Send automatic message every 5 seconds
        if (current_time - last_tx_time > 500000000) {  // 5 seconds with 100MHz clock
            printf("Auto-sending test message\n");
            
            // Turn on LED 1
            *LEDR_ptr |= 0x2;
            
            // Create and send test message
            sprintf(tx_message, "AUTO_MSG_%d\n", message_counter++);
            uart_tx_string(tx_message);
            
            last_tx_time = current_time;
            
            // Turn off LED 1 after a delay
            delay_ms(200);
            *LEDR_ptr &= ~0x2;
        }
        
        // Check for received data with a short timeout
        rx_length = read_line(1);  // 1ms timeout
        
        if (rx_length > 0) {
            // Turn on LED 2
            *LEDR_ptr |= 0x4;
            
            // Print received data
            printf("Received %d bytes: \"%s\"\n", rx_length, rx_buffer);
            
            // Turn off LED 2 after a delay
            delay_ms(200);
            *LEDR_ptr &= ~0x4;
        }
        
        // Update old key value for edge detection
        old_key_value = key_value;
        
        // Small delay to prevent CPU hogging
        delay_ms(10);
    }
    
    return 0;
}