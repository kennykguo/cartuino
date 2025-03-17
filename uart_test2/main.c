// DE1-SoC UART Debug Script
// Enhanced with additional testing options and improved timing

#include <stdio.h>
#include <string.h>
#include "address_map.h"

// JP1 UART Configuration
// Can switch between D10/D11 and D0/D1
#define USE_D0_D1 1   // Set to 1 to use D0/D1 pins, 0 to use D10/D11

#if USE_D0_D1
    #define UART_RX_BIT 0x00000001  // Bit 0 for RX (D0)
    #define UART_TX_BIT 0x00000002  // Bit 1 for TX (D1)
    #define PIN_RX_NAME "D0"
    #define PIN_TX_NAME "D1"
#else
    #define UART_RX_BIT 0x00000400  // Bit 10 for RX (D10)
    #define UART_TX_BIT 0x00000800  // Bit 11 for TX (D11)
    #define PIN_RX_NAME "D10"
    #define PIN_TX_NAME "D11"
#endif

#define UART_BAUD_RATE 2400     // Reduced baud rate for more reliable communication
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

// Test mode selection
#define MODE_NORMAL  0
#define MODE_PIN_TEST 1
#define MODE_LOOPBACK 2
#define MODE_PATTERN 3

int current_test_mode = MODE_NORMAL;

// Simple delay in milliseconds
void delay_ms(int ms) {
    volatile int i;
    for (i = 0; i < ms * (CLOCK_RATE / 10000); i++);
}

// Delay loop for accurate bit timing
void bit_delay() {
    volatile int i;
    for (i = 0; i < BIT_PERIOD; i++);
}

// Delay for half a bit period - modified to sample at 60% instead of 50%
void half_bit_delay() {
    volatile int i;
    // Use 60% of bit time instead of 50% for better noise margin
    for (i = 0; i < (BIT_PERIOD*6)/10; i++);
}

// Initialize the JP1 port for UART communication
void init_jp1_uart() {
    // Set direction for RX as input, TX as output
    *(JP1_ptr + 1) = UART_TX_BIT;  // TX as output, RX as input
    
    // Set TX pin high (idle state for UART)
    *(JP1_ptr) |= UART_TX_BIT;
    
    // Add 10ms delay for stability
    delay_ms(10);
    
    printf("JP1 UART initialized at %d baud\n", UART_BAUD_RATE);
    printf("UART_RX_BIT = 0x%08X (%s), UART_TX_BIT = 0x%08X (%s)\n", 
           UART_RX_BIT, PIN_RX_NAME, UART_TX_BIT, PIN_TX_NAME);
    printf("JP1 direction register value: 0x%08X\n", *(JP1_ptr + 1));
    printf("JP1 data register value: 0x%08X\n", *(JP1_ptr));
    printf("BIT_PERIOD = %d clock cycles\n", BIT_PERIOD);
    
    // Manual test of pin state at initialization
    printf("RX pin initial state: %s\n", 
           (*(JP1_ptr) & UART_RX_BIT) ? "HIGH" : "LOW");
}

// Send a single byte over the software UART - improved with stabilization delays
void uart_tx_byte(unsigned char data) {
    int i;
    int tx_data = *(JP1_ptr);
    
    // Print debug info
    printf("Sending byte: 0x%02X ('%c')\n", data, (data >= 32 && data <= 126) ? data : '?');
    
    // Start bit (LOW)
    tx_data &= ~UART_TX_BIT;  // Clear TX bit
    *(JP1_ptr) = tx_data;
    delay_ms(1);  // Short stabilization delay after bit transition
    printf("TX pin set LOW (start bit)\n");
    bit_delay();
    
    // Data bits (LSB first)
    for (i = 0; i < 8; i++) {
        if (data & (1 << i)) {
            // Data bit is 1
            tx_data |= UART_TX_BIT;
            printf("TX bit %d: HIGH\n", i);
        } else {
            // Data bit is 0
            tx_data &= ~UART_TX_BIT;
            printf("TX bit %d: LOW\n", i);
        }
        *(JP1_ptr) = tx_data;
        delay_ms(1);  // Short stabilization delay after bit transition
        bit_delay();
    }
    
    // Stop bit (HIGH)
    tx_data |= UART_TX_BIT;
    *(JP1_ptr) = tx_data;
    delay_ms(1);  // Short stabilization delay after bit transition
    printf("TX pin set HIGH (stop bit)\n");
    bit_delay();
    
    // Extra delay for better framing - double the stop bit time
    bit_delay();
    printf("Byte transmission complete\n");
}

// Send a string over the software UART
void uart_tx_string(char *str) {
    char *p = str;
    printf("Starting to send string: \"%s\"\n", str);
    while (*p) {
        uart_tx_byte(*p++);
        delay_ms(10);  // Add delay between bytes for better reliability
    }
    printf("String transmission complete\n");
}

// Wait for and receive one byte over the software UART - improved for better tolerance
int uart_rx_byte(int timeout_ms) {
    int i;
    unsigned char rx_data = 0;
    int current_bit;
    
    // Maximum wait cycles for timeo