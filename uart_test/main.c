// Simple DE1-SoC UART Test
// Tests basic UART communication with Feather 32u4 via JP1 expansion port

// Include address map for hardware locations
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
    while (*str) {
        uart_tx_byte(*str++);
    }
    printf("Sent: \"%s\"\n", str);
}

// Wait for and receive one byte over the software UART
// Returns -1 on timeout or framing error
int uart_rx_byte(int timeout_ms) {
    int i;
    unsigned char rx_data = 0;
    int