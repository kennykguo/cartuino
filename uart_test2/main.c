// DE1-SoC UART Debug Script
// Enhanced to troubleshoot one-way communication issues

#include <stdio.h>
#include <string.h>
#include "address_map.h"

// JP1 UART Configuration
// Using D10/D11 for RX/TX
#define UART_RX_BIT 0x00000400  // Bit 10 for RX (D10)
#define UART_TX_BIT 0x00000800  // Bit 11 for TX (D11)
#define UART_BAUD_RATE 2400  // Even slower for more tolerance
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

// Initialize the JP1 port for UART communication
void init_jp1_uart() {
    // Set direction for RX as input, TX as output
    *(JP1_ptr + 1) = UART_TX_BIT;  // TX as output, RX as input
    
    // Set TX pin high (idle state for UART)
    *(JP1_ptr) |= UART_TX_BIT;
    
    // Add 10ms delay for stability
    delay_ms(10);
    
    printf("JP1 UART initialized at %d baud\n", UART_BAUD_RATE);
    printf("UART_RX_BIT = 0x%08X, UART_TX_BIT = 0x%08X\n", UART_RX_BIT, UART_TX_BIT);
    printf("JP1 direction register value: 0x%08X\n", *(JP1_ptr + 1));
    printf("JP1 data register value: 0x%08X\n", *(JP1_ptr));
    printf("BIT_PERIOD = %d clock cycles\n", BIT_PERIOD);
    
    // Manual test of pin state at initialization
    printf("RX pin initial state: %s\n", 
           (*(JP1_ptr) & UART_RX_BIT) ? "HIGH" : "LOW");
}

// Delay loop for accurate bit timing
void bit_delay() {
    volatile int i;
    for (i = 0; i < BIT_PERIOD; i++);
}

// Delay for half a bit period
void half_bit_delay() {
    volatile int i;
    // Sample at 60% of bit time instead of 50%
    for (i = 0; i < (BIT_PERIOD*6)/10; i++);
}

// Simple delay in milliseconds
void delay_ms(int ms) {
    volatile int i;
    for (i = 0; i < ms * (CLOCK_RATE / 10000); i++);
}

// Send a single byte over the software UART
void uart_tx_byte(unsigned char data) {
    int i;
    // In uart_tx_byte(), after each bit transition:
    *(JP1_ptr) = tx_data;
    delay_ms(1);  // Short stabilization delay
    bit_delay();
    
    // Print debug info
    printf("Sending byte: 0x%02X ('%c')\n", data, (data >= 32 && data <= 126) ? data : '?');
    
    // Start bit (LOW)
    tx_data &= ~UART_TX_BIT;  // Clear TX bit
    *(JP1_ptr) = tx_data;
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
        bit_delay();
    }
    
    // Stop bit (HIGH)
    tx_data |= UART_TX_BIT;
    *(JP1_ptr) = tx_data;
    printf("TX pin set HIGH (stop bit)\n");
    bit_delay();
    
    // Extra delay to ensure proper framing
    bit_delay();
    printf("Byte transmission complete\n");
}

// Send a string over the software UART
void uart_tx_string(char *str) {
    char *p = str;
    printf("Starting to send string: \"%s\"\n", str);
    while (*p) {
        uart_tx_byte(*p++);
    }
    printf("String transmission complete\n");
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
        printf("Warning: Missing stop bit (framing error)\n");
        return -1;  // Missing stop bit (framing error)
    }
    
    printf("Received byte: 0x%02X ('%c')\n", rx_data, 
          (rx_data >= 32 && rx_data <= 126) ? (char)rx_data : '?');
    
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

// Test TX pin by toggling it directly
void test_tx_pin() {
    printf("\n===== TX PIN TOGGLE TEST =====\n");
    printf("Toggling TX pin (JP1 D11) 10 times...\n");
    
    // Turn on LED 9 during test
    *LEDR_ptr |= 0x200;
    
    // Toggle TX pin 10 times with visible timing
    for (int i = 0; i < 10; i++) {
        // Set TX low
        int tx_data = *(JP1_ptr);
        tx_data &= ~UART_TX_BIT;
        *(JP1_ptr) = tx_data;
        printf("TX pin set LOW, JP1 data reg: 0x%08X\n", *(JP1_ptr));
        
        // Update LEDs to show state
        *LEDR_ptr = (*LEDR_ptr & 0xFFE) | 0x200;
        delay_ms(300);
        
        // Set TX high
        tx_data |= UART_TX_BIT;
        *(JP1_ptr) = tx_data;
        printf("TX pin set HIGH, JP1 data reg: 0x%08X\n", *(JP1_ptr));
        
        // Update LEDs to show state
        *LEDR_ptr = (*LEDR_ptr & 0xFFE) | 0x201 | 0x200;
        delay_ms(300);
    }
    
    // Turn off LED 9
    *LEDR_ptr &= ~0x200;
    
    printf("TX pin toggle test complete\n");
    printf("===========================\n\n");
}

// Test loopback by connecting TX to RX externally
void test_loopback() {
    printf("\n===== LOOPBACK TEST =====\n");
    printf("Connect DE1-SoC JP1 D11 (TX) to JP1 D10 (RX) physically\n");
    printf("Sending test characters and checking if they loop back...\n");
    
    // Turn on LED 8 during test
    *LEDR_ptr |= 0x100;
    
    char test_chars[] = "ABCDE";
    int success_count = 0;
    
    for (int i = 0; i < 5; i++) {
        printf("Testing character '%c':\n", test_chars[i]);
        
        // Send the character
        uart_tx_byte(test_chars[i]);
        
        // Try to receive it back with a timeout
        int received = uart_rx_byte(200);  // 200ms timeout
        
        if (received == test_chars[i]) {
            printf("SUCCESS: Sent '%c', Received '%c'\n", test_chars[i], (char)received);
            success_count++;
            
            // Flash LED 0 for success
            *LEDR_ptr |= 0x1;
            delay_ms(200);
            *LEDR_ptr &= ~0x1;
        } else {
            printf("FAILED: Sent '%c', Received %d (0x%02X)\n", 
                  test_chars[i], received, received);
            
            // Flash LED 1 for failure
            *LEDR_ptr |= 0x2;
            delay_ms(200);
            *LEDR_ptr &= ~0x2;
        }
        
        delay_ms(500);  // Delay between tests
    }
    
    // Turn off LED 8
    *LEDR_ptr &= ~0x100;
    
    printf("Loopback test complete: %d/5 successful\n", success_count);
    printf("=========================\n\n");
}

// Test sending a distinctive bit pattern
void test_bit_pattern() {
    printf("\n===== BIT PATTERN TEST =====\n");
    printf("Sending a distinctive bit pattern...\n");
    
    // Turn on LED 7 during test
    *LEDR_ptr |= 0x80;
    
    // Send a series of alternating 0x55 (01010101) and 0xAA (10101010)
    // These patterns are distinctive and easy to identify on a logic analyzer
    for (int i = 0; i < 5; i++) {
        printf("Sending 0x55 (alternating bits starting with 0)...\n");
        uart_tx_byte(0x55);
        delay_ms(500);
        
        printf("Sending 0xAA (alternating bits starting with 1)...\n");
        uart_tx_byte(0xAA);
        delay_ms(500);
    }
    
    // Now send a simple ASCII pattern
    uart_tx_string("UART-TEST");
    
    // Turn off LED 7
    *LEDR_ptr &= ~0x80;
    
    printf("Bit pattern test complete\n");
    printf("==========================\n\n");
}

// Direct pin-to-pin loopback test
void test_direct_loopback() {
    printf("\n===== DIRECT PIN LOOPBACK TEST =====\n");
    printf("Testing direct connection between TX (0x%08X) and RX (0x%08X) pins\n", 
           UART_TX_BIT, UART_RX_BIT);
    
    // Turn on LED 6 during test
    *LEDR_ptr |= 0x40;
    
    // Get current pin state
    int tx_data = *(JP1_ptr);
    int initial_rx_state = (*(JP1_ptr) & UART_RX_BIT) ? 1 : 0;
    
    printf("Initial state: TX pin is %s, RX pin is %s\n",
           (tx_data & UART_TX_BIT) ? "HIGH" : "LOW",
           initial_rx_state ? "HIGH" : "LOW");
    
    // Test 1: Set TX high and check RX
    tx_data |= UART_TX_BIT;
    *(JP1_ptr) = tx_data;
    delay_ms(50);  // Allow time for signal to propagate
    
    int rx_state_when_tx_high = (*(JP1_ptr) & UART_RX_BIT) ? 1 : 0;
    printf("TX HIGH test: TX pin set HIGH, RX pin is %s\n", 
           rx_state_when_tx_high ? "HIGH (following)" : "LOW (not following)");
    
    // Test 2: Set TX low and check RX
    tx_data &= ~UART_TX_BIT;
    *(JP1_ptr) = tx_data;
    delay_ms(50);  // Allow time for signal to propagate
    
    int rx_state_when_tx_low = (*(JP1_ptr) & UART_RX_BIT) ? 1 : 0;
    printf("TX LOW test: TX pin set LOW, RX pin is %s\n", 
           rx_state_when_tx_low ? "HIGH (not following)" : "LOW (following)");
    
    // Test 3: Perform several fast toggles to check consistent behavior
    printf("\nRapid toggle test (TX HIGH→LOW→HIGH, checking RX after each change):\n");
    for (int i = 0; i < 5; i++) {
        // Set TX HIGH
        tx_data |= UART_TX_BIT;
        *(JP1_ptr) = tx_data;
        delay_ms(20);
        rx_state_when_tx_high = (*(JP1_ptr) & UART_RX_BIT) ? 1 : 0;
        printf("Toggle %d HIGH: RX is %s (JP1=0x%08X)\n", 
               i+1, rx_state_when_tx_high ? "HIGH" : "LOW", *(JP1_ptr));
        
        // Set TX LOW
        tx_data &= ~UART_TX_BIT;
        *(JP1_ptr) = tx_data;
        delay_ms(20);
        rx_state_when_tx_low = (*(JP1_ptr) & UART_RX_BIT) ? 1 : 0;
        printf("Toggle %d LOW: RX is %s (JP1=0x%08X)\n", 
               i+1, rx_state_when_tx_low ? "HIGH" : "LOW", *(JP1_ptr));
    }
    
    // Test 4: Check if reading/writing the I/O register affects behavior
    printf("\nTesting input latching behavior:\n");
    
    // Set TX HIGH but read JP1 multiple times
    tx_data |= UART_TX_BIT;
    *(JP1_ptr) = tx_data;
    
    for (int i = 0; i < 3; i++) {
        delay_ms(10);
        printf("Read %d after TX HIGH: RX is %s (JP1=0x%08X)\n", 
               i+1, (*(JP1_ptr) & UART_RX_BIT) ? "HIGH" : "LOW", *(JP1_ptr));
    }
    
    // Set TX LOW but read JP1 multiple times
    tx_data &= ~UART_TX_BIT;
    *(JP1_ptr) = tx_data;
    
    for (int i = 0; i < 3; i++) {
        delay_ms(10);
        printf("Read %d after TX LOW: RX is %s (JP1=0x%08X)\n", 
               i+1, (*(JP1_ptr) & UART_RX_BIT) ? "HIGH" : "LOW", *(JP1_ptr));
    }
    
    // Result summary
    printf("\nTest Summary:\n");
    if (rx_state_when_tx_high && !rx_state_when_tx_low) {
        printf("PASS: RX pin correctly follows TX pin (normal connection)\n");
    } else if (!rx_state_when_tx_high && rx_state_when_tx_low) {
        printf("PASS with INVERSION: RX pin follows TX pin with inversion\n");
    } else {
        printf("FAIL: RX pin does not reliably follow TX pin\n");
        printf("Possible causes:\n");
        printf("1. No physical connection between pins\n");
        printf("2. Input buffer configuration issue\n");
        printf("3. I/O standard mismatch\n");
        printf("4. Electrical level incompatibility\n");
    }
    
    // Turn off LED 6
    *LEDR_ptr &= ~0x40;
    
    printf("====================================\n\n");
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
    printf("DE1-SoC UART Debug Test Started\n");
    printf("===================================\n");
    
    // Initialize JP1 for UART
    init_jp1_uart();
    
    // Main loop variables
    unsigned long last_tx_time = 0;
    unsigned long current_time;
    char tx_message[40];
    int key_value, old_key_value = 0;
    int rx_length;
    int sw_value, old_sw_value = 0;
    
    printf("Test Controls:\n");
    printf("- KEY0: Send test message in current mode\n");
    printf("- KEY1: Run TX pin toggle test\n");
    printf("- KEY2: Run loopback test\n");
    printf("- KEY3: Run direct pin loopback test\n");
    printf("- Switch settings control test mode:\n");
    printf("  SW0=0: Normal mode (short messages)\n");
    printf("  SW0=1: Different test patterns\n");
    printf("===================================\n\n");
    
    // Initial LED state
    *LEDR_ptr = 0;
    
    // Main loop
    while (1) {
        // Read current time
        current_time = *(TIMER_ptr);
        
        // Read key value for edge detection
        key_value = *KEY_ptr;
        
        // Read switch value
        sw_value = *SW_ptr;
        
        // Update test mode based on switches
        current_test_mode = sw_value & 0x1;
        
        // Show current mode on LEDs 4-6
        *LEDR_ptr = (*LEDR_ptr & 0xFFF0) | (current_test_mode & 0xF);
        
        // If switches changed, print the new mode
        if (sw_value != old_sw_value) {
            printf("Test mode changed to: %d\n", current_test_mode);
            old_sw_value = sw_value;
        }
        
        // KEY0: Send test message
        if ((key_value & 0x1) && !(old_key_value & 0x1)) {
            printf("\nKEY0 pressed - Sending test message\n");
            
            // Turn on LED 0
            *LEDR_ptr |= 0x1;
            
            // Create and send test message based on mode
            if (current_test_mode == MODE_NORMAL) {
                // In normal mode, send a short, simple message
                sprintf(tx_message, "TEST%d\n", message_counter++);
            } else {
                // In alternate mode, send more distinctive patterns
                sprintf(tx_message, "DE1_%c%c%c\n", 'A' + (message_counter % 26),
                       'A' + ((message_counter + 1) % 26),
                       'A' + ((message_counter + 2) % 26));
                message_counter++;
            }
            
            uart_tx_string(tx_message);
            
            // Wait a bit before turning off LED
            delay_ms(200);
            
            // Turn off LED 0
            *LEDR_ptr &= ~0x1;
        }
        
        // KEY1: Run TX pin toggle test
        if ((key_value & 0x2) && !(old_key_value & 0x2)) {
            printf("\nKEY1 pressed - Running TX pin toggle test\n");
            test_tx_pin();
        }
        
        // KEY2: Run loopback test
        if ((key_value & 0x4) && !(old_key_value & 0x4)) {
            printf("\nKEY2 pressed - Running loopback test\n");
            test_loopback();
        }
        
        // KEY3: Run direct pin loopback test
        if ((key_value & 0x8) && !(old_key_value & 0x8)) {
            printf("\nKEY3 pressed - Running direct pin loopback test\n");
            test_direct_loopback();
        }
        
        // Check for received data with a short timeout
        rx_length = read_line(1);  // 1ms timeout
        
        if (rx_length > 0) {
            // Turn on LED 2
            *LEDR_ptr |= 0x4;
            
            // Print received data
            printf("Received %d bytes: \"", rx_length);
            for (int i = 0; i < rx_length; i++) {
                if (rx_buffer[i] >= 32 && rx_buffer[i] <= 126) {
                    printf("%c", rx_buffer[i]);
                } else {
                    printf("\\x%02X", rx_buffer[i] & 0xFF);
                }
            }
            printf("\"\n");
            
            // Also print hex values for debugging
            printf("Hex values: ");
            for (int i = 0; i < rx_length; i++) {
                printf("%02X ", rx_buffer[i] & 0xFF);
            }
            printf("\n");
            
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