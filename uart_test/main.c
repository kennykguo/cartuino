// DE1-SoC UART Benchmarking via JP1 port
// Tests latency and reliability of JP1 UART connection with Arduino

// Include address map for hardware locations
#include "address_map.h"

// JP1 UART Configuration - using the same pins as in your main code
// D0 = RX pin (input to DE1-SoC) connected to Arduino's TX (pin 12)
// D1 = TX pin (output from DE1-SoC) connected to Arduino's RX (pin 13)
#define UART_RX_BIT 0x00000001  // Bit 0 for RX (D0)
#define UART_TX_BIT 0x00000002  // Bit 1 for TX (D1)
#define UART_BAUD_RATE 115200
#define CLOCK_RATE 100000000    // 100MHz DE1-SoC system clock
#define BIT_PERIOD (CLOCK_RATE / UART_BAUD_RATE)

// Benchmark parameters
#define NUM_PACKETS 100         // Number of packets to send in the test
#define PACKET_SIZE 32          // Size of each test packet in bytes
#define WARMUP_PACKETS 5        // Number of warmup packets before measurement
#define ECHO_TEST_PACKETS 20    // Number of packets for round-trip test
#define TEST_TYPE_LATENCY 1
#define TEST_TYPE_THROUGHPUT 2
#define TEST_TYPE_RELIABILITY 3

// Global variables
volatile int *JP1_ptr;          // Pointer to JP1 expansion port for UART
volatile int *KEY_ptr;          // Pointer to pushbutton KEYs
volatile int *SW_ptr;           // Pointer to slider switches
volatile int *LEDR_ptr;         // Pointer to red LEDs
volatile int *TIMER_ptr;        // Pointer to interval timer
volatile char *VGA_CHAR_ptr;    // Pointer to VGA character buffer

// Statistics
unsigned long total_time = 0;
unsigned long min_time = 0xFFFFFFFF;
unsigned long max_time = 0;
int packet_errors = 0;
int bytes_received = 0;
int bytes_sent = 0;
int test_type = TEST_TYPE_LATENCY;

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

// Initialize the JP1 port for UART communication
void init_jp1_uart() {
    // Set direction for D0 (RX) as input, D1 (TX) as output
    // In the Direction register at Base+4, 0=input, 1=output
    *(JP1_ptr + 1) = UART_TX_BIT;  // D1 as output, D0 as input
    
    // Set TX pin high (idle state for UART)
    *(JP1_ptr) = UART_TX_BIT;
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
        bytes_sent++;
    }
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
    
    bytes_received++;
    return rx_data;
}

// Generate a test packet with incrementing sequence
void generate_test_packet(unsigned char *packet, int size, int seq_num) {
    for (int i = 0; i < size; i++) {
        if (i == 0) {
            // First byte is sequence number
            packet[i] = (unsigned char)(seq_num & 0xFF);
        } else {
            // Rest is incrementing data
            packet[i] = (unsigned char)((i + seq_num) & 0xFF);
        }
    }
}

// Check if received packet matches what was sent
int validate_packet(unsigned char *sent, unsigned char *received, int size) {
    int errors = 0;
    for (int i = 0; i < size; i++) {
        if (sent[i] != received[i]) {
            errors++;
        }
    }
    return errors;
}

// Send a test command to Arduino
void send_test_command(int test_type) {
    char command[16];
    switch (test_type) {
        case TEST_TYPE_LATENCY:
            uart_tx_string("B,L\n");  // Benchmark, Latency test
            break;
        case TEST_TYPE_THROUGHPUT:
            uart_tx_string("B,T\n");  // Benchmark, Throughput test
            break;
        case TEST_TYPE_RELIABILITY:
            uart_tx_string("B,R\n");  // Benchmark, Reliability test
            break;
    }
}

// Write to VGA character buffer
void write_to_vga(int x, int y, char* text) {
    volatile char* char_ptr = VGA_CHAR_ptr + (y * 80 + x);
    while (*text) {
        *char_ptr = *text;
        char_ptr++;
        text++;
    }
}

// Integer to string conversion
void int_to_str(int num, char* str) {
    int i = 0;
    int is_negative = 0;
    
    // Handle 0 explicitly
    if (num == 0) {
        str[0] = '0';
        str[1] = '\0';
        return;
    }
    
    // Handle negative numbers
    if (num < 0) {
        is_negative = 1;
        num = -num;
    }
    
    // Convert to string (reversed)
    while (num != 0) {
        int digit = num % 10;
        str[i++] = digit + '0';
        num = num / 10;
    }
    
    // Add negative sign if needed
    if (is_negative) {
        str[i++] = '-';
    }
    
    // Add null terminator
    str[i] = '\0';
    
    // Reverse the string
    int start = 0;
    int end = i - 1;
    while (start < end) {
        char temp = str[start];
        str[start] = str[end];
        str[end] = temp;
        start++;
        end--;
    }
}

// Update display with benchmark results
void update_display() {
    char str_buffer[30];
    
    // Clear screen
    for (int y = 0; y < 60; y++) {
        for (int x = 0; x < 80; x++) {
            *(VGA_CHAR_ptr + (y * 80 + x)) = ' ';
        }
    }
    
    // Display title
    write_to_vga(20, 1, "UART BENCHMARK RESULTS");
    
    // Display test type
    write_to_vga(2, 3, "Test Type: ");
    switch (test_type) {
        case TEST_TYPE_LATENCY:
            write_to_vga(13, 3, "Latency");
            break;
        case TEST_TYPE_THROUGHPUT:
            write_to_vga(13, 3, "Throughput");
            break;
        case TEST_TYPE_RELIABILITY:
            write_to_vga(13, 3, "Reliability");
            break;
    }
    
    // Display statistics
    write_to_vga(2, 5, "Packets Sent: ");
    int_to_str(NUM_PACKETS, str_buffer);
    write_to_vga(16, 5, str_buffer);
    
    write_to_vga(2, 6, "Bytes Sent: ");
    int_to_str(bytes_sent, str_buffer);
    write_to_vga(14, 6, str_buffer);
    
    write_to_vga(2, 7, "Bytes Received: ");
    int_to_str(bytes_received, str_buffer);
    write_to_vga(18, 7, str_buffer);
    
    // Display timing information
    if (test_type == TEST_TYPE_LATENCY) {
        write_to_vga(2, 9, "Avg Latency (us): ");
        int_to_str(total_time / (NUM_PACKETS - WARMUP_PACKETS), str_buffer);
        write_to_vga(20, 9, str_buffer);
        
        write_to_vga(2, 10, "Min Latency (us): ");
        int_to_str(min_time, str_buffer);
        write_to_vga(20, 10, str_buffer);
        
        write_to_vga(2, 11, "Max Latency (us): ");
        int_to_str(max_time, str_buffer);
        write_to_vga(20, 11, str_buffer);
    }
    
    // Display throughput
    if (test_type == TEST_TYPE_THROUGHPUT) {
        unsigned long bytes_per_second = (bytes_received * 1000000UL) / total_time;
        write_to_vga(2, 9, "Throughput (bytes/sec): ");
        int_to_str(bytes_per_second, str_buffer);
        write_to_vga(26, 9, str_buffer);
    }
    
    // Display error information
    write_to_vga(2, 13, "Packet Errors: ");
    int_to_str(packet_errors, str_buffer);
    write_to_vga(17, 13, str_buffer);
    
    // Display connection information
    write_to_vga(40, 3, "Connection:");
    write_to_vga(40, 4, "- DE1-SoC JP1 D0 -> Arduino pin 12");
    write_to_vga(40, 5, "- DE1-SoC JP1 D1 -> Arduino pin 13");
    write_to_vga(40, 6, "- Baud Rate: 115200");
    
    // Display instructions
    write_to_vga(2, 20, "SW[0]=0: Latency Test");
    write_to_vga(2, 21, "SW[1]=1: Throughput Test");
    write_to_vga(2, 22, "SW[2]=1: Reliability Test");
    write_to_vga(2, 23, "KEY[0]: Start New Test");
}

// Latency benchmark
void run_latency_benchmark() {
    unsigned char tx_buffer[1];
    int rx_data;
    unsigned long start_time, elapsed_time;
    
    test_type = TEST_TYPE_LATENCY;
    send_test_command(TEST_TYPE_LATENCY);
    
    // Wait for Arduino to be ready
    unsigned long ready_wait = 0;
    do {
        rx_data = uart_rx_byte(1000);
        ready_wait++;
    } while (rx_data != 'R' && ready_wait < 1000);
    
    if (ready_wait >= 1000) {
        // Timeout waiting for Arduino
        packet_errors = 9999;
        return;
    }
    
    // Reset statistics
    total_time = 0;
    min_time = 0xFFFFFFFF;
    max_time = 0;
    packet_errors = 0;
    bytes_received = 0;
    bytes_sent = 0;
    
    // Run test
    for (int i = 0; i < NUM_PACKETS; i++) {
        // Generate test byte
        tx_buffer[0] = (unsigned char)(i & 0xFF);
        
        // Start timer
        start_time = *(TIMER_ptr);
        
        // Send single byte
        uart_tx_byte(tx_buffer[0]);
        bytes_sent++;
        
        // Wait for echo
        rx_data = uart_rx_byte(100);
        
        // End timer
        elapsed_time = *(TIMER_ptr) - start_time;
        
        // Check for errors
        if (rx_data < 0 || (unsigned char)rx_data != tx_buffer[0]) {
            packet_errors++;
        } else {
            // Only count non-warmup packets in statistics
            if (i >= WARMUP_PACKETS) {
                total_time += elapsed_time;
                if (elapsed_time < min_time) min_time = elapsed_time;
                if (elapsed_time > max_time) max_time = elapsed_time;
            }
        }
        
        // Short delay between packets
        for (volatile int j = 0; j < 1000000; j++);
    }
    
    // Convert clock cycles to microseconds
    // Assuming 100MHz clock, 1 cycle = 10ns = 0.01us
    total_time /= 100;
    min_time /= 100;
    max_time /= 100;
}

// Throughput benchmark
void run_throughput_benchmark() {
    unsigned char tx_buffer[PACKET_SIZE];
    unsigned char rx_buffer[PACKET_SIZE];
    unsigned long start_time, elapsed_time;
    
    test_type = TEST_TYPE_THROUGHPUT;
    send_test_command(TEST_TYPE_THROUGHPUT);
    
    // Wait for Arduino to be ready
    int rx_data;
    unsigned long ready_wait = 0;
    do {
        rx_data = uart_rx_byte(1000);
        ready_wait++;
    } while (rx_data != 'R' && ready_wait < 1000);
    
    if (ready_wait >= 1000) {
        // Timeout waiting for Arduino
        packet_errors = 9999;
        return;
    }
    
    // Reset statistics
    total_time = 0;
    packet_errors = 0;
    bytes_received = 0;
    bytes_sent = 0;
    
    // Start timer for entire test
    start_time = *(TIMER_ptr);
    
    // Run test
    for (int i = 0; i < NUM_PACKETS; i++) {
        // Generate test packet
        generate_test_packet(tx_buffer, PACKET_SIZE, i);
        
        // Send packet
        for (int j = 0; j < PACKET_SIZE; j++) {
            uart_tx_byte(tx_buffer[j]);
            bytes_sent++;
        }
        
        // Update LEDs to show progress
        *LEDR_ptr = (1 << (i % 10));
    }
    
    // End timer
    elapsed_time = *(TIMER_ptr) - start_time;
    
    // Convert clock cycles to microseconds
    total_time = elapsed_time / 100;
}

// Reliability benchmark
void run_reliability_benchmark() {
    unsigned char tx_buffer[PACKET_SIZE];
    unsigned char rx_buffer[PACKET_SIZE];
    int rx_data;
    
    test_type = TEST_TYPE_RELIABILITY;
    send_test_command(TEST_TYPE_RELIABILITY);
    
    // Wait for Arduino to be ready
    unsigned long ready_wait = 0;
    do {
        rx_data = uart_rx_byte(1000);
        ready_wait++;
    } while (rx_data != 'R' && ready_wait < 1000);
    
    if (ready_wait >= 1000) {
        // Timeout waiting for Arduino
        packet_errors = 9999;
        return;
    }
    
    // Reset statistics
    packet_errors = 0;
    bytes_received = 0;
    bytes_sent = 0;
    
    // Run test (send packets and wait for echo)
    for (int i = 0; i < ECHO_TEST_PACKETS; i++) {
        // Generate test packet
        generate_test_packet(tx_buffer, PACKET_SIZE, i);
        
        // Send packet
        for (int j = 0; j < PACKET_SIZE; j++) {
            uart_tx_byte(tx_buffer[j]);
            bytes_sent++;
        }
        
        // Receive echo
        for (int j = 0; j < PACKET_SIZE; j++) {
            rx_data = uart_rx_byte(100);
            if (rx_data < 0) {
                // Timeout or framing error
                packet_errors++;
                break;
            }
            rx_buffer[j] = (unsigned char)rx_data;
        }
        
        // Validate received packet
        packet_errors += validate_packet(tx_buffer, rx_buffer, PACKET_SIZE);
        
        // Update LEDs to show progress
        *LEDR_ptr = (1 << (i % 10));
        
        // Short delay between packets
        for (volatile int j = 0; j < 1000000; j++);
    }
}

int main(void) {
    // Initialize pointers to I/O devices
    JP1_ptr = (int *)JP1_BASE;
    KEY_ptr = (int *)KEY_BASE;
    SW_ptr = (int *)SW_BASE;
    LEDR_ptr = (int *)LEDR_BASE;
    TIMER_ptr = (int *)TIMER_BASE;
    VGA_CHAR_ptr = (char *)FPGA_CHAR_BASE;
    
    // Initialize JP1 for UART
    init_jp1_uart();
    
    // Initialize display
    update_display();
    
    while (1) {
        // Read switch values to determine test type
        int sw_value = *SW_ptr;
        if (sw_value & 0x4) {
            test_type = TEST_TYPE_RELIABILITY;
        } else if (sw_value & 0x2) {
            test_type = TEST_TYPE_THROUGHPUT;
        } else {
            test_type = TEST_TYPE_LATENCY;
        }
        
        // Start test on KEY0 press
        if (*KEY_ptr & 0x1) {
            // Wait for key release
            while (*KEY_ptr & 0x1);
            
            // Run the selected benchmark
            switch (test_type) {
                case TEST_TYPE_LATENCY:
                    run_latency_benchmark();
                    break;
                case TEST_TYPE_THROUGHPUT:
                    run_throughput_benchmark();
                    break;
                case TEST_TYPE_RELIABILITY:
                    run_reliability_benchmark();
                    break;
            }
            
            // Update display with results
            update_display();
        }
    }
    
    return 0;
}