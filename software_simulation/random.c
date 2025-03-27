#include <stdio.h>

// Memory-mapped register addresses from DE1-SoC documentation
#define MTIME_BASE 0xFF202100

// Interval timer register addresses
#define TIMER_BASE 0xFF202000
#define TIMER_STATUS      ((volatile unsigned short *)(TIMER_BASE))        // 0xFF202000
#define TIMER_CONTROL     ((volatile unsigned short *)(TIMER_BASE + 4))    // 0xFF202004
#define TIMER_START_LOW   ((volatile unsigned short *)(TIMER_BASE + 8))    // 0xFF202008
#define TIMER_START_HIGH  ((volatile unsigned short *)(TIMER_BASE + 12))   // 0xFF20200C
#define TIMER_SNAP_LOW    ((volatile unsigned short *)(TIMER_BASE + 16))   // 0xFF202010
#define TIMER_SNAP_HIGH   ((volatile unsigned short *)(TIMER_BASE + 20))   // 0xFF202014

#define LEDR_BASE  0xFF200000
#define HEX3_HEX0_BASE 0xFF200020

// Control register bit masks
#define TIMER_ITO    0x01    // Interrupt on timeout
#define TIMER_CONT   0x02    // Continuous mode
#define TIMER_START  0x04    // Start timer
#define TIMER_STOP   0x08    // Stop timer

// Define size for distribution visualization
#define NUM_BINS 10
#define NUM_SAMPLES 1000

// Function prototypes
void init_interval_timer(void);
unsigned int read_timer_value(void);
unsigned int xorshift32(unsigned int *seed);
void display_distribution(int bins[], int num_bins);
void update_leds(unsigned int value);
void display_hex(unsigned int value);

int main(void) {
    int i;
    int bins[NUM_BINS] = {0};
    
    // Initialize the interval timer
    init_interval_timer();
    printf("Interval Timer initialized\n");
    
    // Initialize the first seed from timer
    unsigned int seed = read_timer_value();
    printf("Initial seed from timer: 0x%08X\n", seed);
    
    // Mix entropy by running the generator a few times
    for (i = 0; i < 10; i++) {
        seed = xorshift32(&seed);
    }
    
    printf("Testing random distribution across %d samples:\n", NUM_SAMPLES);
    
    // Generate random numbers and count their distribution
    for (i = 0; i < NUM_SAMPLES; i++) {
        // Generate a new random number
        unsigned int rand_val = xorshift32(&seed);
        
        // Map to a bin (0-9) and increment count
        int bin = (rand_val % NUM_BINS);
        bins[bin]++;
        
        // Visualize on hardware every 50 iterations
        if (i % 50 == 0) {
            // Display current random value on LEDs
            update_leds(rand_val);
            
            // Display bin distribution on 7-segment display
            display_hex(bin);
        }
        
        // Print every 100th value
        if (i % 100 == 0) {
            printf("Sample %4d: 0x%08X (bin %d)\n", i, rand_val, bin);
        }
    }
    
    // Display the final distribution
    display_distribution(bins, NUM_BINS);
    
    // Test for sequential correlation
    printf("\nTesting for sequential patterns (10 pairs):\n");
    for (i = 0; i < 10; i++) {
        unsigned int val1 = xorshift32(&seed);
        unsigned int val2 = xorshift32(&seed);
        printf("Pair %d: 0x%08X -> 0x%08X\n", i, val1, val2);
    }
    
    // Test reading the timer multiple times to see the variation
    printf("\nInterval timer values (5 consecutive reads):\n");
    for (i = 0; i < 5; i++) {
        unsigned int timer_val = read_timer_value();
        printf("Timer reading %d: 0x%08X\n", i, timer_val);
        
        // Small delay
        for (volatile int j = 0; j < 1000000; j++);
    }
    printf("hello");
    return 0;
}

/**
 * Initialize the interval timer
 * 
 * Sets up the interval timer to run in continuous mode with a specific period
 */
void init_interval_timer(void) {
    // Stop timer if it's running
    *TIMER_CONTROL = TIMER_STOP;
    
    // Set timer period (use a prime number for better randomness)
    // This will be different from the default 12.5M value to avoid correlation
    unsigned int count_value = 9973651;  // Prime number less than 10M
    
    // Set the start value (low and high parts)
    *TIMER_START_LOW = count_value & 0xFFFF;
    *TIMER_START_HIGH = (count_value >> 16) & 0xFFFF;
    
    // Start the timer in continuous mode
    *TIMER_CONTROL = TIMER_START | TIMER_CONT;
}

/**
 * Read the timer value using the interval timer's snapshot capability
 * 
 * Based on the DE1-SoC documentation section 2.6, this properly reads
 * the 32-bit timer value by:
 * 1. Writing to the snapshot register
 * 2. Reading the low and high snapshot values
 */
unsigned int read_timer_value(void) {
    // Writing to TIMER_SNAP_LOW causes the counter value to be stored
    // in the TIMER_SNAP_LOW and TIMER_SNAP_HIGH registers
    *TIMER_SNAP_LOW = 0;  // Value doesn't matter, just the write operation
    
    // Now read the snapshot values
    unsigned int low = *TIMER_SNAP_LOW;
    unsigned int high = *TIMER_SNAP_HIGH;
    
    // Combine to form a 32-bit value
    unsigned int timer_value = (high << 16) | low;
    
    // For added randomness, we can also mix in the machine timer
    volatile unsigned int *mtime_ptr = (unsigned int *)MTIME_BASE;
    unsigned int mtime_low = *(mtime_ptr + 1);  // 0xFF202104
    
    // XOR the values for better randomness
    return timer_value ^ (mtime_low & 0xFFFF);
}

// XORShift32 random number generator
// This is the same algorithm used in the training script
unsigned int xorshift32(unsigned int *seed) {
    unsigned int x = *seed;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    
    // Additional mixing for better quality
    x ^= x << 3;
    x ^= x >> 7;
    
    *seed = x;
    return x;
}

// Display the distribution of random numbers
void display_distribution(int bins[], int num_bins) {
    int i, j;
    int max_count = 0;
    
    // Find maximum count for scaling
    for (i = 0; i < num_bins; i++) {
        if (bins[i] > max_count) {
            max_count = bins[i];
        }
    }
    
    // Print header
    printf("\nRandom number distribution (ideal: %d per bin):\n", NUM_SAMPLES / NUM_BINS);
    printf("--------------------------------------------------\n");
    
    // Print the histogram
    for (i = 0; i < num_bins; i++) {
        printf("Bin %2d [%3d]: ", i, bins[i]);
        
        // Calculate bar length (scaled to max 50 chars)
        int bar_length = (bins[i] * 50) / max_count;
        
        // Print the bar
        for (j = 0; j < bar_length; j++) {
            printf("#");
        }
        
        // Print percentage
        float percentage = (float)bins[i] / NUM_SAMPLES * 100.0f;
        printf(" (%.1f%%)\n", percentage);
    }
    
    // Calculate chi-square statistic for uniformity test
    float expected = (float)NUM_SAMPLES / num_bins;
    float chi_square = 0.0;
    
    for (i = 0; i < num_bins; i++) {
        float diff = bins[i] - expected;
        chi_square += (diff * diff) / expected;
    }
    
    printf("\nChi-square statistic: %.2f\n", chi_square);
    printf("For %d bins, values < 16.92 suggest good uniformity at 95%% confidence\n", num_bins);
}

// Update LEDs with random value
void update_leds(unsigned int value) {
    volatile unsigned int *led_ptr = (unsigned int *)LEDR_BASE;
    *led_ptr = value & 0x3FF; // Only use lower 10 bits for LEDR9-0
}

// Display a value on 7-segment display
void display_hex(unsigned int value) {
    volatile unsigned int *hex_ptr = (unsigned int *)HEX3_HEX0_BASE;
    
    // 7-segment patterns for digits 0-9
    static const unsigned char seven_seg[10] = {
        0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x67
    };
    
    // Display the value on HEX0
    *hex_ptr = seven_seg[value % 10];
}