#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "address_map.h"

// Function test result structure
typedef struct {
    const char* feature_name;
    int is_available;
    char description[100];
} FeatureTest;

// Forward declarations for all test functions
int test_jp1_uart();
int test_key_buttons();
int test_switches();
int test_leds();
int test_timer();
int test_vga_display();
int test_math_functions();
int test_neural_network();

int main(void) {
    printf("Starting hardware and feature availability test...\n");
    printf("================================================\n\n");

    // Collect all features to test
    FeatureTest tests[] = {
        {"JP1 UART Port", 0, ""},
        {"Push Buttons (KEY)", 0, ""},
        {"Slider Switches (SW)", 0, ""},
        {"Red LEDs (LEDR)", 0, ""},
        {"Interval Timer", 0, ""},
        {"VGA Character Display", 0, ""},
        {"Math Library Functions", 0, ""},
        {"Neural Network Memory", 0, ""}
    };
    
    int num_tests = sizeof(tests) / sizeof(tests[0]);
    
    // Run all tests
    tests[0].is_available = test_jp1_uart();
    tests[1].is_available = test_key_buttons();
    tests[2].is_available = test_switches();
    tests[3].is_available = test_leds();
    tests[4].is_available = test_timer();
    tests[5].is_available = test_vga_display();
    tests[6].is_available = test_math_functions();
    tests[7].is_available = test_neural_network();
    
    // Add descriptions based on test results
    strcpy(tests[0].description, tests[0].is_available ? "JP1 port accessible for UART communication" : "Error accessing JP1 memory-mapped address");
    strcpy(tests[1].description, tests[1].is_available ? "Push buttons accessible for input" : "Error accessing KEY memory-mapped address");
    strcpy(tests[2].description, tests[2].is_available ? "Slider switches accessible for input" : "Error accessing SW memory-mapped address");
    strcpy(tests[3].description, tests[3].is_available ? "Red LEDs accessible for output" : "Error accessing LEDR memory-mapped address");
    strcpy(tests[4].description, tests[4].is_available ? "Interval timer accessible" : "Error accessing TIMER memory-mapped address");
    strcpy(tests[5].description, tests[5].is_available ? "VGA character buffer accessible" : "Error accessing VGA character buffer address");
    strcpy(tests[6].description, tests[6].is_available ? "Math library functions available" : "Math library functions missing");
    strcpy(tests[7].description, tests[7].is_available ? "Sufficient memory for neural network" : "Insufficient memory for neural network structures");
    
    // Print results
    printf("Test Results:\n");
    printf("================================================\n");
    int passed = 0;
    for (int i = 0; i < num_tests; i++) {
        printf("[%s] %s: %s\n", 
               tests[i].is_available ? "PASS" : "FAIL", 
               tests[i].feature_name,
               tests[i].description);
        if (tests[i].is_available) passed++;
    }
    
    printf("\n================================================\n");
    printf("Summary: %d/%d tests passed\n", passed, num_tests);
    
    // Check address map values in use
    printf("\nAddress Map Diagnostic:\n");
    printf("================================================\n");
    printf("JP1_BASE = 0x%X\n", JP1_BASE);
    printf("KEY_BASE = 0x%X\n", KEY_BASE);
    printf("SW_BASE = 0x%X\n", SW_BASE);
    printf("LEDR_BASE = 0x%X\n", LEDR_BASE);
    printf("TIMER_BASE = 0x%X\n", TIMER_BASE);
    printf("FPGA_CHAR_BASE = 0x%X\n", FPGA_CHAR_BASE);
    
    return 0;
}

// Test if JP1 UART is accessible
int test_jp1_uart() {
    volatile int *jp1_ptr;
    
    // Try to access the JP1 port
    printf("Testing JP1 UART accessibility...\n");
    
    // Use a try-catch-like approach with setjmp/longjmp if available
    // For simplicity, we'll just try to access and check if it crashes
    jp1_ptr = (int *)JP1_BASE;
    
    // If we can read from the address without crash, it might be accessible
    // This is a naive check that doesn't guarantee the hardware is actually there
    int value = -1;
    
    // Try to read - if segfault occurs, this won't complete
    value = *jp1_ptr;
    
    printf("  JP1 read value: 0x%X\n", value);
    
    // Try writing a value that shouldn't affect operation
    // Do a read-modify-write to preserve other bits
    int original = *jp1_ptr;
    *jp1_ptr = original;
    
    return 1; // If we made it here without crashing, assume success
}

// Test if KEY buttons are accessible
int test_key_buttons() {
    volatile int *key_ptr;
    
    printf("Testing KEY buttons accessibility...\n");
    key_ptr = (int *)KEY_BASE;
    
    // Try to read the current state
    int value = *key_ptr;
    printf("  KEY read value: 0x%X\n", value);
    printf("  If buttons are pressed, this value should change\n");
    
    return 1; // If we made it here without crashing, assume success
}

// Test if SW switches are accessible
int test_switches() {
    volatile int *sw_ptr;
    
    printf("Testing SW switches accessibility...\n");
    sw_ptr = (int *)SW_BASE;
    
    // Try to read the current state
    int value = *sw_ptr;
    printf("  SW read value: 0x%X\n", value);
    printf("  If switches are toggled, this value should change\n");
    
    return 1; // If we made it here without crashing, assume success
}

// Test if LEDs are accessible
int test_leds() {
    volatile int *ledr_ptr;
    
    printf("Testing LEDR accessibility...\n");
    ledr_ptr = (int *)LEDR_BASE;
    
    // Save original value
    int original = *ledr_ptr;
    
    // Try setting all LEDs on
    *ledr_ptr = 0xFFFFFFFF;
    printf("  All LEDs should be ON now\n");
    
    // Small delay
    for (volatile int i = 0; i < 10000000; i++);
    
    // Try setting all LEDs off
    *ledr_ptr = 0x0;
    printf("  All LEDs should be OFF now\n");
    
    // Small delay
    for (volatile int i = 0; i < 10000000; i++);
    
    // Restore original value
    *ledr_ptr = original;
    
    return 1; // If we made it here without crashing, assume success
}

// Test if timer is accessible
int test_timer() {
    volatile int *timer_ptr;
    
    printf("Testing interval timer accessibility...\n");
    timer_ptr = (int *)TIMER_BASE;
    
    // Try to read the current value
    int value = *timer_ptr;
    printf("  Timer value: 0x%X\n", value);
    
    // Read again to see if it changes (it should if it's a running timer)
    for (volatile int i = 0; i < 10000000; i++);
    int new_value = *timer_ptr;
    printf("  Timer new value: 0x%X\n", new_value);
    
    if (value != new_value) {
        printf("  Timer value changed - timer appears to be running\n");
    } else {
        printf("  Timer value unchanged - timer may not be running or may require initialization\n");
    }
    
    return 1; // If we made it here without crashing, assume success
}

// Test if VGA display is accessible
int test_vga_display() {
    volatile char *vga_char_ptr;
    
    printf("Testing VGA character buffer accessibility...\n");
    vga_char_ptr = (char *)FPGA_CHAR_BASE;
    
    // Try to write a test message to the first line
    char message[] = "VGA TEST - If you can see this on the VGA display, the test passed!";
    
    // Save original content
    char original[sizeof(message)];
    for (int i = 0; i < sizeof(message) - 1; i++) {
        original[i] = *(vga_char_ptr + i);
    }
    
    // Write test message
    for (int i = 0; i < sizeof(message) - 1; i++) {
        *(vga_char_ptr + i) = message[i];
    }
    
    printf("  Wrote test message to VGA display\n");
    
    // Small delay
    for (volatile int i = 0; i < 20000000; i++);
    
    // Restore original content
    for (int i = 0; i < sizeof(message) - 1; i++) {
        *(vga_char_ptr + i) = original[i];
    }
    
    return 1; // If we made it here without crashing, assume success
}

// Test math library functions used by the neural network
int test_math_functions() {
    printf("Testing math library functions...\n");
    
    // Test exp function
    float exp_test = exp(1.0f);
    printf("  exp(1.0) = %f (should be ~2.718)\n", exp_test);
    
    // Test log function
    float log_test = log(2.718f);
    printf("  log(2.718) = %f (should be ~1.0)\n", log_test);
    
    // Test sqrt function
    float sqrt_test = sqrt(4.0f);
    printf("  sqrt(4.0) = %f (should be 2.0)\n", sqrt_test);
    
    // Test fmin/fmax functions
    float min_test = fmin(3.0f, 5.0f);
    float max_test = fmax(3.0f, 5.0f);
    printf("  fmin(3.0, 5.0) = %f (should be 3.0)\n", min_test);
    printf("  fmax(3.0, 5.0) = %f (should be 5.0)\n", max_test);
    
    // If any of these functions are missing, compilation would fail
    // But let's also check if they're producing reasonable results
    if (fabs(exp_test - 2.718f) < 0.1f && 
        fabs(log_test - 1.0f) < 0.1f && 
        fabs(sqrt_test - 2.0f) < 0.001f &&
        fabs(min_test - 3.0f) < 0.001f &&
        fabs(max_test - 5.0f) < 0.001f) {
        return 1;
    } else {
        printf("  Math functions appear to be available but may not be working correctly\n");
        return 0;
    }
}

// Test if there's enough memory for the neural network structures
int test_neural_network() {
    printf("Testing neural network memory allocation...\n");
    
    // Define the dimensions from original code
    #define STATE_DIM 4
    #define ACTION_DIM 2
    #define HIDDEN_DIM 32
    #define MAX_TIMESTEPS 500
    
    // Try to allocate memory for the neural network
    printf("  Attempting to allocate memory for neural network...\n");
    
    // Test policy network allocation
    float (*weights1)[HIDDEN_DIM] = malloc(STATE_DIM * sizeof(float[HIDDEN_DIM]));
    float *bias1 = malloc(HIDDEN_DIM * sizeof(float));
    float (*weights2)[ACTION_DIM] = malloc(HIDDEN_DIM * sizeof(float[ACTION_DIM]));
    float *bias2 = malloc(ACTION_DIM * sizeof(float));
    
    // Test value network allocation
    float (*value_weights1)[HIDDEN_DIM] = malloc(STATE_DIM * sizeof(float[HIDDEN_DIM]));
    float *value_bias1 = malloc(HIDDEN_DIM * sizeof(float));
    float (*value_weights2)[1] = malloc(HIDDEN_DIM * sizeof(float[1]));
    float *value_bias2 = malloc(sizeof(float));
    
    // Test memory buffer allocation
    float (*states)[STATE_DIM] = malloc(MAX_TIMESTEPS * sizeof(float[STATE_DIM]));
    int *actions = malloc(MAX_TIMESTEPS * sizeof(int));
    float *rewards = malloc(MAX_TIMESTEPS * sizeof(float));
    float *values = malloc(MAX_TIMESTEPS * sizeof(float));
    float *log_probs = malloc(MAX_TIMESTEPS * sizeof(float));
    float *advantages = malloc(MAX_TIMESTEPS * sizeof(float));
    float *returns = malloc(MAX_TIMESTEPS * sizeof(float));
    
    // Check if all allocations succeeded
    if (weights1 && bias1 && weights2 && bias2 && 
        value_weights1 && value_bias1 && value_weights2 && value_bias2 &&
        states && actions && rewards && values && log_probs && advantages && returns) {
        printf("  Successfully allocated all neural network memory\n");
        
        // Free all allocated memory
        free(weights1);
        free(bias1);
        free(weights2);
        free(bias2);
        free(value_weights1);
        free(value_bias1);
        free(value_weights2);
        free(value_bias2);
        free(states);
        free(actions);
        free(rewards);
        free(values);
        free(log_probs);
        free(advantages);
        free(returns);
        
        return 1;
    } else {
        printf("  Failed to allocate neural network memory\n");
        
        // Free any successfully allocated memory
        if (weights1) free(weights1);
        if (bias1) free(bias1);
        if (weights2) free(weights2);
        if (bias2) free(bias2);
        if (value_weights1) free(value_weights1);
        if (value_bias1) free(value_bias1);
        if (value_weights2) free(value_weights2);
        if (value_bias2) free(value_bias2);
        if (states) free(states);
        if (actions) free(actions);
        if (rewards) free(rewards);
        if (values) free(values);
        if (log_probs) free(log_probs);
        if (advantages) free(advantages);
        if (returns) free(returns);
        
        return 0;
    }
}