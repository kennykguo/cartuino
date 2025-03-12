#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "address_map.h"

// Hyperparameters
#define STATE_DIM 4
#define ACTION_DIM 2
#define HIDDEN_DIM 32
// #define BATCH_SIZE 128
#define GAMMA 0.99f
#define LAMBDA 0.95f
#define CLIP_EPSILON 0.2f
#define LEARNING_RATE 0.001f
#define MAX_EPISODES 30
#define MAX_TIMESTEPS 500
#define PPO_EPOCHS 4

// Training/inference mode
#define MODE_TRAIN 0
#define MODE_INFERENCE 1

// JP1 UART Configuration - these define which pins we use on the JP1 port
// According to the DE1-SoC documentation, JP1's base address is 0xFF200060
// Direction register is at Base+4 (0xFF200064)
// D0 = RX pin (input to DE1-SoC) connected to Arduino's TX (pin 12)
// D1 = TX pin (output from DE1-SoC) connected to Arduino's RX (pin 13)
#define UART_RX_BIT 0x00000001  // Bit 0 for RX (D0)
#define UART_TX_BIT 0x00000002  // Bit 1 for TX (D1)
#define UART_BAUD_RATE 115200
#define CLOCK_RATE 100000000    // 100MHz DE1-SoC system clock
#define BIT_PERIOD (CLOCK_RATE / UART_BAUD_RATE)

// Neural Network Structure
typedef struct {
    // Policy network
    float weights1[STATE_DIM][HIDDEN_DIM];  // Input layer to hidden
    float bias1[HIDDEN_DIM];                // Hidden layer bias
    float weights2[HIDDEN_DIM][ACTION_DIM]; // Hidden to output
    float bias2[ACTION_DIM];                // Output bias
    
    // Value network
    float value_weights1[STATE_DIM][HIDDEN_DIM];
    float value_bias1[HIDDEN_DIM];
    float value_weights2[HIDDEN_DIM][1];
    float value_bias2[1];
} NeuralNetwork;

// Memory buffer to store experiences
typedef struct {
    float states[MAX_TIMESTEPS][STATE_DIM];
    int actions[MAX_TIMESTEPS];
    float rewards[MAX_TIMESTEPS];
    float values[MAX_TIMESTEPS];
    float log_probs[MAX_TIMESTEPS];
    float advantages[MAX_TIMESTEPS];
    float returns[MAX_TIMESTEPS];
    int size;
} Memory;

// Global variables
volatile int *JP1_ptr;      // Pointer to JP1 expansion port for UART
volatile int *KEY_ptr;      // Pointer to pushbutton KEYs
volatile int *SW_ptr;       // Pointer to slider switches
volatile int *LEDR_ptr;     // Pointer to red LEDs
volatile int *TIMER_ptr;    // Pointer to interval timer
volatile char *VGA_CHAR_ptr; // Pointer to VGA character buffer

NeuralNetwork nn;
Memory memory;
int current_mode = MODE_TRAIN;
int episode_count = 0;
int best_steps = 0;

// Forward declarations
void init_jp1_uart();
void bit_delay();
void half_bit_delay();
void uart_tx_byte(unsigned char data);
void uart_tx_string(char *str);
int uart_rx_byte(int timeout_ms);
float relu(float x);
void policy_forward(float state[STATE_DIM], float probs[ACTION_DIM]);
float value_forward(float state[STATE_DIM]);
int sample_action(float probs[ACTION_DIM]);
float log_prob(float probs[ACTION_DIM], int action);
void update_network();
void compute_advantages();
void init_network();
void send_action(int action);
int read_state(float state[STATE_DIM]);
float calculate_reward(float state[STATE_DIM], int done);
void normalize_state(float state[STATE_DIM]);
void write_to_vga(int x, int y, char* text);
void update_display(int episode, int steps, float reward);

// Initialize the JP1 port for UART communication
void init_jp1_uart() {
    // Set direction for D0 (RX) as input, D1 (TX) as output
    // In the Direction register at Base+4, 0=input, 1=output
    *(JP1_ptr + 1) = UART_TX_BIT;  // D1 as output, D0 as input
    
    // Set TX pin high (idle state for UART)
    *(JP1_ptr) = UART_TX_BIT;
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
}

// Wait for and receive one byte over the software UART
// Returns -1 on timeout or framing error
int uart_rx_byte(int timeout_ms) {
    int i;
    unsigned char rx_data = 0;
    int current_bit;
    int data = 0;
    
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

// ReLU activation function
float relu(float x) {
    return x > 0.0f ? x : 0.0f;
}

// Neural Network forward pass for policy
void policy_forward(float state[STATE_DIM], float probs[ACTION_DIM]) {
    float hidden[HIDDEN_DIM];
    int i, j;
    float sum_exp = 0.0f;
    float output[ACTION_DIM];
    
    // Input to hidden layer
    for (i = 0; i < HIDDEN_DIM; i++) {
        hidden[i] = 0.0f;
        for (j = 0; j < STATE_DIM; j++) {
            hidden[i] += state[j] * nn.weights1[j][i];
        }
        hidden[i] += nn.bias1[i];
        hidden[i] = relu(hidden[i]);  // ReLU activation
    }
    
    // Hidden to output layer
    for (i = 0; i < ACTION_DIM; i++) {
        output[i] = 0.0f;
        for (j = 0; j < HIDDEN_DIM; j++) {
            output[i] += hidden[j] * nn.weights2[j][i];
        }
        output[i] += nn.bias2[i];
    }
    
    // Softmax activation for probabilities
    for (i = 0; i < ACTION_DIM; i++) {
        probs[i] = exp(output[i]);
        sum_exp += probs[i];
    }
    for (i = 0; i < ACTION_DIM; i++) {
        probs[i] /= sum_exp;
    }
}

// Neural Network forward pass for value function
float value_forward(float state[STATE_DIM]) {
    float hidden[HIDDEN_DIM];
    int i, j;
    float value = 0.0f;
    
    // Input to hidden layer
    for (i = 0; i < HIDDEN_DIM; i++) {
        hidden[i] = 0.0f;
        for (j = 0; j < STATE_DIM; j++) {
            hidden[i] += state[j] * nn.value_weights1[j][i];
        }
        hidden[i] += nn.value_bias1[i];
        hidden[i] = relu(hidden[i]);  // ReLU activation
    }
    
    // Hidden to output layer
    for (j = 0; j < HIDDEN_DIM; j++) {
        value += hidden[j] * nn.value_weights2[j][0];
    }
    value += nn.value_bias2[0];
    return value;
}

// Sample action from policy
int sample_action(float probs[ACTION_DIM]) {
    float r = (float)rand() / RAND_MAX;
    if (r < probs[0]) return 0;  // Left
    return 1;                    // Right
}

// Calculate log probability of action
float log_prob(float probs[ACTION_DIM], int action) {
    return log(probs[action]);
}

// Update neural network parameters using simple gradient descent
void update_network() {
    float dw1[STATE_DIM][HIDDEN_DIM] = {0};
    float db1[HIDDEN_DIM] = {0};
    float dw2[HIDDEN_DIM][ACTION_DIM] = {0};
    float db2[ACTION_DIM] = {0};
    float vdw1[STATE_DIM][HIDDEN_DIM] = {0};
    float vdb1[HIDDEN_DIM] = {0};
    float vdw2[HIDDEN_DIM][1] = {0};
    float vdb2[1] = {0};
    int epoch, t, i, j;
    
    // Perform multiple epochs of updates
    for (epoch = 0; epoch < PPO_EPOCHS; epoch++) {
        // Process all experiences in memory
        for (t = 0; t < memory.size; t++) {
            float state[STATE_DIM];
            float probs[ACTION_DIM];
            float current_log_prob, ratio, surrogate1, surrogate2;
            float policy_loss, value, value_diff, value_loss, loss;
            
            memcpy(state, memory.states[t], sizeof(float) * STATE_DIM);
            
            // Forward pass for current policy
            policy_forward(state, probs);
            
            // Get log probability of the action that was taken
            current_log_prob = log_prob(probs, memory.actions[t]);
            
            // Calculate ratio for PPO
            ratio = exp(current_log_prob - memory.log_probs[t]);
            
            // Calculate surrogate losses
            surrogate1 = ratio * memory.advantages[t];
            surrogate2 = fmin(fmax(ratio, 1.0f - CLIP_EPSILON), 1.0f + CLIP_EPSILON) * memory.advantages[t];
            
            // Policy loss (negative because we want to maximize it)
            policy_loss = -fmin(surrogate1, surrogate2);
            
            // Value loss
            value = value_forward(state);
            value_diff = value - memory.returns[t];
            value_loss = 0.5f * value_diff * value_diff;
            
            // Combined loss
            loss = policy_loss + 0.5f * value_loss;
            
            // Simple gradient approximation
            for (i = 0; i < STATE_DIM; i++) {
                for (j = 0; j < HIDDEN_DIM; j++) {
                    dw1[i][j] += state[i] * loss * 0.01f;
                    vdw1[i][j] += state[i] * value_loss * 0.01f;
                }
            }
        }
    }
    
    // Apply gradients to network parameters
    for (i = 0; i < STATE_DIM; i++) {
        for (j = 0; j < HIDDEN_DIM; j++) {
            nn.weights1[i][j] -= LEARNING_RATE * dw1[i][j] / memory.size;
            nn.value_weights1[i][j] -= LEARNING_RATE * vdw1[i][j] / memory.size;
        }
    }
    
    for (i = 0; i < HIDDEN_DIM; i++) {
        nn.bias1[i] -= LEARNING_RATE * db1[i] / memory.size;
        nn.value_bias1[i] -= LEARNING_RATE * vdb1[i] / memory.size;
        
        for (j = 0; j < ACTION_DIM; j++) {
            nn.weights2[i][j] -= LEARNING_RATE * dw2[i][j] / memory.size;
        }
        
        nn.value_weights2[i][0] -= LEARNING_RATE * vdw2[i][0] / memory.size;
    }
    
    for (i = 0; i < ACTION_DIM; i++) {
        nn.bias2[i] -= LEARNING_RATE * db2[i] / memory.size;
    }
    
    nn.value_bias2[0] -= LEARNING_RATE * vdb2[0] / memory.size;
}

// Calculate advantages and returns for PPO
void compute_advantages() {
    float gae = 0.0f;
    int t;
    
    for (t = memory.size - 1; t >= 0; t--) {
        float next_value = (t == memory.size - 1) ? 0.0f : memory.values[t + 1];
        float delta = memory.rewards[t] + GAMMA * next_value - memory.values[t];
        gae = delta + GAMMA * LAMBDA * gae;
        memory.advantages[t] = gae;
        memory.returns[t] = memory.advantages[t] + memory.values[t];
    }
}

// Initialize the neural network with small random weights
void init_network() {
    int i, j;
    
    // Get current time for random seed
    srand(*(TIMER_ptr));

    // Xavier initialization for weights
    float init_range1 = sqrt(6.0f / (STATE_DIM + HIDDEN_DIM));
    float init_range2 = sqrt(6.0f / (HIDDEN_DIM + ACTION_DIM));
    float init_range3 = sqrt(6.0f / (HIDDEN_DIM + 1));
    
    for (i = 0; i < STATE_DIM; i++) {
        for (j = 0; j < HIDDEN_DIM; j++) {
            nn.weights1[i][j] = (2.0f * ((float)rand() / RAND_MAX) - 1.0f) * init_range1;
            nn.value_weights1[i][j] = (2.0f * ((float)rand() / RAND_MAX) - 1.0f) * init_range1;
        }
    }
    
    for (i = 0; i < HIDDEN_DIM; i++) {
        nn.bias1[i] = 0.0f;
        nn.value_bias1[i] = 0.0f;
        
        for (j = 0; j < ACTION_DIM; j++) {
            nn.weights2[i][j] = (2.0f * ((float)rand() / RAND_MAX) - 1.0f) * init_range2;
        }
        
        nn.value_weights2[i][0] = (2.0f * ((float)rand() / RAND_MAX) - 1.0f) * init_range3;
    }
    
    for (i = 0; i < ACTION_DIM; i++) {
        nn.bias2[i] = 0.0f;
    }
    
    nn.value_bias2[0] = 0.0f;
}

// Send an action to the Arduino via JP1 UART
// Need to ensure consistency with Arduino code!
void send_action(int action) {
    // Convert action (0,1) to motor command (-1,1)
    int motor_command = action == 0 ? -1 : 1;
    
    // Special command for reset
    if (action == 2) motor_command = 0;  // Reset signal
    
    // Format command as string
    char command[4];
    if (motor_command < 0) {
        sprintf(command, "-1\n");
    } else if (motor_command > 0) {
        sprintf(command, "1\n");
    } else {
        sprintf(command, "0\n");
    }
    // Send the command string via UART
    uart_tx_string(command);
}

// Read state from Arduino via JP1 UART
int read_state(float state[STATE_DIM]) {
    char buffer[100] = {0};
    int index = 0;
    int done = 0;
    int timeout_counter = 0;
    int c;
    
    // Read characters until we get a newline or buffer is full
    while (index < 99) {
        // Read a character with timeout (100ms)
        c = uart_rx_byte(100);
        
        // Check for timeout or error
        if (c < 0) {
            timeout_counter++;
            if (timeout_counter > 50) {  // Allow multiple retries before giving up
                return 1;  // Signal timeout/failure
            }
            continue;
        }
        
        // If we got a newline, we're done
        if (c == '\n') {
            buffer[index] = '\0';
            break;
        }
        
        // Otherwise add the character to our buffer
        buffer[index++] = (char)c;
    }
    
    // Parse the CSV data using sscanf
    // Format from Arduino: cart_pos,cart_vel,pole_angle,pole_ang_vel,done
    if (sscanf(buffer, "%f,%f,%f,%f,%d", 
               &state[0], &state[1], &state[2], &state[3], &done) != 5) {
        // Failed to parse all fields
        return 1;
    }
    
    return done;
}

// Calculate reward based on state
float calculate_reward(float state[STATE_DIM], int done) {
    // Extract variables
    float cart_position = state[0];
    float cart_velocity = state[1];
    float pole_angle = state[2];
    float pole_angular_velocity = state[3];
    
    // If episode is done due to failure, give negative reward
    if (done && (fabs(pole_angle) > 0.2f || fabs(cart_position) > 2.4f)) {
        return -10.0f;
    }
    
    // Otherwise give a positive reward based on how centered the cart and pole are
    float angle_reward = 1.0f - fabs(pole_angle) / 0.2f;  // Max at center, min at failure threshold
    float position_reward = 1.0f - fabs(cart_position) / 2.4f;  // Max at center, min at failure threshold
    
    return angle_reward + position_reward;
}

// Normalize state variables
// Need to check these values actually with the program
void normalize_state(float state[STATE_DIM]) {
    // Normalize position: typical range [-2.4, 2.4] to [-1, 1]
    state[0] /= 2.4f;
    
    // Normalize velocity: clip to [-10, 10] and normalize to [-1, 1]
    state[1] = fmax(-10.0f, fmin(10.0f, state[1])) / 10.0f;
    
    // Normalize angle: typical range [-0.2, 0.2] to [-1, 1]
    state[2] /= 0.2f;
    
    // Normalize angular velocity: clip to [-10, 10] and normalize to [-1, 1]
    state[3] = fmax(-10.0f, fmin(10.0f, state[3])) / 10.0f;
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

// Update the VGA display with training stats
void update_display(int episode, int steps, float reward) {
    char buffer[80];
    int y, x;
    
    // Clear screen
    for (y = 0; y < 60; y++) {
        for (x = 0; x < 80; x++) {
            *(VGA_CHAR_ptr + (y * 80 + x)) = ' ';
        }
    }
    
    // Write header
    write_to_vga(25, 1, "CARTPOLE RL TRAINING");
    
    // Write statistics
    sprintf(buffer, "Episode: %d", episode);
    write_to_vga(2, 3, buffer);
    
    sprintf(buffer, "Steps: %d", steps);
    write_to_vga(2, 4, buffer);
    
    sprintf(buffer, "Reward: %.2f", reward);
    write_to_vga(2, 5, buffer);
    
    sprintf(buffer, "Mode: %s", current_mode == MODE_TRAIN ? "TRAINING" : "INFERENCE");
    write_to_vga(2, 6, buffer);
    
    // Write connection info
    write_to_vga(40, 3, "Wiring Guide:");
    write_to_vga(40, 4, "DE1-SoC JP1 D0 -> Arduino pin 12");
    write_to_vga(40, 5, "DE1-SoC JP1 D1 -> Arduino pin 13");
    write_to_vga(40, 6, "DE1-SoC GND -> Arduino GND");
    
    // Write instructions
    write_to_vga(2, 8, "SW[0]=0: Training Mode");
    write_to_vga(2, 9, "SW[0]=1: Inference Mode");
    write_to_vga(2, 10, "KEY[0]: Reset/Start New Episode");
    
    // Write best performance
    sprintf(buffer, "Best Steps: %d", best_steps);
    write_to_vga(2, 12, buffer);
}


// May need to add buffer times to devices
int main(void) {
    printf("Initializing pointers to I/O devices...\n");
    JP1_ptr = (int *)JP1_BASE;
    KEY_ptr = (int *)KEY_BASE;
    SW_ptr = (int *)SW_BASE;
    LEDR_ptr = (int *)LEDR_BASE;
    TIMER_ptr = (int *)TIMER_BASE;
    VGA_CHAR_ptr = (char *)FPGA_CHAR_BASE;
    
    printf("Initializing JP1 expansion port for UART...\n");
    init_jp1_uart();
    
    printf("Initializing neural network...\n");
    init_network();
    
    printf("Initializing VGA display...\n");
    update_display(0, 0, 0.0f);
    
    while (1) {
        int sw_value = *SW_ptr;
        current_mode = (sw_value & 0x1) ? MODE_INFERENCE : MODE_TRAIN;
        printf("Current mode: %s\n", current_mode == MODE_TRAIN ? "TRAIN" : "INFERENCE");
        
        *LEDR_ptr = current_mode;
        
        int key_value = *KEY_ptr;
        if (key_value & 0x1) {
            printf("KEY[0] pressed. Starting new episode...\n");
            while (*KEY_ptr & 0x1);
            
            float total_reward = 0.0f;
            int total_steps = 0;
            
            printf("Resetting environment...\n");
            send_action(2);
            
            if (current_mode == MODE_TRAIN) {
                memory.size = 0;
            }
            
            for (int i = 0; i < 1000000; i++);
            
            float state[STATE_DIM];
            int done = read_state(state);
            normalize_state(state);
            
            if (!done) {
                while (!done && total_steps < MAX_TIMESTEPS) {
                    *LEDR_ptr = 1 << (total_steps % 10);
                    printf("Step %d: Fetching action...\n", total_steps);
                    
                    float probs[ACTION_DIM];
                    policy_forward(state, probs);
                    
                    int action = (current_mode == MODE_TRAIN) ? sample_action(probs) : (probs[1] > probs[0] ? 1 : 0);
                    printf("Action taken: %d\n", action);
                    
                    send_action(action);
                    
                    // Get the next state to compute the reward
                    float next_state[STATE_DIM];
                    done = read_state(next_state);
                    normalize_state(next_state);
                    float reward = calculate_reward(next_state, done);
                    total_reward += reward;
                    
                    if (current_mode == MODE_TRAIN) {
                        float value = value_forward(state);
                        memcpy(memory.states[memory.size], state, sizeof(float) * STATE_DIM);
                        memory.actions[memory.size] = action;
                        memory.rewards[memory.size] = reward;
                        memory.values[memory.size] = value;
                        memory.log_probs[memory.size] = log_prob(probs, action);
                        memory.size++;
                    }
                    
                    // Copy over the next_state variable to the current state
                    memcpy(state, next_state, sizeof(float) * STATE_DIM);
                    total_steps++;
                    
                    // Check on training progress
                    // Should update the update_display function to display the percentage of dead neurons, or exploding gradients
                    if (total_steps % 10 == 0 || done) {
                        printf("Updating display: Episode %d, Steps %d, Total Reward %.2f\n", episode_count, total_steps, total_reward);
                        update_display(episode_count, total_steps, total_reward);
                    }
                }
                // The model ended the episode by failing here
                // Need to implement functionality where if the model goes too far left or right, then it automatically fails

                if (total_steps > best_steps) {
                    best_steps = total_steps;
                }
                
                if (current_mode == MODE_TRAIN) {
                    printf("Computing advantages and updating network...\n");
                    compute_advantages();
                    update_network();
                    episode_count++;
                }
                
                printf("Final update: Episode %d, Steps %d, Total Reward %.2f\n", episode_count, total_steps, total_reward);
                update_display(episode_count, total_steps, total_reward);
                
                // Buffer time
                for (int i = 0; i < 2000000; i++);
            }
        }
    }
    
    return 0;
}
