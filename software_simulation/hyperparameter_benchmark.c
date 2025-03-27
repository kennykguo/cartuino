#include <stdlib.h>
#include <math.h>
#include <stddef.h>  // for null def.
#define PIXEL_BUF_CTRL_BASE 0xFF203020
#define KEY_BASE            0xFF200050
#define LEDR_BASE           0xFF200000
#define HEX3_HEX0_BASE      0xFF200020
#define TIMER_BASE          0xFF202000
#define MTIME_BASE          0xFF202100

// Interval timer register definitions
#define TIMER_STATUS      ((volatile unsigned short *)(TIMER_BASE))        // 0xFF202000
#define TIMER_CONTROL     ((volatile unsigned short *)(TIMER_BASE + 4))    // 0xFF202004
#define TIMER_START_LOW   ((volatile unsigned short *)(TIMER_BASE + 8))    // 0xFF202008
#define TIMER_START_HIGH  ((volatile unsigned short *)(TIMER_BASE + 12))   // 0xFF20200C
#define TIMER_SNAP_LOW    ((volatile unsigned short *)(TIMER_BASE + 16))   // 0xFF202010
#define TIMER_SNAP_HIGH   ((volatile unsigned short *)(TIMER_BASE + 20))   // 0xFF202014

// Timer control register bit masks
#define TIMER_ITO    0x01    // Interrupt on timeout
#define TIMER_CONT   0x02    // Continuous mode
#define TIMER_START  0x04    // Start timer
#define TIMER_STOP   0x08    // Stop timer

#define VGA_WIDTH 320
#define VGA_HEIGHT 240
#define LINE_COLOUR 0x1E3D

// SIMULATION PARAMETERS
#define GRAVITY 9.8f
#define CART_MASS 1.0f
#define POLE_MASS 0.1f
#define POLE_HALF_LENGTH 0.5f
#define FORCE_MAG 20.0f
#define TIME_STEP 0.02f
#define RANDOM_FORCE_MAX 0.5f
#define PIXEL_SCALE 50.0f
#define MAX_ANGLE_RAD 0.75

// VGA DISPLAY PARAMETERS
#define CART_WIDTH 40
#define CART_HEIGHT 20
#define POLE_WIDTH 6
#define POLE_LENGTH 100

// COLOUR DEFINITIONS
#define BLACK 0x0000
#define WHITE 0xFFFF
#define RED 0xF800
#define GREEN 0x07E0
#define BLUE 0x001F
#define YELLOW 0xFFE0
#define CYAN 0x07FF
#define MAGENTA 0xF81F

// STATE DIMENSIONS
#define STATE_DIM 4
#define ACTION_DIM 2
#define HIDDEN_DIM 64

// BENCHMARK PARAMETERS
#define MAX_EPISODES 40     // Run each hyperparameter set for 40 episodes
#define MAX_STEPS_PER_EPOCH 300
#define MASTER_SEED 42      // For reproducibility
#define NUM_HYPERPARAMS 18  // Number of hyperparameter combinations to test

// TRAINING/INFERENCES MODES
#define MODE_TRAIN 0
#define MODE_INFERENCE 1

// Hyperparameter structure for testing
typedef struct {
    int ppo_epochs;
    float gamma;
    float lambda;
    float clip_epsilon;
    float learning_rate;
    float explore_rate;
} Hyperparameters;

typedef struct {
    float cart_position;
    float cart_velocity;
    float pole_angle;       // radians, 0 = upright
    float pole_angular_vel; // angular velocity of pole (radians/second)
} CartPoleState;

typedef struct {
    int epoch;
    int step;
    int total_reward;
    int best_reward;
    int best_episode;
} PerformanceStats;

typedef struct {
    // Policy network
    float weights1[STATE_DIM][HIDDEN_DIM];        // Input layer to first hidden
    float bias1[HIDDEN_DIM];                      // First hidden layer bias
    float weights_h2h[HIDDEN_DIM][HIDDEN_DIM];    // First hidden to second hidden
    float bias_h2[HIDDEN_DIM];                    // Second hidden layer bias
    float weights2[HIDDEN_DIM][ACTION_DIM];       // Second hidden to output
    float bias2[ACTION_DIM];                      // Output bias

    // Value network
    float value_weights1[STATE_DIM][HIDDEN_DIM];          // Input to first hidden
    float value_bias1[HIDDEN_DIM];                        // First hidden bias
    float value_weights_h2h[HIDDEN_DIM][HIDDEN_DIM];      // First hidden to second hidden
    float value_bias_h2[HIDDEN_DIM];                      // Second hidden bias
    float value_weights2[HIDDEN_DIM][1];                  // Second hidden to output
    float value_bias2[1];                                 // Output bias
} NeuralNetwork;

typedef struct {
    float states[MAX_STEPS_PER_EPOCH][STATE_DIM];
    float next_states[MAX_STEPS_PER_EPOCH][STATE_DIM];
    int actions[MAX_STEPS_PER_EPOCH];
    float rewards[MAX_STEPS_PER_EPOCH];
    float values[MAX_STEPS_PER_EPOCH];
    float log_probs[MAX_STEPS_PER_EPOCH];
    float advantages[MAX_STEPS_PER_EPOCH];
    float returns[MAX_STEPS_PER_EPOCH];
    int dones[MAX_STEPS_PER_EPOCH]; // 1 if next state is terminal
    int size;
} Memory;

// Benchmark result structure
typedef struct {
    Hyperparameters params;
    int best_reward;
    int best_episode;
} BenchmarkResult;

// GLOBAL VARIABLES
volatile int *KEY_ptr;
volatile int *LEDR_ptr;
volatile unsigned int *HEX3_HEX0_ptr;
volatile int timer_uses = 0;  // Count times timer is used for randomness

// SIMULATION STATE
CartPoleState state;
PerformanceStats stats;
NeuralNetwork nn;
Memory memory;
unsigned int random_seed;

// FUNCTION PROTOTYPES
void init_interval_timer(void);
unsigned int timer_value(void);
unsigned int xorshift32(void);
float random_float(void);
int random_int(int min, int max);
float random_range(float min, float max);

void update_physics(int action);
float calculate_reward();
int is_terminal_state();
void apply_random_disturbance();
float my_abs(float x);
float my_max(float a, float b);
float my_min(float a, float b);
void my_memcpy(void *dest, void *src, int n);
void policy_forward(float state[STATE_DIM], float probs[ACTION_DIM]);
float value_forward(float state[STATE_DIM]);
float log_prob(float probs[ACTION_DIM], int action);
void compute_advantages(float gamma, float lambda);
void update_network(Hyperparameters *params);
void init_network();
void normalize_state(float normalized_state[STATE_DIM], CartPoleState *cart_state);
float my_clamp(float value, float min, float max);
int sample_action(float probs[ACTION_DIM]);
void run_benchmark();
float my_sqrt(float x);
float my_exp(float x);
float my_log(float x);
float leaky_relu(float x);

// MAIN FUNCTION
int main(void) {
    KEY_ptr = (int *)KEY_BASE;
    LEDR_ptr = (int *)LEDR_BASE;
    HEX3_HEX0_ptr = (unsigned int *)HEX3_HEX0_BASE;
    
    // Initialize the interval timer with proper settings
    init_interval_timer();
    
    // Use timer value for initial seeding, properly mixed
    random_seed = timer_value() ^ MASTER_SEED;
    
    // Additional entropy mixing
    for (int i = 0; i < 10; i++) {
        random_seed = xorshift32();
    }

    // Run the hyperparameter benchmark
    run_benchmark();

    return 0;
}

/**
 * Initialize the interval timer for random number generation
 * 
 * Sets up the interval timer to run in continuous mode with a specific period
 * based on the documentation in section 2.6
 */
void init_interval_timer(void) {
    // Stop timer if it's running
    *TIMER_CONTROL = TIMER_STOP;
    
    // Set timer period to a prime number for better randomness
    // Instead of default 12.5M value
    unsigned int count_value = 9973651;  // Prime number < 10M
    
    // Set the start value (low and high parts)
    *TIMER_START_LOW = count_value & 0xFFFF;
    *TIMER_START_HIGH = (count_value >> 16) & 0xFFFF;
    
    // Start the timer in continuous mode
    *TIMER_CONTROL = TIMER_START | TIMER_CONT;
}

/**
 * Read the timer value using the interval timer's snapshot capability
 * 
 * Based on section 2.6 documentation, this properly reads the current
 * 32-bit counter value by triggering a snapshot and reading both halves
 */
unsigned int timer_value(void) {
    // Increment usage counter (for debugging)
    timer_uses++;
    
    // Write to TIMER_SNAP_LOW to capture the current count
    *TIMER_SNAP_LOW = 0;  // Value doesn't matter, just the write operation
    
    // Now read the snapshot values
    unsigned int low = *TIMER_SNAP_LOW;
    unsigned int high = *TIMER_SNAP_HIGH;
    
    // Combine to form a 32-bit value
    unsigned int timer_value = (high << 16) | low;
    
    // For added randomness, we can also mix in the machine timer
    volatile unsigned int *mtime_ptr = (unsigned int *)MTIME_BASE;
    unsigned int mtime_low = *(mtime_ptr + 1);  // 0xFF202104 (mtime low)
    
    // Use XOR to mix values without losing entropy
    return timer_value ^ (mtime_low & 0xFFFF);
}

/**
 * Enhanced XORshift32 random number generator
 * Periodically refreshes entropy from hardware timer
 * 
 * @return A 32-bit random number
 */
unsigned int xorshift32(void) {
    // Every 64 calls, mix in new timer entropy
    if ((random_seed & 0x3F) == 0) {
        random_seed ^= timer_value();
    }
    
    // Standard XORshift algorithm
    random_seed ^= random_seed << 13;
    random_seed ^= random_seed >> 17;
    random_seed ^= random_seed << 5;
    
    // Additional mixing
    random_seed ^= random_seed << 3;
    random_seed ^= random_seed >> 7;
    
    return random_seed;
}

/**
 * Get a random float between 0 and 1
 * 
 * @return Float between 0.0 and 1.0
 */
float random_float(void) {
    return (float)xorshift32() / 4294967295.0f;
}

/**
 * Get a random float in a specific range
 * 
 * @param min Minimum value (inclusive)
 * @param max Maximum value (inclusive)
 * @return Random float between min and max
 */
float random_range(float min, float max) {
    return min + random_float() * (max - min);
}

/**
 * Get a random integer in a specific range
 * 
 * @param min Minimum value (inclusive)
 * @param max Maximum value (inclusive)
 * @return Random integer between min and max
 */
int random_int(int min, int max) {
    return min + (xorshift32() % (max - min + 1));
}

// Run a benchmark of different hyperparameter combinations
void run_benchmark() {
    // Define the hyperparameter combinations to test
    Hyperparameters param_sets[NUM_HYPERPARAMS] = {
        // Variations of PPO_EPOCHS
        {4, 0.98f, 0.95f, 0.1f, 0.006f, 0.25f},
        {8, 0.98f, 0.95f, 0.1f, 0.006f, 0.25f},   // baseline
        {16, 0.98f, 0.95f, 0.1f, 0.006f, 0.25f},
        
        // Variations of GAMMA
        {8, 0.95f, 0.95f, 0.1f, 0.006f, 0.25f},
        {8, 0.97f, 0.95f, 0.1f, 0.006f, 0.25f},
        {8, 0.99f, 0.95f, 0.1f, 0.006f, 0.25f},
        
        // Variations of LAMBDA
        {8, 0.98f, 0.9f, 0.1f, 0.006f, 0.25f},
        {8, 0.98f, 0.97f, 0.1f, 0.006f, 0.25f},
        {8, 0.98f, 0.99f, 0.1f, 0.006f, 0.25f},
        
        // Variations of CLIP_EPSILON
        {8, 0.98f, 0.95f, 0.05f, 0.006f, 0.25f},
        {8, 0.98f, 0.95f, 0.2f, 0.006f, 0.25f},
        {8, 0.98f, 0.95f, 0.3f, 0.006f, 0.25f},
        
        // Variations of LEARNING_RATE
        {8, 0.98f, 0.95f, 0.1f, 0.002f, 0.25f},
        {8, 0.98f, 0.95f, 0.1f, 0.01f, 0.25f},
        {8, 0.98f, 0.95f, 0.1f, 0.015f, 0.25f},
        
        // Variations of EXPLORE_RATE
        {8, 0.98f, 0.95f, 0.1f, 0.006f, 0.15f},
        {8, 0.98f, 0.95f, 0.1f, 0.006f, 0.35f},
        {8, 0.98f, 0.95f, 0.1f, 0.006f, 0.5f}
    };
    
    // Track overall best result
    BenchmarkResult best_result = {
        {0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // Placeholder values
        -10000,                             // Initialize with very low reward
        0
    };

    // Test each hyperparameter set
    for (int param_idx = 0; param_idx < NUM_HYPERPARAMS; param_idx++) {
        Hyperparameters current_params = param_sets[param_idx];
        
        // Reset the seed for reproducibility across tests
        // Use timer value for consistent but unique seeding
        random_seed = timer_value() ^ MASTER_SEED ^ param_idx;
        for (int i = 0; i < 10; i++) {
            random_seed = xorshift32();
        }
        
        // Initialize neural network
        init_network();
        
        // Reset statistics
        stats.epoch = 0;
        stats.step = 0;
        stats.total_reward = 0;
        stats.best_reward = -10000;
        stats.best_episode = 0;
        
        // Reset memory
        memory.size = 0;
        
        // Run for MAX_EPISODES episodes
        while (stats.epoch < MAX_EPISODES) {
            // Reset state with small random perturbation for variety
            // Using new random number functions
            state.cart_position = random_range(-0.05f, 0.05f);
            state.cart_velocity = random_range(-0.05f, 0.05f);
            state.pole_angle = random_range(-0.05f, 0.05f);
            state.pole_angular_vel = random_range(-0.025f, 0.025f);
            
            stats.step = 0;
            stats.total_reward = 0;
            memory.size = 0;
            
            // Episode loop
            while (!is_terminal_state() && stats.step < MAX_STEPS_PER_EPOCH) {
                // Choose an action using policy network
                float normalized_state[STATE_DIM];
                normalize_state(normalized_state, &state);
                
                float action_probs[ACTION_DIM];
                policy_forward(normalized_state, action_probs);
                
                // Sample action for training
                float explore_rate = my_max(current_params.explore_rate, 
                               0.5f * (1.0f - (float)stats.epoch / 30.0f));
                
                int action;
                if (random_float() < explore_rate) {
                    // Random action with improved randomness
                    action = random_int(0, ACTION_DIM - 1);
                } else {
                    // Sample from policy distribution
                    action = sample_action(action_probs);
                }
                
                // Store experience in memory buffer
                if (memory.size < MAX_STEPS_PER_EPOCH) {
                    // Store current state
                    my_memcpy(memory.states[memory.size], normalized_state, sizeof(float) * STATE_DIM);
                    
                    // Store log probability of chosen action
                    memory.log_probs[memory.size] = log_prob(action_probs, action);
                    
                    // Store action
                    memory.actions[memory.size] = action;
                    
                    // Store value estimate
                    memory.values[memory.size] = value_forward(normalized_state);
                }
                
                // Apply occasional random disturbance
                if (stats.step % 20 == 0 && random_float() < 0.2f * (1.0f - (float)stats.epoch / 30.0f)) {
                    apply_random_disturbance();
                }
                
                // Update physics based on action
                update_physics(action);
                
                // Calculate reward
                float reward = calculate_reward();
                
                // Update total reward
                stats.total_reward += (int)reward;
                
                // Normalize the next state
                float next_normalized_state[STATE_DIM];
                normalize_state(next_normalized_state, &state);
                
                // Check if next state is terminal
                int done = is_terminal_state() || (stats.step >= MAX_STEPS_PER_EPOCH - 1);
                
                if (memory.size < MAX_STEPS_PER_EPOCH) {
                    // Store next state and done flag
                    my_memcpy(memory.next_states[memory.size], next_normalized_state, sizeof(float) * STATE_DIM);
                    memory.dones[memory.size] = done;
                    memory.rewards[memory.size] = reward;
                    memory.size++;
                }
                
                // Update step counter
                stats.step++;
            }
            
            // Update best reward
            if (stats.total_reward > stats.best_reward) {
                stats.best_reward = stats.total_reward;
                stats.best_episode = stats.epoch;
            }
            
            // Update the network
            compute_advantages(current_params.gamma, current_params.lambda);
            update_network(&current_params);
            
            // Update epoch counter
            stats.epoch++;
            
            // Indicate progress on LED
            *LEDR_ptr = stats.epoch & 0x3FF;
        }
        
        // Track best overall result
        if (stats.best_reward > best_result.best_reward) {
            best_result.params = current_params;
            best_result.best_reward = stats.best_reward;
            best_result.best_episode = stats.best_episode;
        }
        
        // Show the best result for this parameter set
        // Flash LED when a parameter set finishes
        for (int i = 0; i < 3; i++) {
            *LEDR_ptr = 0x3FF;
            for (int d = 0; d < 100000; d++) asm volatile("nop");
            *LEDR_ptr = 0;
            for (int d = 0; d < 100000; d++) asm volatile("nop");
        }
    }
    
    // Print only the final best result
    printf("Best Hyperparameters:\n");
    printf("PPO_EPOCHS: %d\n", best_result.params.ppo_epochs);
    printf("GAMMA: %.3f\n", best_result.params.gamma);
    printf("LAMBDA: %.3f\n", best_result.params.lambda);
    printf("CLIP_EPSILON: %.3f\n", best_result.params.clip_epsilon);
    printf("LEARNING_RATE: %.5f\n", best_result.params.learning_rate);
    printf("EXPLORE_RATE: %.3f\n", best_result.params.explore_rate);
    printf("Best Reward: %d at Episode %d\n", best_result.best_reward, best_result.best_episode);
    printf("Timer was used %d times for entropy\n", timer_uses);
    
    // Show the best set on the LEDs
    // Display a special pattern when benchmarking is complete
    *LEDR_ptr = 0x555;
    for (int d = 0; d < 500000; d++) asm volatile("nop");
    *LEDR_ptr = 0xAAA;
    for (int d = 0; d < 500000; d++) asm volatile("nop");
}

// Choose an action based on policy probabilities
int sample_action(float probs[ACTION_DIM]) {
    // Random value between 0-1 with improved randomness
    float r = random_float();

    // Cumulative probability check
    float cumulative = 0.0f;
    for (int i = 0; i < ACTION_DIM; i++) {
        cumulative += probs[i];
        if (r < cumulative) {
            return i;
        }
    }

    // Fallback: return highest probability action
    return (probs[1] > probs[0]) ? 1 : 0;
}

// Calculate rewards for current state
float calculate_reward() {
    // Larger penalty for terminal states
    if (is_terminal_state()) {
        return -50.0f;
    }

    // Angle component - higher reward for balancing pole upright
    float angle_reward = 1.0f - (my_abs(state.pole_angle) / MAX_ANGLE_RAD);
    angle_reward = angle_reward * angle_reward * angle_reward * angle_reward;

    // Position component - encourage staying near center
    float position_reward = 1.0f - (my_abs(state.cart_position) / 2.0f);
    position_reward = position_reward * position_reward;

    // Velocity penalties - discourage both cart velocity and angular velocity
    float vel_penalty = -0.05f * my_abs(state.cart_velocity);
    float ang_vel_penalty = -0.1f * my_abs(state.pole_angular_vel);

    // Living bonus - encourage agent to survive longer
    float living_bonus = 0.1f;

    // Combined reward
    return 30.0f * angle_reward + 5.0f * position_reward + vel_penalty + ang_vel_penalty + living_bonus;
}

// Check if state is terminal (pole fallen or cart out of bounds)
int is_terminal_state() {
    // Terminal if pole angle exceeds MAX_ANGLE_RAD radians
    if (my_abs(state.pole_angle) > MAX_ANGLE_RAD) return 1;

    // Terminal if cart position exceeds ±2.0 meters
    if (my_abs(state.cart_position) > 2.0f) return 1;

    // Additional termination for excessive angular velocity
    if (my_abs(state.pole_angular_vel) > 8.0f) return 1;

    return 0;
}

// Update physics simulation
void update_physics(int action) {
    // Apply force based on action (0 = left, 1 = right)
    float force = (action == 0) ? -FORCE_MAG : FORCE_MAG;

    // Cart-pole physics calculations based on differential equations
    float cosine = cosf(state.pole_angle);
    float sine = sinf(state.pole_angle);
    float total_mass = CART_MASS + POLE_MASS;
    
    // Calculate acceleration components
    float temp = (force + POLE_MASS * POLE_HALF_LENGTH * state.pole_angular_vel * state.pole_angular_vel * sine) / total_mass;
    float pole_accel = (GRAVITY * sine - cosine * temp) /
                     (POLE_HALF_LENGTH * (4.0f / 3.0f - POLE_MASS * cosine * cosine / total_mass));
    float cart_accel = temp - POLE_MASS * POLE_HALF_LENGTH * pole_accel * cosine / total_mass;

    // Update state using Euler integration
    state.cart_position += TIME_STEP * state.cart_velocity;
    state.cart_velocity += TIME_STEP * cart_accel;
    state.pole_angle += TIME_STEP * state.pole_angular_vel;
    state.pole_angular_vel += TIME_STEP * pole_accel;

    // Normalize pole angle to [-PI, PI]
    while (state.pole_angle > 3.14159f) state.pole_angle -= 2.0f * 3.14159f;
    while (state.pole_angle < -3.14159f) state.pole_angle += 2.0f * 3.14159f;
}

// Apply random disturbance to the pole
void apply_random_disturbance() {
    float stability = 1.0f - my_abs(state.pole_angle) / MAX_ANGLE_RAD;
    if (stability < 0.0f) stability = 0.0f;

    // Scale disturbance based on stability with improved randomness
    float max_disturbance = RANDOM_FORCE_MAX * stability;
    float random_force = random_range(-max_disturbance, max_disturbance);
    state.pole_angular_vel += random_force;
}

// Math helper functions
float my_abs(float x) {
    return x < 0 ? -x : x;
}

float my_max(float a, float b) {
    return a > b ? a : b;
}

float my_min(float a, float b) {
    return a < b ? a : b;
}

float my_sqrt(float x) {
    if (x <= 0.0f) return 0.0f;
    float result = x;
    for (int i = 0; i < 10; i++) {
        result = 0.5f * (result + x / result);
    }
    return result;
}

float my_exp(float x) {
    if (x < -5.0f) return 0.0f;
    float result = 1.0f;
    float term = 1.0f;
    for (int i = 1; i < 10; i++) {
        term *= x / i;
        result += term;
    }
    return result;
}

float my_log(float x) {
    if (x <= 0.0f) return -10.0f;
    float a = x;
    int n = 0;
    while (a >= 2.0f) {
        a *= 0.5f;
        n++;
    }
    while (a < 1.0f) {
        a *= 2.0f;
        n--;
    }
    float a_minus_1 = a - 1.0f;
    float log_a = a_minus_1 - 0.5f * a_minus_1 * a_minus_1;
    return log_a + n * 0.693147f;
}

void my_memcpy(void *dest, void *src, int n) {
    char *d = (char *)dest;
    char *s = (char *)src;
    for (int i = 0; i < n; i++) {
        d[i] = s[i];
    }
}

float leaky_relu(float x) {
    return x > 0.0f ? x : 0.01f * x;
}

float my_clamp(float value, float min, float max) {
    return value < min ? min : (value > max ? max : value);
}

// Neural Network forward pass for policy network
void policy_forward(float state[STATE_DIM], float probs[ACTION_DIM]) {
    float hidden1[HIDDEN_DIM];
    float hidden2[HIDDEN_DIM];

    // Input to first hidden layer
    for (int i = 0; i < HIDDEN_DIM; i++) {
        hidden1[i] = 0.0f;
        for (int j = 0; j < STATE_DIM; j++) {
            hidden1[i] += state[j] * nn.weights1[j][i];
        }
        hidden1[i] += nn.bias1[i];
        hidden1[i] = leaky_relu(hidden1[i]);
    }

    // First hidden to second hidden layer
    for (int i = 0; i < HIDDEN_DIM; i++) {
        hidden2[i] = 0.0f;
        for (int j = 0; j < HIDDEN_DIM; j++) {
            hidden2[i] += hidden1[j] * nn.weights_h2h[j][i];
        }
        hidden2[i] += nn.bias_h2[i];
        hidden2[i] = leaky_relu(hidden2[i]);
    }

    // Second hidden to output layer
    float output[ACTION_DIM];
    for (int i = 0; i < ACTION_DIM; i++) {
        output[i] = 0.0f;
        for (int j = 0; j < HIDDEN_DIM; j++) {
            output[i] += hidden2[j] * nn.weights2[j][i];
        }
        output[i] += nn.bias2[i];
    }

    // Softmax activation with numerical stability
    float max_val = output[0];
    for (int i = 1; i < ACTION_DIM; i++) {
        if (output[i] > max_val) max_val = output[i];
    }

    float sum_exp = 0.0f;
    for (int i = 0; i < ACTION_DIM; i++) {
        probs[i] = my_exp(output[i] - max_val);
        sum_exp += probs[i];
    }

    if (sum_exp < 1e-8f) sum_exp = 1e-8f;
    
    for (int i = 0; i < ACTION_DIM; i++) {
        probs[i] /= sum_exp;
        
        // Clamp probabilities
        if (probs[i] < 0.001f) probs[i] = 0.001f;
        if (probs[i] > 0.999f) probs[i] = 0.999f;
    }

    // Renormalize after clamping
    sum_exp = probs[0] + probs[1];
    if (sum_exp < 1e-8f) sum_exp = 1.0f;
    
    probs[0] /= sum_exp;
    probs[1] /= sum_exp;
}

// Neural Network forward pass for value function
float value_forward(float state[STATE_DIM]) {
    float hidden1[HIDDEN_DIM];
    float hidden2[HIDDEN_DIM];

    // Input to first hidden layer
    for (int i = 0; i < HIDDEN_DIM; i++) {
        hidden1[i] = 0.0f;
        for (int j = 0; j < STATE_DIM; j++) {
            hidden1[i] += state[j] * nn.value_weights1[j][i];
        }
        hidden1[i] += nn.value_bias1[i];
        hidden1[i] = leaky_relu(hidden1[i]);
    }

    // First hidden to second hidden layer
    for (int i = 0; i < HIDDEN_DIM; i++) {
        hidden2[i] = 0.0f;
        for (int j = 0; j < HIDDEN_DIM; j++) {
            hidden2[i] += hidden1[j] * nn.value_weights_h2h[j][i];
        }
        hidden2[i] += nn.value_bias_h2[i];
        hidden2[i] = leaky_relu(hidden2[i]);
    }

    // Second hidden to output layer
    float value = 0.0f;
    for (int j = 0; j < HIDDEN_DIM; j++) {
        value += hidden2[j] * nn.value_weights2[j][0];
    }
    value += nn.value_bias2[0];

    return value;
}

// Calculate log probability of action
float log_prob(float probs[ACTION_DIM], int action) {
    return my_log(probs[action]);
}

// Calculate advantages and returns with configurable gamma and lambda
void compute_advantages(float gamma, float lambda) {
    // Compute returns with N-step bootstrapping
    for (int t = memory.size - 1; t >= 0; t--) {
        float next_value;
        if (memory.dones[t]) {
            next_value = 0.0f;
        } else {
            next_value = value_forward(memory.next_states[t]);
        }
        memory.returns[t] = memory.rewards[t] + gamma * next_value;
    }

    // Compute GAE advantages
    float gae = 0.0f;
    for (int t = memory.size - 1; t >= 0; t--) {
        float next_value;
        if (memory.dones[t]) {
            next_value = 0.0f;
        } else {
            next_value = value_forward(memory.next_states[t]);
        }
        float delta = memory.rewards[t] + gamma * next_value - memory.values[t];
        gae = delta + gamma * lambda * gae;
        memory.advantages[t] = gae;
    }

    // Normalize advantages for better training stability
    if (memory.size > 1) {
        float sum = 0.0f, sum_sq = 0.0f;
        for (int t = 0; t < memory.size; t++) {
            sum += memory.advantages[t];
            sum_sq += memory.advantages[t] * memory.advantages[t];
        }

        float mean = sum / memory.size;
        float variance = (sum_sq / memory.size) - (mean * mean);
        float std_dev = my_sqrt(variance + 1e-8f);

        for (int t = 0; t < memory.size; t++) {
            memory.advantages[t] = (memory.advantages[t] - mean) / std_dev;
        }
    }
}

// Update neural network parameters using configurable hyperparameters
void update_network(Hyperparameters *params) {
    // Adaptive learning rate
    float current_lr = params->learning_rate * (1.0f - (float)stats.epoch / MAX_EPISODES);
    if (current_lr < params->learning_rate * 0.1f) {
        current_lr = params->learning_rate * 0.1f;
    }

    // Perform multiple epochs of updates
    for (int epoch = 0; epoch < params->ppo_epochs; epoch++) {
        // Initialize parameter gradients
        float dw1[STATE_DIM][HIDDEN_DIM] = {{0.0f}};
        float db1[HIDDEN_DIM] = {0.0f};
        float dw_h2h[HIDDEN_DIM][HIDDEN_DIM] = {{0.0f}};
        float db_h2[HIDDEN_DIM] = {0.0f};
        float dw2[HIDDEN_DIM][ACTION_DIM] = {{0.0f}};
        float db2[ACTION_DIM] = {0.0f};

        float vdw1[STATE_DIM][HIDDEN_DIM] = {{0.0f}};
        float vdb1[HIDDEN_DIM] = {0.0f};
        float vdw_h2h[HIDDEN_DIM][HIDDEN_DIM] = {{0.0f}};
        float vdb_h2[HIDDEN_DIM] = {0.0f};
        float vdw2[HIDDEN_DIM][1] = {{0.0f}};
        float vdb2[1] = {0.0f};

        // Create shuffle indices for randomized mini-batches
        int indices[MAX_STEPS_PER_EPOCH];
        for (int i = 0; i < memory.size; i++) {
            indices[i] = i;
        }

        // Fisher-Yates shuffle with improved randomness
        for (int i = memory.size - 1; i > 0; i--) {
            int j = random_int(0, i);
            int temp = indices[i];
            indices[i] = indices[j];
            indices[j] = temp;
        }

        // Process mini-batches
        int batch_size = memory.size > 4 ? memory.size / 4 : 1;
        for (int start = 0; start < memory.size; start += batch_size) {
            int end = start + batch_size;
            if (end > memory.size) end = memory.size;

            for (int b = start; b < end; b++) {
                int t = indices[b];
                float state[STATE_DIM];
                my_memcpy(state, memory.states[t], sizeof(float) * STATE_DIM);

                // Policy network forward pass with stored activations
                float hidden1[HIDDEN_DIM];
                float hidden2[HIDDEN_DIM];
                float output[ACTION_DIM];
                float probs[ACTION_DIM];

                // First hidden layer
                for (int i = 0; i < HIDDEN_DIM; i++) {
                    hidden1[i] = 0.0f;
                    for (int j = 0; j < STATE_DIM; j++) {
                        hidden1[i] += state[j] * nn.weights1[j][i];
                    }
                    hidden1[i] += nn.bias1[i];
                    hidden1[i] = leaky_relu(hidden1[i]);
                }

                // Second hidden layer
                for (int i = 0; i < HIDDEN_DIM; i++) {
                    hidden2[i] = 0.0f;
                    for (int j = 0; j < HIDDEN_DIM; j++) {
                        hidden2[i] += hidden1[j] * nn.weights_h2h[j][i];
                    }
                    hidden2[i] += nn.bias_h2[i];
                    hidden2[i] = leaky_relu(hidden2[i]);
                }

                // Output layer
                for (int i = 0; i < ACTION_DIM; i++) {
                    output[i] = 0.0f;
                    for (int j = 0; j < HIDDEN_DIM; j++) {
                        output[i] += hidden2[j] * nn.weights2[j][i];
                    }
                    output[i] += nn.bias2[i];
                }

                // Softmax
                float max_val = output[0];
                for (int i = 1; i < ACTION_DIM; i++) {
                    if (output[i] > max_val) max_val = output[i];
                }

                float sum_exp = 0.0f;
                for (int i = 0; i < ACTION_DIM; i++) {
                    probs[i] = my_exp(output[i] - max_val);
                    sum_exp += probs[i];
                }

                if (sum_exp < 1e-8f) sum_exp = 1e-8f;
                
                for (int i = 0; i < ACTION_DIM; i++) {
                    probs[i] /= sum_exp;
                    if (probs[i] < 0.001f) probs[i] = 0.001f;
                    if (probs[i] > 0.999f) probs[i] = 0.999f;
                }

                sum_exp = probs[0] + probs[1];
                if (sum_exp < 1e-8f) sum_exp = 1.0f;
                probs[0] /= sum_exp;
                probs[1] /= sum_exp;

                // Log probability and PPO ratio
                float current_log_prob = my_log(probs[memory.actions[t]]);
                float ratio = my_exp(current_log_prob - memory.log_probs[t]);
                float advantage = memory.advantages[t];
                float clip_ratio = my_clamp(ratio, 1.0f - params->clip_epsilon, 1.0f + params->clip_epsilon);
                float policy_loss = -my_min(ratio * advantage, clip_ratio * advantage);

                // Policy gradient for output layer
                float doutput[ACTION_DIM] = {0};
                if (ratio < 1.0f - params->clip_epsilon || ratio > 1.0f + params->clip_epsilon) {
                    // Use clipped gradient
                    float clip_factor = clip_ratio / ratio;
                    for (int i = 0; i < ACTION_DIM; i++) {
                        doutput[i] = (i == memory.actions[t])
                            ? -clip_factor * advantage * (1.0f - probs[i])
                            : clip_factor * advantage * probs[i];
                    }
                } else {
                    // Use unclipped gradient
                    for (int i = 0; i < ACTION_DIM; i++) {
                        doutput[i] = (i == memory.actions[t])
                            ? -advantage * (1.0f - probs[i])
                            : advantage * probs[i];
                    }
                }

                // Backpropagate through policy network
                // Second hidden layer gradients
                float dhidden2[HIDDEN_DIM] = {0};
                for (int i = 0; i < HIDDEN_DIM; i++) {
                    float grad = 0;
                    for (int j = 0; j < ACTION_DIM; j++) {
                        grad += doutput[j] * nn.weights2[i][j];
                    }
                    dhidden2[i] = grad * (hidden2[i] > 0 ? 1.0f : 0.01f); // LeakyReLU derivative
                }

                // First hidden layer gradients
                float dhidden1[HIDDEN_DIM] = {0};
                for (int i = 0; i < HIDDEN_DIM; i++) {
                    float grad = 0;
                    for (int j = 0; j < HIDDEN_DIM; j++) {
                        grad += dhidden2[j] * nn.weights_h2h[i][j];
                    }
                    dhidden1[i] = grad * (hidden1[i] > 0 ? 1.0f : 0.01f); // LeakyReLU derivative
                }

                // Accumulate gradients for all layers
                // Input to first hidden
                for (int i = 0; i < STATE_DIM; i++) {
                    for (int j = 0; j < HIDDEN_DIM; j++) {
                        dw1[i][j] += state[i] * dhidden1[j];
                    }
                }
                for (int i = 0; i < HIDDEN_DIM; i++) {
                    db1[i] += dhidden1[i];
                }

                // First hidden to second hidden
                for (int i = 0; i < HIDDEN_DIM; i++) {
                    for (int j = 0; j < HIDDEN_DIM; j++) {
                        dw_h2h[i][j] += hidden1[i] * dhidden2[j];
                    }
                }
                for (int i = 0; i < HIDDEN_DIM; i++) {
                    db_h2[i] += dhidden2[i];
                }

                // Second hidden to output
                for (int i = 0; i < HIDDEN_DIM; i++) {
                    for (int j = 0; j < ACTION_DIM; j++) {
                        dw2[i][j] += hidden2[i] * doutput[j];
                    }
                }
                for (int j = 0; j < ACTION_DIM; j++) {
                    db2[j] += doutput[j];
                }

                // Value network forward pass
                float v_hidden1[HIDDEN_DIM];
                float v_hidden2[HIDDEN_DIM];
                
                // First hidden layer
                for (int i = 0; i < HIDDEN_DIM; i++) {
                    v_hidden1[i] = 0.0f;
                    for (int j = 0; j < STATE_DIM; j++) {
                        v_hidden1[i] += state[j] * nn.value_weights1[j][i];
                    }
                    v_hidden1[i] += nn.value_bias1[i];
                    v_hidden1[i] = leaky_relu(v_hidden1[i]);
                }
                
                // Second hidden layer
                for (int i = 0; i < HIDDEN_DIM; i++) {
                    v_hidden2[i] = 0.0f;
                    for (int j = 0; j < HIDDEN_DIM; j++) {
                        v_hidden2[i] += v_hidden1[j] * nn.value_weights_h2h[j][i];
                    }
                    v_hidden2[i] += nn.value_bias_h2[i];
                    v_hidden2[i] = leaky_relu(v_hidden2[i]);
                }
                
                // Output value
                float value = 0.0f;
                for (int i = 0; i < HIDDEN_DIM; i++) {
                    value += v_hidden2[i] * nn.value_weights2[i][0];
                }
                value += nn.value_bias2[0];
                
                // Value loss gradient (MSE)
                float value_diff = value - memory.returns[t];
                float dvalue = 2.0f * value_diff;
                
                // Backpropagate through value network
                // Second hidden layer gradients
                float dv_hidden2[HIDDEN_DIM] = {0};
                for (int i = 0; i < HIDDEN_DIM; i++) {
                    dv_hidden2[i] = dvalue * nn.value_weights2[i][0];
                    dv_hidden2[i] *= (v_hidden2[i] > 0.0f ? 1.0f : 0.01f); // LeakyReLU derivative
                }
                
                // First hidden layer gradients
                float dv_hidden1[HIDDEN_DIM] = {0};
                for (int i = 0; i < HIDDEN_DIM; i++) {
                    float grad = 0;
                    for (int j = 0; j < HIDDEN_DIM; j++) {
                        grad += dv_hidden2[j] * nn.value_weights_h2h[i][j];
                    }
                    dv_hidden1[i] = grad * (v_hidden1[i] > 0.0f ? 1.0f : 0.01f); // LeakyReLU derivative
                }
                
                // Accumulate value gradients
                // Input to first hidden
                for (int i = 0; i < STATE_DIM; i++) {
                    for (int j = 0; j < HIDDEN_DIM; j++) {
                        vdw1[i][j] += state[i] * dv_hidden1[j];
                    }
                }
                for (int i = 0; i < HIDDEN_DIM; i++) {
                    vdb1[i] += dv_hidden1[i];
                }
                
                // First hidden to second hidden
                for (int i = 0; i < HIDDEN_DIM; i++) {
                    for (int j = 0; j < HIDDEN_DIM; j++) {
                        vdw_h2h[i][j] += v_hidden1[i] * dv_hidden2[j];
                    }
                }
                for (int i = 0; i < HIDDEN_DIM; i++) {
                    vdb_h2[i] += dv_hidden2[i];
                }
                
                // Second hidden to output
                for (int i = 0; i < HIDDEN_DIM; i++) {
                    vdw2[i][0] += v_hidden2[i] * dvalue;
                }
                vdb2[0] += dvalue;
            }

            // Apply gradients after each mini-batch with proper scaling
            float batch_scale = current_lr / (end - start);

            // Update policy network
            // First layer weights and biases
            for (int i = 0; i < STATE_DIM; i++) {
                for (int j = 0; j < HIDDEN_DIM; j++) {
                    nn.weights1[i][j] -= dw1[i][j] * batch_scale;
                    nn.value_weights1[i][j] -= vdw1[i][j] * batch_scale;
                }
            }
            for (int i = 0; i < HIDDEN_DIM; i++) {
                nn.bias1[i] -= db1[i] * batch_scale;
                nn.value_bias1[i] -= vdb1[i] * batch_scale;
            }

            // Hidden-to-hidden layer weights and biases
            for (int i = 0; i < HIDDEN_DIM; i++) {
                for (int j = 0; j < HIDDEN_DIM; j++) {
                    nn.weights_h2h[i][j] -= dw_h2h[i][j] * batch_scale;
                    nn.value_weights_h2h[i][j] -= vdw_h2h[i][j] * batch_scale;
                }
            }
            for (int i = 0; i < HIDDEN_DIM; i++) {
                nn.bias_h2[i] -= db_h2[i] * batch_scale;
                nn.value_bias_h2[i] -= vdb_h2[i] * batch_scale;
            }

            // Output layer weights and biases
            for (int i = 0; i < HIDDEN_DIM; i++) {
                for (int j = 0; j < ACTION_DIM; j++) {
                    nn.weights2[i][j] -= dw2[i][j] * batch_scale;
                }
                nn.value_weights2[i][0] -= vdw2[i][0] * batch_scale;
            }
            for (int i = 0; i < ACTION_DIM; i++) {
                nn.bias2[i] -= db2[i] * batch_scale;
            }
            nn.value_bias2[0] -= vdb2[0] * batch_scale;
        }
    }
}

// Initialize neural network with optimized weight initialization
void init_network() {
    // Mix in timer entropy for better initialization
    random_seed ^= timer_value();

    // Ensure random seed has enough entropy
    for (int i = 0; i < 10; i++) {
        random_seed = xorshift32();
    }

    // Kaiming initialization scale factors
    float init_range1 = sqrtf(2.0f / STATE_DIM);
    float init_range2 = sqrtf(2.0f / HIDDEN_DIM);
    float init_range3 = sqrtf(2.0f / HIDDEN_DIM);

    // Initialize policy network weights
    // First layer - Input to first hidden layer
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < HIDDEN_DIM; j++) {
            nn.weights1[i][j] = (2.0f * random_float() - 1.0f) * init_range1;
            nn.value_weights1[i][j] = (2.0f * random_float() - 1.0f) * init_range1;
        }
    }
    
    // Hidden-to-hidden layer
    for (int i = 0; i < HIDDEN_DIM; i++) {
        for (int j = 0; j < HIDDEN_DIM; j++) {
            nn.weights_h2h[i][j] = (2.0f * random_float() - 1.0f) * init_range2;
            nn.value_weights_h2h[i][j] = (2.0f * random_float() - 1.0f) * init_range2;
        }
    }
    
    // Output layer
    for (int i = 0; i < HIDDEN_DIM; i++) {
        for (int j = 0; j < ACTION_DIM; j++) {
            nn.weights2[i][j] = (2.0f * random_float() - 1.0f) * init_range3;
        }
        nn.value_weights2[i][0] = (2.0f * random_float() - 1.0f) * init_range3;
    }

    // Initialize biases
    for (int i = 0; i < HIDDEN_DIM; i++) {
        nn.bias1[i] = 0.01f * (random_float() - 0.5f);
        nn.value_bias1[i] = 0.01f * (random_float() - 0.5f);
        
        nn.bias_h2[i] = 0.01f * (random_float() - 0.5f);
        nn.value_bias_h2[i] = 0.01f * (random_float() - 0.5f);
    }

    // Initialize output biases with slight bias for better initial performance
    nn.bias2[0] = -0.1f;  // Slight bias against going left
    nn.bias2[1] = 0.1f;   // Slight bias toward going right

    // Initialize value function bias
    nn.value_bias2[0] = 0.0f;

    // Verify network produces reasonable initial outputs
    float test_state[STATE_DIM] = {0, 0, 0.1f, 0}; // Slightly tilted right
    float probs[ACTION_DIM];
    policy_forward(test_state, probs);

    // Ensure the initial policy responds correctly to a tilted pole
    if (probs[1] < 0.55f) {  // Should favor moving right for right-tilted pole
        nn.bias2[0] = -0.2f;
        nn.bias2[1] = 0.2f;
    }

    // Test with pole tilted left
    test_state[2] = -0.1f;  // Slightly tilted left
    policy_forward(test_state, probs);

    // Ensure the initial policy responds correctly to a left-tilted pole
    if (probs[0] < 0.55f) {  // Should favor moving left for left-tilted pole
        nn.bias2[0] = 0.2f;
        nn.bias2[1] = -0.2f;
    }
}

// Normalize state variables for neural network input
void normalize_state(float normalized_state[STATE_DIM], CartPoleState *cart_state) {
    // Position: [-2.4, 2.4] -> [-1, 1]
    normalized_state[0] = cart_state->cart_position / 2.4f;
    
    // Velocity: clip and normalize
    normalized_state[1] = cart_state->cart_velocity;
    if (normalized_state[1] > 10.0f) normalized_state[1] = 10.0f;
    if (normalized_state[1] < -10.0f) normalized_state[1] = -10.0f;
    normalized_state[1] /= 10.0f;
    
    // Angle: normalize to [-1, 1]
    normalized_state[2] = cart_state->pole_angle / MAX_ANGLE_RAD;
    
    // Angular velocity: clip and normalize
    normalized_state[3] = cart_state->pole_angular_vel;
    if (normalized_state[3] > 10.0f) normalized_state[3] = 10.0f;
    if (normalized_state[3] < -10.0f) normalized_state[3] = -10.0f;
    normalized_state[3] /= 10.0f;
    
    // Emphasis on angle for dangerous situations
    if (cart_state->pole_angle * cart_state->pole_angular_vel > 0) {
        // If pole is moving away from center, emphasize the angle
        normalized_state[2] *= 1.05f;
    }
}