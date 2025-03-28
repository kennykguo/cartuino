// This inference file is much more stable now
#define PIXEL_BUF_CTRL_BASE 0xFF203020
#define KEY_BASE            0xFF200050
#define LEDR_BASE           0xFF200000
#define HEX3_HEX0_BASE      0xFF200020
#define TIMER_BASE          0xFF202000
#define MTIME_BASE          0xFF202100

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <stddef.h>  // for null def

#define VGA_WIDTH 320
#define VGA_HEIGHT 240
#define LINE_COLOUR 0x1E3D

// SIMULATION PARAMETERS
#define GRAVITY 9.8f
#define CART_MASS 1.0f
#define POLE_MASS 0.2f
#define POLE_HALF_LENGTH 0.75f
#define FORCE_MAG 20.0f
#define TIME_STEP 0.02f
#define RANDOM_FORCE_MAX 0.5f
#define PIXEL_SCALE 50.0f
#define MAX_ANGLE_RAD 0.45
#define MAX_ANGULAR_VELOCITY 4.0f

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

// PPO ALGO PARAMETERS - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
#define STATE_DIM 4 // NEVER CHANGE
#define ACTION_DIM 2 // NEVER CHANGE
#define HIDDEN_DIM 64
#define MAX_EPISODES 1000
#define MAX_STEPS_PER_EPOCH 300  // longer episodes -> allow more learning


#define PPO_EPOCHS 12
#define GAMMA 0.98f
#define LAMBDA 0.95f
#define CLIP_EPSILON 0.1f
#define LEARNING_RATE 0.007f   
#define EXPLORE_RATE 0.25f  // Updated from 0.25f based on tuning results
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// TRAINING/INFERENCES MODES
#define MODE_TRAIN 0
#define MODE_INFERENCE 1

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
} PerformanceStats;

// NN-struct using arrays only
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
    float reward;
} NeuralNetwork;

// mem. buffer to store experiences
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

// GLOBAL VAR
volatile int pixel_buffer_start;
short int Buffer1[240][512];
short int Buffer2[240][512]; // Back buffer
volatile int *KEY_ptr;
volatile int *LEDR_ptr;
volatile unsigned int *HEX3_HEX0_ptr;
volatile int timer_uses = 0;  // Count times timer is used for randomness

// GLOBAL SIM. STATES
CartPoleState state;
CartPoleState prev_state;
PerformanceStats stats;
NeuralNetwork nn;
Memory memory;
int simulation_running = 0;
int current_mode = MODE_TRAIN;

// Need to have greater range of randomness
unsigned int random_seed;
int best_steps = 0;


// Forward declarations
void plot_pixel(int x, int y, short int line_color);
void wait_for_vsync();
void draw_line(int startx, int starty, int endx, int endy, short int colour);
void swap(int* a1, int* a2);
void clear_screen();
void draw_cart_pole();
void draw_start_screen();
void draw_stats();
void update_physics(int action);
int choose_action();
float calculate_reward();
int is_terminal_state();
void apply_random_disturbance();
void init_interval_timer(void);
unsigned int timer_value(void);
unsigned int xorshift32();
float random_float(void);
int random_int(int min, int max);
float random_range(float min, float max);
void draw_char(int x, int y, char c, short int color);
void draw_text(int x, int y, char* str, short int color);
void int_to_str(int num, char* str);
void float_to_str(float num, char* str, int precision);
float my_abs(float x);
float my_max(float a, float b);
float my_min(float a, float b);
void my_memcpy(void *dest, void *src, int n);
void policy_forward(float state[STATE_DIM], float probs[ACTION_DIM]);
float value_forward(float state[STATE_DIM]);
float log_prob(float probs[ACTION_DIM], int action);
void compute_advantages();
void update_network();
void init_network();
void normalize_state(float normalized_state[STATE_DIM], CartPoleState *cart_state);
float my_clamp(float value, float min, float max);
int sample_action(float probs[ACTION_DIM]);
void print_training_stats();
float my_sqrt(float x);
float my_exp(float x);
float my_log(float x);
float leaky_relu(float x);
void calculate_weight_stats(float weights[][HIDDEN_DIM], int rows, int cols, char* name);
int min(x,y);


void save_best_weights();
void load_best_weights();
NeuralNetwork best_nn;  // To store the best network weights
int best_epoch = 0;     // To track which epoch had the best performance
NeuralNetwork training_backup;  // To preserve current training progress
int backup_valid = 0;           // Flag indicating backup exists


void save_best_weights() {
    my_memcpy(&best_nn, &nn, sizeof(NeuralNetwork));
    best_epoch = stats.epoch;
    best_steps = stats.step;
    best_nn.reward = stats.total_reward;  // Store the reward
    
    // Visual feedback
    *LEDR_ptr = 0x3FF;  // Turn on all LEDs briefly
    printf("Saved best weights from epoch %d (reward: %d, steps: %d)\n", 
           best_epoch, stats.total_reward, best_steps);
}

// New function to restore training progress
void restore_training_weights() {
    if (backup_valid) {
        my_memcpy(&nn, &training_backup, sizeof(NeuralNetwork));
        printf("Restored training progress\n");
    }
}


// Modified load function
void load_best_weights() {
    if (best_epoch > 0) {
        my_memcpy(&training_backup, &nn, sizeof(NeuralNetwork));
        backup_valid = 1;
        my_memcpy(&nn, &best_nn, sizeof(NeuralNetwork));
        
        printf("Loaded best weights (Epoch %d, Reward %d)\n", 
              best_epoch, (int)best_nn.reward);
    }
}



int main(void) {
    volatile int * pixel_ctrl_ptr = (int *)PIXEL_BUF_CTRL_BASE;
    KEY_ptr = (int *)KEY_BASE;
    LEDR_ptr = (int *)LEDR_BASE;
    HEX3_HEX0_ptr = (unsigned int *)HEX3_HEX0_BASE;

    // Initialize the interval timer for better randomness
    init_interval_timer();
    
    // Use timer value for initial seeding, properly mixed
    random_seed = timer_value();
    
    // Additional entropy mixing
    for (int i = 0; i < 10; i++) {
        random_seed = xorshift32();
    }

    // Initialize cart-pole state
    state.cart_position = 0.0f;
    state.cart_velocity = 0.0f;
    state.pole_angle = 0.01f;  // Start with a small angle to make it unstable
    state.pole_angular_vel = 0.0f;

    prev_state = state; // Save initial state

    // Initialize performance stats
    stats.epoch = 0;
    stats.step = 0;
    stats.total_reward = 0;
    stats.best_reward = 0;

    // Initialize the neural network
    init_network();

    // Initialize memory buffer
    memory.size = 0;

    // Initialize the pixel buffer
    /* set front pixel buffer to Buffer 1 */
    *(pixel_ctrl_ptr + 1) = (int) &Buffer1; // first store the address in the back buffer

    /* now, swap the front/back buffers, to set the front buffer location */
    wait_for_vsync();

    /* initialize a pointer to the pixel buffer, used by drawing functions */
    pixel_buffer_start = *pixel_ctrl_ptr;
    clear_screen(); // pixel_buffer_start points to the pixel buffer

    /* set back pixel buffer to Buffer 2 */
    *(pixel_ctrl_ptr + 1) = (int) &Buffer2;
    pixel_buffer_start = *(pixel_ctrl_ptr + 1); // we draw on the back buffer
    clear_screen(); // Clear the back buffer

    // Show start screen
    draw_start_screen();

    // Main loop - wait for KEY0 to start
    while (1) {
        // Check if KEY0 is pressed to start simulation
        if (*KEY_ptr & 0x1) {
            // Wait for key release (simple debouncing)
            while (*KEY_ptr & 0x1);

            simulation_running = 1;
            stats.epoch++;
            memory.size = 0; // Reset memory buffer for new episode
            break;
        }

        // Check if KEY1 is pressed to toggle between train/inference modes
        if (*KEY_ptr & 0x2) {
            // Wait for key release (simple debouncing)
            while (*KEY_ptr & 0x2);

            // Toggle mode
            current_mode = (current_mode == MODE_TRAIN) ? MODE_INFERENCE : MODE_TRAIN;
        }

        wait_for_vsync(); // Swap buffers
        pixel_buffer_start = *(pixel_ctrl_ptr + 1); // Update to draw on the back buffer

        // Keep drawing the start screen while waiting
        clear_screen();
        draw_start_screen();
    }

    // Main simulation loop
    while (1) {
        // Swap buffers - we now draw to the back buffer
        wait_for_vsync();
        pixel_buffer_start = *(pixel_ctrl_ptr + 1);

        // Clear the screen (back buffer)
        clear_screen();

        // Save current state for erasing
        prev_state = state;

        if (simulation_running) {
            // Choose an action using policy network
            float normalized_state[STATE_DIM];
            normalize_state(normalized_state, &state);

            float action_probs[ACTION_DIM];
            policy_forward(normalized_state, action_probs);

            int action;
            if (current_mode == MODE_TRAIN) {
                // Sample action for training with more exploration
                float explore_rate = my_max(EXPLORE_RATE, 0.6f * (1.0f - (float)stats.epoch / 200.0f));
                if (random_float() < explore_rate) {
                    // Take random action with probability explore_rate
                    action = random_int(0, ACTION_DIM - 1);

                    // DEBUG: Show exploration with LED
                    *LEDR_ptr = 0x300 | (stats.step & 0xFF);  // Pattern for exploration
                } else {
                    // Sample from policy distribution
                    action = sample_action(action_probs);

                    // DEBUG: Show exploitation with LED
                    *LEDR_ptr = 0x500 | (stats.step & 0xFF);  // Pattern for exploitation
                }

                // Store experience in memory buffer
                if (memory.size < MAX_STEPS_PER_EPOCH) {
                    // Store current state
                    my_memcpy(memory.states[memory.size], normalized_state, sizeof(float) * STATE_DIM);

                    // Store log probability of the chosen action
                    memory.log_probs[memory.size] = log_prob(action_probs, action);

                    // Store action
                    memory.actions[memory.size] = action;

                    // Store value estimate
                    memory.values[memory.size] = value_forward(normalized_state);
                }
            } else {
                // For inference, we can either select the most probable action or sample from the distribution
                // Using the most probable action is more stable for demonstration
                action = (action_probs[1] > action_probs[0]) ? 1 : 0;
            }

            // Apply random disturbance only in training mode
            // and less often as training progresses
            if (current_mode == MODE_TRAIN && stats.step % 20 == 0 &&
                (random_float() < 0.2f * (1.0f - (float)stats.epoch / 30.0f))) {
                apply_random_disturbance();
            }

            // Update physics based on action
            update_physics(action);

            // Calculate reward
            float reward = calculate_reward();

            // Normalize the next state
            float next_normalized_state[STATE_DIM];
            normalize_state(next_normalized_state, &state);

            // Check if next state is terminal
            int done = is_terminal_state() || (stats.step >= MAX_STEPS_PER_EPOCH - 1);

            if (current_mode == MODE_TRAIN && memory.size < MAX_STEPS_PER_EPOCH) {
                // Store next state and done flag
                my_memcpy(memory.next_states[memory.size], next_normalized_state, sizeof(float) * STATE_DIM);
                memory.dones[memory.size] = done;
                memory.rewards[memory.size] = reward;
                memory.size++;
            }

            stats.total_reward += (int)reward;

            // Update step counter
            stats.step++;

            // Check if state is terminal or max steps reached
            if (is_terminal_state() || stats.step >= MAX_STEPS_PER_EPOCH) {
                // Update best reward
                if (stats.total_reward > stats.best_reward) {
                    stats.best_reward = stats.total_reward;
                    print_training_stats();  // Print stats when new best reward is achieved
                    printf("NEW BEST REWARD ACHIEVED!\n");
                    save_best_weights();
                }

                if (current_mode == MODE_TRAIN) {
                    // Ensure the network gets updated
                    compute_advantages();
                    update_network();
                    print_training_stats();

                    // Debug visualization - show training progress on LEDs
                    *LEDR_ptr = 0x155;  // Pattern to indicate network update

                    // Small delay to see the LED pattern
                    for (int d = 0; d < 100000; d++) {  // Reduced delay to avoid long pauses
                        asm volatile("nop");
                    }
                }

                // Reset state for new epoch with better randomization
                // Regenerate random seed for more variation
                random_seed = timer_value() ^ (stats.epoch * 12345) ^ (xorshift32() * 54321);

                // Initialize with small random values to increase variance between episodes
                state.cart_position = random_range(-0.05f, 0.05f);
                state.cart_velocity = random_range(-0.05f, 0.05f);
                state.pole_angle = random_range(-0.05f, 0.05f);
                state.pole_angular_vel = random_range(-0.025f, 0.025f);

                // Update epoch counter
                stats.epoch++;
                stats.step = 0;
                stats.total_reward = 0;
                memory.size = 0; // Reset memory for new episode

                // Turn on LED0 to indicate new epoch
                *LEDR_ptr = 0x1;

            } else {
                // Update LEDs to show step count (binary)
                *LEDR_ptr = (stats.step & 0x3FF);  // Lower 10 bits
            }
        }

        // Check if KEY0 is pressed to restart
        if (*KEY_ptr & 0x1) {
            // Wait for key release (simple debouncing)
            while (*KEY_ptr & 0x1);

            // Reset state and start new epoch with better randomization
            random_seed = timer_value() ^ (stats.epoch * 12345) ^ (xorshift32() * 54321);

            // Initialize with small random values to increase variance between episodes
            state.cart_position = random_range(-0.05f, 0.05f);
            state.cart_velocity = random_range(-0.05f, 0.05f);
            state.pole_angle = random_range(-0.05f, 0.05f);
            state.pole_angular_vel = random_range(-0.025f, 0.025f);

            stats.epoch++;
            stats.step = 0;
            stats.total_reward = 0;
            memory.size = 0; // Reset memory for new episode

            simulation_running = 1;
        }

        // Check if KEY1 is pressed to toggle between train/inference modes
        if (*KEY_ptr & 0x2) {
            // Wait for key release (simple debouncing)
            while (*KEY_ptr & 0x2);

            if (current_mode == MODE_TRAIN) {
                // Switching from TRAIN to INFERENCE mode
                my_memcpy(&training_backup, &nn, sizeof(NeuralNetwork));
                backup_valid = 1;
                
                // 2. Load best weights for inference if available
                if (best_epoch > 0) {
                    my_memcpy(&nn, &best_nn, sizeof(NeuralNetwork));
                    printf("Switched to INFERENCE mode using best weights (Epoch %d, Reward %d)\n", 
                        best_epoch, (int)best_nn.reward);
                } else {
                    printf("Switched to INFERENCE mode (no best weights yet)\n");
                }
                
                // 3. Set mode to inference
                current_mode = MODE_INFERENCE;
                
            } else {
                // Switching from INFERENCE back to TRAIN mode
                
                // 1. Restore training weights if backup exists
                if (backup_valid) {
                    my_memcpy(&nn, &training_backup, sizeof(NeuralNetwork));
                    printf("Switched to TRAINING mode (restored training weights)\n");
                } else {
                    printf("Switched to TRAINING mode\n");
                }
                
                // 2. Set mode to training
                current_mode = MODE_TRAIN;
            }
            
            // Visual feedback
            *LEDR_ptr = 0x2AA;  // Pattern to indicate mode change
            for (int i = 0; i < 100000; i++) asm volatile("nop");
            *LEDR_ptr = 0;
        }
        // Draw the cart-pole and stats
        draw_cart_pole();
        draw_stats();
    }

    return 0;
}

/**
 * Initialize the interval timer for random number generation
 */
void init_interval_timer(void) {
    volatile unsigned short *TIMER_CONTROL_ptr = (volatile unsigned short *)(TIMER_BASE + 4);
    volatile unsigned short *TIMER_START_LOW_ptr = (volatile unsigned short *)(TIMER_BASE + 8);
    volatile unsigned short *TIMER_START_HIGH_ptr = (volatile unsigned short *)(TIMER_BASE + 12);
    
    // Stop timer if it's running
    *TIMER_CONTROL_ptr = 0x8;  // TIMER_STOP
    
    // Set timer period to a prime number for better randomness
    // Instead of default 12.5M value
    unsigned int count_value = 9973651;  // Prime number < 10M
    
    // Set the start value (low and high parts)
    *TIMER_START_LOW_ptr = count_value & 0xFFFF;
    *TIMER_START_HIGH_ptr = (count_value >> 16) & 0xFFFF;
    
    // Start the timer in continuous mode
    *TIMER_CONTROL_ptr = 0x6;  // TIMER_START | TIMER_CONT
}

/**
 * Read the timer value using the interval timer's snapshot capability
 */
unsigned int timer_value(void) {
    volatile unsigned short *TIMER_SNAP_LOW_ptr = (volatile unsigned short *)(TIMER_BASE + 16);
    volatile unsigned short *TIMER_SNAP_HIGH_ptr = (volatile unsigned short *)(TIMER_BASE + 20);
    
    // Increment usage counter (for debugging)
    timer_uses++;
    
    // Write to TIMER_SNAP_LOW to capture the current count
    *TIMER_SNAP_LOW_ptr = 0;  // Value doesn't matter, just the write operation
    
    // Now read the snapshot values
    unsigned int low = *TIMER_SNAP_LOW_ptr;
    unsigned int high = *TIMER_SNAP_HIGH_ptr;
    
    // Combine to form a 32-bit value
    unsigned int timer_value = (high << 16) | low;
    
    // For added randomness, we can also mix in the machine timer
    volatile unsigned int *mtime_ptr = (unsigned int *)MTIME_BASE;
    unsigned int mtime_low = *(mtime_ptr + 1);  // 0xFF202104 (mtime low)
    
    // Use XOR to mix values without losing entropy
    return timer_value ^ (mtime_low & 0xFFFF);
}

// choose action based on PPO
int choose_action() {
    float normalized_state[STATE_DIM];
    normalize_state(normalized_state, &state);

    float action_probs[ACTION_DIM];
    policy_forward(normalized_state, action_probs);

    if (current_mode == MODE_TRAIN) {
        return sample_action(action_probs);
    } else {
        // explotation -> choose most probable action
        return (action_probs[1] > action_probs[0]) ? 1 : 0;
    }
    // return (action_probs[1] > action_probs[0]) ? 1 : 0;
}

// sample action from policy probabilities
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
// Why are we sampling like this and not just argmax?
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
int sample_action(float probs[ACTION_DIM]) {
    // Random value between 0-1 with improved randomness
    float r = random_float();

    // Cumulative probability check
    float cumulative = 0.0f;
    for (int i = 0; i < ACTION_DIM; i++) {
        cumulative += probs[i];
        if (r < cumulative) {
            return i;  // return first action where cumulative probability exceeds r
        }
    }

    // Fallback: return highest probability action
    return (probs[1] > probs[0]) ? 1 : 0;
}
// calculate the reward for the current state
// float calculate_reward() {
//     if (is_terminal_state()) {
//         return -10.0f;
//     }
    
//     // Quadratic angle reward - heavily rewards staying upright
//     float angle_reward = 1.0f - (my_abs(state.pole_angle) / MAX_ANGLE_RAD);
//     angle_reward = angle_reward * angle_reward;
    
//     // Position reward - less important than angle
//     float position_reward = 1.0f - (my_abs(state.cart_position) / 2.0f);
//     position_reward = position_reward * position_reward;
    
//     // Velocity penalties to encourage smoother control
//     float vel_penalty = 0.1f * my_abs(state.cart_velocity) / 10.0f;
//     float ang_vel_penalty = 0.1f * my_abs(state.pole_angular_vel) / MAX_ANGULAR_VELOCITY;
    
//     return 1.0f + (5.0f * angle_reward + 2.0f * position_reward - vel_penalty - ang_vel_penalty);
// }
// calculate the reward for the current state

// Revised calculate_reward function with more stable reward scaling
float calculate_reward() {
    // Larger penalty for terminal states to discourage failures
    if (is_terminal_state()) {
        return -20.0f;  // Reduced from -50.0f to avoid extreme values
    }

    // Angle component - higher reward for balancing pole upright
    // Cubic reward function instead of quartic for better numerical stability
    float angle_reward = 1.0f - (my_abs(state.pole_angle) / MAX_ANGLE_RAD);
    angle_reward = angle_reward * angle_reward * angle_reward;  // Cubic reward curve

    // Position component - encourage staying near center
    float position_reward = 1.0f - (my_abs(state.cart_position) / 2.0f);
    position_reward = position_reward * position_reward;  // Squared reward curve

    // Velocity penalties - discourage both cart velocity and angular velocity
    // Added clipping to prevent extreme values
    float cart_vel_clipped = my_clamp(state.cart_velocity, -10.0f, 10.0f);
    float ang_vel_clipped = my_clamp(state.pole_angular_vel, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
    
    float vel_penalty = -0.05f * my_abs(cart_vel_clipped);
    float ang_vel_penalty = -0.1f * my_abs(ang_vel_clipped);

    // Living bonus - encourage agent to survive longer
    float living_bonus = 0.1f;

    // Combined reward - angle is most important, but scaled down for stability
    return 15.0f * angle_reward + 5.0f * position_reward + vel_penalty + ang_vel_penalty + living_bonus;
}


// check if the state is terminal (pole fallen or cart out of bounds)
int is_terminal_state() {
    // Terminal if pole angle exceeds ±12 degrees (MAX_ANGLE_RAD radians)
    if (my_abs(state.pole_angle) > MAX_ANGLE_RAD) return 1;

    // Terminal if cart position exceeds ±2.0 meters
    if (my_abs(state.cart_position) > 2.0f) return 1;

    // Additional termination condition - if angular velocity is too high
    // This helps prevent wild oscillations
    if (my_abs(state.pole_angular_vel) > MAX_ANGULAR_VELOCITY) return 1;

    return 0;
}

// physics simulation
void update_physics(int action) {
    // Apply force based on action (0 = left, 1 = right)
    float force = (action == 0) ? -FORCE_MAG : FORCE_MAG;

    // Cart-pole physics calculations based on differential equations
    // Using Euler integration method for simplicity

    // Calculate acceleration components
    float cosine = cosf(state.pole_angle);
    float sine = sinf(state.pole_angle);

    // Calculate total mass
    float total_mass = CART_MASS + POLE_MASS;

    // Calculate acceleration of the cart
    float temp = (force + POLE_MASS * POLE_HALF_LENGTH * state.pole_angular_vel * state.pole_angular_vel * sine) / total_mass;

    // Calculate angular acceleration of the pole
    float pole_accel = (GRAVITY * sine - cosine * temp) /
                       (POLE_HALF_LENGTH * (4.0f / 3.0f - POLE_MASS * cosine * cosine / total_mass));

    // Calculate cart acceleration
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

// performance stats
void draw_stats() {
    char buffer[20];

    // Display epoch counter in top right
    draw_text(VGA_WIDTH - 60, 10, "EPOCH", GREEN);
    int_to_str(stats.epoch, buffer);
    draw_text(VGA_WIDTH - 20, 10, buffer, GREEN);

    // Display step counter
    draw_text(VGA_WIDTH - 50, 20, "STEP", YELLOW);
    int_to_str(stats.step, buffer);
    draw_text(VGA_WIDTH - 20, 20, buffer, YELLOW);

    // Display reward
    draw_text(10, 10, "REWARD", CYAN);
    int_to_str(stats.total_reward, buffer);
    draw_text(50, 10, buffer, CYAN);

    // Display best reward
    draw_text(10, 20, "BEST", MAGENTA);
    int_to_str(stats.best_reward, buffer);
    draw_text(40, 20, buffer, MAGENTA);

    // Display memory size for debugging
    draw_text(10, 30, "MEM", GREEN);
    int_to_str(memory.size, buffer);
    draw_text(40, 30, buffer, GREEN);

    // Display current mode
    if (current_mode == MODE_TRAIN) {
        draw_text(VGA_WIDTH - 60, 30, "TRAINING", MAGENTA);

        // Draw learning progress bar in training mode
        int progress_width = 100;
        int progress_height = 5;
        int progress_x = VGA_WIDTH / 2 - progress_width / 2;
        int progress_y = 40;

        // Draw border
        for (int x = -1; x <= progress_width; x++) {
            plot_pixel(progress_x + x, progress_y - 1, WHITE);
            plot_pixel(progress_x + x, progress_y + progress_height, WHITE);
        }
        for (int y = -1; y <= progress_height; y++) {
            plot_pixel(progress_x - 1, progress_y + y, WHITE);
            plot_pixel(progress_x + progress_width, progress_y + y, WHITE);
        }

        // draw progress - base it on ratio of current reward to best reward
        float progress = 0.0f;
        if (stats.best_reward > 0) {
            progress = (float)stats.total_reward / stats.best_reward;
            if (progress > 1.0f) progress = 1.0f;
        }

        int fill_width = (int)(progress * progress_width);
        for (int y = 0; y < progress_height; y++) {
            for (int x = 0; x < fill_width; x++) {
                plot_pixel(progress_x + x, progress_y + y, CYAN);
            }
        }
    } else {
        draw_text(VGA_WIDTH - 70, 30, "INFERENCE", CYAN);
    }

    // Display state information
    draw_text(10, VGA_HEIGHT - 10, "POS", WHITE);
    int pos = (int)(state.cart_position * 100);
    int_to_str(pos, buffer);
    draw_text(30, VGA_HEIGHT - 10, buffer, WHITE);

    // Display RNG info
    int_to_str(timer_uses, buffer);
    draw_text(140, VGA_HEIGHT - 10, buffer, YELLOW);
    draw_text(110, VGA_HEIGHT - 10, "TMR:", YELLOW);

    draw_text(60, VGA_HEIGHT - 10, "ANG", WHITE);
    int ang = (int)(state.pole_angle * 57.3); // Convert to degrees
    int_to_str(ang, buffer);
    draw_text(85, VGA_HEIGHT - 10, buffer, WHITE);
}

// plot a pixel at (x, y) with the specified color
void plot_pixel(int x, int y, short int line_color) {
    volatile short int *one_pixel_address;

    one_pixel_address = (volatile short int*)(pixel_buffer_start + (y << 10) + (x << 1));
    *one_pixel_address = line_color;
}

// buffer swap
void wait_for_vsync() {
    volatile int * pixel_ctrl_ptr = (int *)PIXEL_BUF_CTRL_BASE;

    // sync buffer
    *pixel_ctrl_ptr = 1;

    // poll the status bit until it becomes 0
    volatile int *status_ptr = (int *)(PIXEL_BUF_CTRL_BASE + 0xC);
    while ((*status_ptr & 0x01) != 0);
}

// clear screen to black
void clear_screen() {
    for (int x = 0; x < VGA_WIDTH; x++) {
        for (int y = 0; y < VGA_HEIGHT; y++) {
            plot_pixel(x, y, BLACK);
        }
    }
}

// swap two integers
void swap(int* a1, int* a2) {
    *a1 = *a1 ^ *a2;
    *a2 = *a1 ^ *a2;
    *a1 = *a1 ^ *a2;
}

// draw a line from (startx, starty) to (endx, endy) with the specified color
void draw_line(int startx, int starty, int endx, int endy, short int colour) {
    int is_steep = 0;
    int dy = endy - starty;
    int dx = endx - startx;

    if (abs(dy) > abs(dx))
        is_steep = 1;

    if (is_steep) {
        swap(&startx, &starty);
        swap(&endx, &endy);
    }

    if (startx > endx) {
        swap(&startx, &endx);
        swap(&starty, &endy);
    }

    dx = endx - startx;
    dy = abs(endy - starty);

    int error = -(dx/2);
    int y = starty;
    int y_step;

    if (starty < endy)
        y_step = 1;
    else
        y_step = -1;

    for (int x = startx; x <= endx; x++) {
        if (is_steep)
            plot_pixel(y, x, colour);
        else
            plot_pixel(x, y, colour);

        error += dy;
        if (error > 0) {
            y += y_step;
            error -= dx;
        }
    }
}

// draw the cart-pole system
void draw_cart_pole() {
    // Calculate screen coordinates for cart
    int cart_screen_x = VGA_WIDTH / 2 + (int)(state.cart_position * PIXEL_SCALE);
    int cart_screen_y = VGA_HEIGHT - CART_HEIGHT - 30;  // 10 pixels from bottom

    // Draw the cart (as a simple rectangle)
    for (int y = 0; y < CART_HEIGHT; y++) {
        for (int x = 0; x < CART_WIDTH; x++) {
            plot_pixel(cart_screen_x - CART_WIDTH/2 + x, cart_screen_y + y, BLUE);
        }
    }

    // Calculate pole endpoint coordinates
    float pole_endpoint_x = state.cart_position + sinf(state.pole_angle) * POLE_HALF_LENGTH * 2;
    float pole_endpoint_y = -cosf(state.pole_angle) * POLE_HALF_LENGTH * 2;

    int pole_screen_endpoint_x = VGA_WIDTH / 2 + (int)(pole_endpoint_x * PIXEL_SCALE);
    int pole_screen_endpoint_y = cart_screen_y + (int)(pole_endpoint_y * PIXEL_SCALE);

    // Draw the pole as a thick line
    for (int i = -POLE_WIDTH/2; i <= POLE_WIDTH/2; i++) {
        draw_line(cart_screen_x + i, cart_screen_y,
                  pole_screen_endpoint_x + i, pole_screen_endpoint_y, RED);
    }

    // Draw a circle at the pivot point (as a small filled circle)
    int radius = 4;
    for (int y = -radius; y <= radius; y++) {
        for (int x = -radius; x <= radius; x++) {
            if (x*x + y*y <= radius*radius) {
                plot_pixel(cart_screen_x + x, cart_screen_y + y, YELLOW);
            }
        }
    }
}

// draw the start screen with more detailed instructions
void draw_start_screen() {
    // "CARTRL" large in the center
    int title_x = VGA_WIDTH / 2 - 60;
    int title_y = VGA_HEIGHT / 3 - 30;

    // C
    for (int i = 0; i < 20; i++) {
        for (int j = 0; j < 5; j++) {
            plot_pixel(title_x + j, title_y + i, CYAN);
        }
    }
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 15; j++) {
            plot_pixel(title_x + 5 + j, title_y + i, CYAN);
            plot_pixel(title_x + 5 + j, title_y + 15 + i, CYAN);
        }
    }

    // A
    title_x += 25;
    for (int i = 0; i < 20; i++) {
        for (int j = 0; j < 5; j++) {
            plot_pixel(title_x + j, title_y + i, CYAN);
            plot_pixel(title_x + 15 + j, title_y + i, CYAN);
        }
    }
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 15; j++) {
            plot_pixel(title_x + 5 + j, title_y + i, CYAN);
            plot_pixel(title_x + 5 + j, title_y + 8 + i, CYAN);
        }
    }

    // R
    title_x += 25;
    for (int i = 0; i < 20; i++) {
        for (int j = 0; j < 5; j++) {
            plot_pixel(title_x + j, title_y + i, CYAN);
        }
    }
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 15; j++) {
            plot_pixel(title_x + 5 + j, title_y + i, CYAN);
            plot_pixel(title_x + 5 + j, title_y + 8 + i, CYAN);
        }
    }
    for (int i = 0; i < 10; i++) {
        for (int j = 0; j < 5; j++) {
            plot_pixel(title_x + 15 + j, title_y + 10 + i, CYAN);
        }
    }

    // T
    title_x += 25;
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 20; j++) {
            plot_pixel(title_x + j, title_y + i, CYAN);
        }
    }
    for (int i = 0; i < 20; i++) {
        for (int j = 0; j < 5; j++) {
            plot_pixel(title_x + 8 + j, title_y + i, CYAN);
        }
    }

    // R
    title_x += 25;
    for (int i = 0; i < 20; i++) {
        for (int j = 0; j < 5; j++) {
            plot_pixel(title_x + j, title_y + i, CYAN);
        }
    }
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 15; j++) {
            plot_pixel(title_x + 5 + j, title_y + i, CYAN);
            plot_pixel(title_x + 5 + j, title_y + 8 + i, CYAN);
        }
    }
    for (int i = 0; i < 10; i++) {
        for (int j = 0; j < 5; j++) {
            plot_pixel(title_x + 15 + j, title_y + 10 + i, CYAN);
        }
    }

    // L
    title_x += 25;
    for (int i = 0; i < 20; i++) {
        for (int j = 0; j < 5; j++) {
            plot_pixel(title_x + j, title_y + i, CYAN);
        }
    }
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 15; j++) {
            plot_pixel(title_x + 5 + j, title_y + 15 + i, CYAN);
        }
    }

    // Draw "REINFORCEMENT LEARNING" below the title
    // int subtitle_y = title_y + 30;
    // draw_text(VGA_WIDTH / 2 - 85, subtitle_y, "REINFORCEMENT LEARNING", YELLOW);

    // Draw a simple cart-pole graphic
    int demo_cart_x = VGA_WIDTH / 2;
    int demo_cart_y = VGA_HEIGHT - 80;

    // Draw cart (blue rectangle)
    for (int y = 0; y < 15; y++) {
        for (int x = 0; x < 40; x++) {
            plot_pixel(demo_cart_x - 20 + x, demo_cart_y + y, BLUE);
        }
    }

    // Draw pole (at an angle)
    int pole_end_x = demo_cart_x + 25;
    int pole_end_y = demo_cart_y - 50;

    for (int i = -2; i <= 2; i++) {
        draw_line(demo_cart_x + i, demo_cart_y, pole_end_x + i, pole_end_y, RED);
    }

    // Draw ground
    draw_line(demo_cart_x - 80, demo_cart_y + 20,
              demo_cart_x + 80, demo_cart_y + 20, WHITE);

    // Draw instructions
    int instr_y = VGA_HEIGHT - 50;
    draw_text(VGA_WIDTH / 2 - 80, instr_y, "HOW TO TRAIN YOUR POLE:", GREEN);
    
    instr_y += 10;
    draw_text(VGA_WIDTH / 2 - 120, instr_y, "2. PRESS KEY0 TO START/RESTART SIMULATION", WHITE);

    instr_y += 10;
    draw_text(VGA_WIDTH / 2 - 120, instr_y, "1. PRESS KEY1 TO TOGGLE TRAIN/INFERENCE", WHITE);

}

// Function to print training statistics
void print_training_stats() {
    char buffer[30]; // Declaring but using printf directly (buffer unused)
    
    // Print a header for clarity
    printf("\n===== CARTRL TRAINING STATISTICS =====\n");
    
    // Print basic episode stats
    printf("Episode: %d\n", stats.epoch);
    printf("Current Step: %d\n", stats.step);
    printf("Current Reward: %d\n", stats.total_reward);
    printf("Best Reward: %d\n", stats.best_reward);
    
    // Print memory buffer usage
    printf("Memory Buffer: %d/%d steps\n", memory.size, MAX_STEPS_PER_EPOCH);
    
    // Print learning parameters
    float current_lr = LEARNING_RATE * (1.0f - (float)stats.epoch / MAX_EPISODES);
    if (current_lr < LEARNING_RATE * 0.1f) current_lr = LEARNING_RATE * 0.1f;
    printf("Current Learning Rate: %.6f\n", current_lr);
    printf("Mode: %s\n", current_mode == MODE_TRAIN ? "TRAINING" : "INFERENCE");
    
    // Print state information
    printf("\nCurrent State:\n");
    printf("  Cart Position: %.4f\n", state.cart_position);
    printf("  Cart Velocity: %.4f\n", state.cart_velocity);
    printf("  Pole Angle: %.4f rad (%.2f deg)\n", state.pole_angle, state.pole_angle * 57.3);
    printf("  Pole Angular Velocity: %.4f rad/s\n", state.pole_angular_vel);
    
    // Weight statistics for each part of the network
    printf("\nNetwork Weight Statistics:\n");
    
    // Policy network stats
    calculate_weight_stats(nn.weights1, STATE_DIM, HIDDEN_DIM, "Policy: Input->Hidden1");
    calculate_weight_stats(nn.weights_h2h, HIDDEN_DIM, HIDDEN_DIM, "Policy: Hidden1->Hidden2");
    calculate_weight_stats(nn.weights2, HIDDEN_DIM, ACTION_DIM, "Policy: Hidden2->Output");
    
    // Value network stats
    calculate_weight_stats(nn.value_weights1, STATE_DIM, HIDDEN_DIM, "Value: Input->Hidden1");
    calculate_weight_stats(nn.value_weights_h2h, HIDDEN_DIM, HIDDEN_DIM, "Value: Hidden1->Hidden2");
    calculate_weight_stats(nn.value_weights2, HIDDEN_DIM, 1, "Value: Hidden2->Output");
    
    // Print a sample action distribution for current state
    float normalized_state[STATE_DIM];
    normalize_state(normalized_state, &state);
    
    float action_probs[ACTION_DIM];
    policy_forward(normalized_state, action_probs);
    
    printf("\nCurrent Action Distribution:\n");
    printf("  Left (0): %.4f\n", action_probs[0]);
    printf("  Right (1): %.4f\n", action_probs[1]);
    
    // Print estimated value of current state
    float value = value_forward(normalized_state);
    printf("Estimated State Value: %.4f\n", value);
    
    printf("=====================================\n\n");
}

// Helper function to calculate weight statistics (mean and std dev)
void calculate_weight_stats(float weights[][HIDDEN_DIM], int rows, int cols, char* name) {
    float sum = 0.0f;
    float sum_sq = 0.0f;
    int count = 0;
    float min_val = 999.0f;
    float max_val = -999.0f;
    
    // Calculate sum and sum of squares for mean and std dev
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            // Access the weight value based on the actual dimensions
            float val;
            
            if (j < HIDDEN_DIM) {
                // Direct access for arrays with second dimension <= HIDDEN_DIM
                val = weights[i][j];
            } else {
                // Skip accessing out-of-bounds memory
                continue;
            }
            
            sum += val;
            sum_sq += val * val;
            count++;
            
            if (val < min_val) min_val = val;
            if (val > max_val) max_val = val;
        }
    }
    
    // Avoid division by zero and ensure valid statistics
    if (count > 0) {
        float mean = sum / count;
        float variance = (sum_sq / count) - (mean * mean);
        
        // Avoid taking square root of negative number
        if (variance >= 0.0f) {
            float std_dev = my_sqrt(variance);
            printf("  %s: mean=%.6f, std=%.6f, min=%.6f, max=%.6f\n", 
                   name, mean, std_dev, min_val, max_val);
        } else {
            printf("  %s: mean=%.6f, std=invalid, min=%.6f, max=%.6f\n", 
                   name, mean, min_val, max_val);
        }
    } else {
        printf("  %s: mean=unavailable, std=unavailable, min=%.6f, max=%.6f\n", 
               name, min_val, max_val);
    }
}


// draw character - expanded character set
void draw_char(int x, int y, char c, short int color) {
    // Define font data for digits 0-9
    const unsigned char font_0[5] = {0x3E, 0x51, 0x49, 0x45, 0x3E}; // 0
    const unsigned char font_1[5] = {0x00, 0x42, 0x7F, 0x40, 0x00}; // 1
    const unsigned char font_2[5] = {0x42, 0x61, 0x51, 0x49, 0x46}; // 2
    const unsigned char font_3[5] = {0x21, 0x41, 0x45, 0x4B, 0x31}; // 3
    const unsigned char font_4[5] = {0x18, 0x14, 0x12, 0x7F, 0x10}; // 4
    const unsigned char font_5[5] = {0x27, 0x45, 0x45, 0x45, 0x39}; // 5
    const unsigned char font_6[5] = {0x3C, 0x4A, 0x49, 0x49, 0x30}; // 6
    const unsigned char font_7[5] = {0x01, 0x71, 0x09, 0x05, 0x03}; // 7
    const unsigned char font_8[5] = {0x36, 0x49, 0x49, 0x49, 0x36}; // 8
    const unsigned char font_9[5] = {0x06, 0x49, 0x49, 0x29, 0x1E}; // 9

    // Complete alphabet - all letters
    const unsigned char font_A[5] = {0x7E, 0x11, 0x11, 0x11, 0x7E}; // A
    const unsigned char font_B[5] = {0x7F, 0x49, 0x49, 0x49, 0x36}; // B
    const unsigned char font_C[5] = {0x3E, 0x41, 0x41, 0x41, 0x22}; // C
    const unsigned char font_D[5] = {0x7F, 0x41, 0x41, 0x22, 0x1C}; // D
    const unsigned char font_E[5] = {0x7F, 0x49, 0x49, 0x49, 0x41}; // E
    const unsigned char font_F[5] = {0x7F, 0x09, 0x09, 0x09, 0x01}; // F
    const unsigned char font_G[5] = {0x3E, 0x41, 0x49, 0x49, 0x7A}; // G
    const unsigned char font_H[5] = {0x7F, 0x08, 0x08, 0x08, 0x7F}; // H
    const unsigned char font_I[5] = {0x00, 0x41, 0x7F, 0x41, 0x00}; // I
    const unsigned char font_J[5] = {0x20, 0x40, 0x41, 0x3F, 0x01}; // J
    const unsigned char font_K[5] = {0x7F, 0x08, 0x14, 0x22, 0x41}; // K
    const unsigned char font_L[5] = {0x7F, 0x40, 0x40, 0x40, 0x40}; // L
    const unsigned char font_M[5] = {0x7F, 0x02, 0x0C, 0x02, 0x7F}; // M
    const unsigned char font_N[5] = {0x7F, 0x04, 0x08, 0x10, 0x7F}; // N
    const unsigned char font_O[5] = {0x3E, 0x41, 0x41, 0x41, 0x3E}; // O
    const unsigned char font_P[5] = {0x7F, 0x09, 0x09, 0x09, 0x06}; // P
    const unsigned char font_Q[5] = {0x3E, 0x41, 0x51, 0x21, 0x5E}; // Q
    const unsigned char font_R[5] = {0x7F, 0x09, 0x19, 0x29, 0x46}; // R
    const unsigned char font_S[5] = {0x46, 0x49, 0x49, 0x49, 0x31}; // S
    const unsigned char font_T[5] = {0x01, 0x01, 0x7F, 0x01, 0x01}; // T
    const unsigned char font_U[5] = {0x3F, 0x40, 0x40, 0x40, 0x3F}; // U
    const unsigned char font_V[5] = {0x1F, 0x20, 0x40, 0x20, 0x1F}; // V
    const unsigned char font_W[5] = {0x3F, 0x40, 0x38, 0x40, 0x3F}; // W
    const unsigned char font_X[5] = {0x63, 0x14, 0x08, 0x14, 0x63}; // X
    const unsigned char font_Y[5] = {0x07, 0x08, 0x70, 0x08, 0x07}; // Y
    const unsigned char font_Z[5] = {0x61, 0x51, 0x49, 0x45, 0x43}; // Z

    // Special characters
    const unsigned char font_space[5] = {0x00, 0x00, 0x00, 0x00, 0x00}; // Space
    const unsigned char font_colon[5] = {0x00, 0x36, 0x36, 0x00, 0x00}; // :
    const unsigned char font_dash[5] = {0x08, 0x08, 0x08, 0x08, 0x08}; // -
    const unsigned char font_period[5] = {0x00, 0x60, 0x60, 0x00, 0x00}; // .
    const unsigned char font_slash[5] = {0x20, 0x10, 0x08, 0x04, 0x02}; // /
    const unsigned char font_percent[5] = {0x26, 0x16, 0x08, 0x34, 0x32}; // %
    const unsigned char font_plus[5] = {0x08, 0x08, 0x3E, 0x08, 0x08}; // +
    const unsigned char font_equal[5] = {0x14, 0x14, 0x14, 0x14, 0x14}; // =

    const unsigned char* font_ptr = NULL;

    // Select font data based on character
    if (c == '0') font_ptr = font_0;
    else if (c == '1') font_ptr = font_1;
    else if (c == '2') font_ptr = font_2;
    else if (c == '3') font_ptr = font_3;
    else if (c == '4') font_ptr = font_4;
    else if (c == '5') font_ptr = font_5;
    else if (c == '6') font_ptr = font_6;
    else if (c == '7') font_ptr = font_7;
    else if (c == '8') font_ptr = font_8;
    else if (c == '9') font_ptr = font_9;
    else if (c == 'A' || c == 'a') font_ptr = font_A;
    else if (c == 'B' || c == 'b') font_ptr = font_B;
    else if (c == 'C' || c == 'c') font_ptr = font_C;
    else if (c == 'D' || c == 'd') font_ptr = font_D;
    else if (c == 'E' || c == 'e') font_ptr = font_E;
    else if (c == 'F' || c == 'f') font_ptr = font_F;
    else if (c == 'G' || c == 'g') font_ptr = font_G;
    else if (c == 'H' || c == 'h') font_ptr = font_H;
    else if (c == 'I' || c == 'i') font_ptr = font_I;
    else if (c == 'J' || c == 'j') font_ptr = font_J;
    else if (c == 'K' || c == 'k') font_ptr = font_K;
    else if (c == 'L' || c == 'l') font_ptr = font_L;
    else if (c == 'M' || c == 'm') font_ptr = font_M;
    else if (c == 'N' || c == 'n') font_ptr = font_N;
    else if (c == 'O' || c == 'o') font_ptr = font_O;
    else if (c == 'P' || c == 'p') font_ptr = font_P;
    else if (c == 'Q' || c == 'q') font_ptr = font_Q;
    else if (c == 'R' || c == 'r') font_ptr = font_R;
    else if (c == 'S' || c == 's') font_ptr = font_S;
    else if (c == 'T' || c == 't') font_ptr = font_T;
    else if (c == 'U' || c == 'u') font_ptr = font_U;
    else if (c == 'V' || c == 'v') font_ptr = font_V;
    else if (c == 'W' || c == 'w') font_ptr = font_W;
    else if (c == 'X' || c == 'x') font_ptr = font_X;
    else if (c == 'Y' || c == 'y') font_ptr = font_Y;
    else if (c == 'Z' || c == 'z') font_ptr = font_Z;
    else if (c == ' ') font_ptr = font_space;
    else if (c == ':') font_ptr = font_colon;
    else if (c == '-') font_ptr = font_dash;
    else if (c == '.') font_ptr = font_period;
    else if (c == '/') font_ptr = font_slash;
    else if (c == '%') font_ptr = font_percent;
    else if (c == '+') font_ptr = font_plus;
    else if (c == '=') font_ptr = font_equal;

    // draw char
    if (font_ptr != NULL) {
        for (int row = 0; row < 8; row++) {
            for (int col = 0; col < 5; col++) {
                if (font_ptr[col] & (1 << row)) {
                    plot_pixel(x + col, y + row, color);
                }
            }
        }
    } else {
        // unsupported characters -> draw a small rectangle
        for (int row = 2; row < 5; row++) {
            for (int col = 1; col < 4; col++) {
                plot_pixel(x + col, y + row, color);
            }
        }
    }
}

// draw text string
void draw_text(int x, int y, char* str, short int color) {
    int i = 0;
    while (str[i] != '\0') {
        draw_char(x + i * 6, y, str[i], color);
        i++;
    }
}

// int to string conversion
void int_to_str(int num, char* str) {
    // special case for 0
    if (num == 0) {
        str[0] = '0';
        str[1] = '\0';
        return;
    }

    // negative numbers
    int neg = 0;
    if (num < 0) {
        neg = 1;
        num = -num;
    }

    // convert digits in reverse order
    int i = 0;
    while (num > 0) {
        str[i++] = '0' + (num % 10);
        num /= 10;
    }

    // add negative sign if needed
    if (neg) str[i++] = '-';

    // add null terminator
    str[i] = '\0';

    // reverse string
    int j = 0;
    i--;
    while (j < i) {
        char temp = str[j];
        str[j] = str[i];
        str[i] = temp;
        j++;
        i--;
    }
}




// normalize state variables for neural network input
void normalize_state(float normalized_state[STATE_DIM], CartPoleState *cart_state) {
    // Position: [-2.4, 2.4] -> [-1, 1]
    normalized_state[0] = cart_state->cart_position / 2.4f;
    
    // Velocity: clip and normalize to [-1, 1]
    normalized_state[1] = cart_state->cart_velocity;
    if (normalized_state[1] > 10.0f) normalized_state[1] = 10.0f;
    if (normalized_state[1] < -10.0f) normalized_state[1] = -10.0f;
    normalized_state[1] /= 10.0f;
    
    // Angle: [-MAX_ANGLE_RAD, MAX_ANGLE_RAD] -> [-1, 1]
    normalized_state[2] = cart_state->pole_angle / MAX_ANGLE_RAD;
    
    // Angular velocity: clip and normalize to [-1, 1]
    normalized_state[3] = cart_state->pole_angular_vel;
    if (normalized_state[3] > 10.0f) normalized_state[3] = 10.0f;
    if (normalized_state[3] < -10.0f) normalized_state[3] = -10.0f;
    normalized_state[3] /= 10.0f;
    
    // Enhancement: apply a small emphasis to angle when moving in the wrong direction
    // This helps the agent learn to respond more quickly to dangerous situations
    if (cart_state->pole_angle * cart_state->pole_angular_vel > 0) {
        // If angle and angular velocity have same sign, pole is moving away from center
        normalized_state[2] *= 1.05f;
    }
}

// float -> string (simplified)
void float_to_str(float num, char* str, int precision) {
    int integer_part = (int)num;
    int_to_str(integer_part, str);

    int i = 0;
    while (str[i] != '\0') i++;

    str[i++] = '.';

    float fractional_part = my_abs(num - integer_part);
    for (int j = 0; j < precision; j++) {
        fractional_part *= 10.0f;
        int digit = (int)fractional_part;
        str[i++] = digit + '0';
        fractional_part -= digit;
    }

    str[i] = '\0';
}

// random initial disturbance to pole
void apply_random_disturbance() {
    float stability = 1.0f - my_abs(state.pole_angle) / MAX_ANGLE_RAD;
    if (stability < 0.0f) stability = 0.0f;

    // scale disturbance based on stability - more stable = can handle larger disturbance
    float max_disturbance = RANDOM_FORCE_MAX * stability;
    float random_force = random_range(-max_disturbance, max_disturbance);
    state.pole_angular_vel += random_force;
}

/**
 * Enhanced XORshift32 random number generator
 * Periodically refreshes entropy from hardware timer
 */
unsigned int xorshift32() {
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
 */
float random_float() {
    return (float)xorshift32() / 4294967295.0f;
}

/**
 * Get a random float in a specific range
 */
float random_range(float min, float max) {
    return min + random_float() * (max - min);
}

/**
 * Get a random integer in a specific range
 */
int random_int(int min, int max) {
    return min + (xorshift32() % (max - min + 1));
}

// Math helper functions (from the second file)
float my_abs(float x) {
    return x < 0 ? -x : x;
}

float my_max(float a, float b) {
    return a > b ? a : b;
}

float my_min(float a, float b) {
    return a < b ? a : b;
}

// Improved my_exp function with better numerical stability
float my_exp(float x) {
    // Prevent overflow/underflow with stricter bounds
    if (x > 15.0f) return 3269017.3724721f;  // e^15 is large but not too extreme
    if (x < -15.0f) return 1e-6f;  // Small positive number instead of 0

    // More accurate Taylor series for small values
    if (x >= -1.0f && x <= 1.0f) {
        float result = 1.0f + x;
        float term = x;
        for (int i = 2; i < 12; i++) {
            term *= x / i;
            result += term;
            // Prevent terms from becoming too small to matter
            if (my_abs(term) < 1e-10f) break;
        }
        return result;
    }

    // For larger ranges, use e^x = (e^(x/2))² with recursion
    if (x < -1.0f || x > 1.0f) {
        float half = my_exp(x * 0.5f);
        return half * half;
    }

    return 1.0f;  // Fallback (shouldn't reach here)
}


// fast approximation of logarithm 
float my_log(float x) {
    // Handle invalid inputs with a more graceful fallback
    if (x <= 1e-10f) return -23.0f;  // log(1e-10) ≈ -23, avoid extreme negatives

    // Clamp input for stability
    x = my_clamp(x, 1e-10f, 1e10f);

    // Normalize input to [1,2)
    float a = x;
    int exponent = 0;
    while (a >= 2.0f) {
        a *= 0.5f;
        exponent++;
    }
    while (a < 1.0f) {
        a *= 2.0f;
        exponent--;
    }

    // More robust polynomial approximation for log(1+x)
    float z = (a - 1.0f) / (a + 1.0f);
    float z2 = z * z;
    float result = 2.0f * z * (1.0f + z2/3.0f + z2*z2/5.0f + z2*z2*z2/7.0f);
    
    // log(2) ≈ 0.693147
    return result + exponent * 0.693147f;
}

float my_sqrt(float x) {
    // Newton's method for square root
    if (x <= 0.0f) return 0.0f;

    float result = x;
    for (int i = 0; i < 10; i++) {  // 10 iterations is typically enough
        result = 0.5f * (result + x / result);
    }

    return result;
}

// Function to copy memory (replacement for memcpy)
void my_memcpy(void *dest, void *src, int n) {
    char *d = (char *)dest;
    char *s = (char *)src;
    for (int i = 0; i < n; i++) {
        d[i] = s[i];
    }
}

// Math activation functions
float relu(float x) {
    return x > 0.0f ? x : 0.0f;
}

// Custom activation function - LeakyReLU instead of ReLU
// This helps with the dying ReLU problem
float leaky_relu(float x) {
    return x > 0.0f ? x : 0.01f * x;
}

// More robust policy_forward with numerical safeguards
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
        hidden1[i] = leaky_relu(hidden1[i]);  // Using improved activation function
    }

    // First hidden to second hidden layer
    for (int i = 0; i < HIDDEN_DIM; i++) {
        hidden2[i] = 0.0f;
        for (int j = 0; j < HIDDEN_DIM; j++) {
            hidden2[i] += hidden1[j] * nn.weights_h2h[j][i];
        }
        hidden2[i] += nn.bias_h2[i];
        hidden2[i] = leaky_relu(hidden2[i]);  // Using improved activation function
    }

    // Second hidden to output layer
    float output[ACTION_DIM];
    for (int i = 0; i < ACTION_DIM; i++) {
        output[i] = 0.0f;
        for (int j = 0; j < HIDDEN_DIM; j++) {
            output[i] += hidden2[j] * nn.weights2[j][i];
        }
        output[i] += nn.bias2[i];
        
        // Clamp outputs to prevent extreme values
        output[i] = my_clamp(output[i], -10.0f, 10.0f);
    }

    // Softmax activation with improved numerical stability
    float max_val = output[0];
    for (int i = 1; i < ACTION_DIM; i++) {
        if (output[i] > max_val) max_val = output[i];
    }

    // Subtract max for numerical stability
    float sum_exp = 0.0f;
    for (int i = 0; i < ACTION_DIM; i++) {
        probs[i] = my_exp(output[i] - max_val);
        sum_exp += probs[i];
    }

    // More robust division protection
    if (sum_exp < 1e-8f) sum_exp = 1e-8f;
    
    for (int i = 0; i < ACTION_DIM; i++) {
        probs[i] /= sum_exp;

        // Clamp probabilities to avoid numerical issues
        if (probs[i] < 0.001f) probs[i] = 0.001f;
        if (probs[i] > 0.999f) probs[i] = 0.999f;
    }

    // Renormalize after clamping
    sum_exp = 0.0f;
    for (int i = 0; i < ACTION_DIM; i++) {
        sum_exp += probs[i];
    }
    
    if (sum_exp > 0.0f) {
        for (int i = 0; i < ACTION_DIM; i++) {
            probs[i] /= sum_exp;
        }
    } else {
        // Fallback to uniform distribution in case of severe numerical problems
        for (int i = 0; i < ACTION_DIM; i++) {
            probs[i] = 1.0f / ACTION_DIM;
        }
    }
}

// Neural Network forward pass for value function - with two hidden layers
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
        hidden1[i] = hidden1[i] > 0.0f ? hidden1[i] : 0.01f * hidden1[i];  // LeakyReLU
    }

    // First hidden to second hidden layer - Match indexing with initialization
    for (int i = 0; i < HIDDEN_DIM; i++) {
        hidden2[i] = 0.0f;
        for (int j = 0; j < HIDDEN_DIM; j++) {
            hidden2[i] += hidden1[j] * nn.value_weights_h2h[j][i];  // [from][to] indexing
        }
        hidden2[i] += nn.value_bias_h2[i];
        hidden2[i] = hidden2[i] > 0.0f ? hidden2[i] : 0.01f * hidden2[i];  // LeakyReLU
    }

    // Second hidden to output layer
    float value = 0.0f;
    for (int j = 0; j < HIDDEN_DIM; j++) {
        value += hidden2[j] * nn.value_weights2[j][0];
    }
    value += nn.value_bias2[0];

    return value;
}

// Helper function to clamp values
float my_clamp(float value, float min, float max) {
    return value < min ? min : (value > max ? max : value);
}

// Initialize the neural network with small random weights
void init_network() {
    // Mix in timer entropy for better initialization
    random_seed ^= timer_value();

    // Ensure random seed has enough entropy
    for (int i = 0; i < 10; i++) {
        random_seed = xorshift32();
    }

    // Kaiming initialization scale factors
    float init_range1 = sqrtf(2.0f / STATE_DIM);     // Input to first hidden
    float init_range2 = sqrtf(2.0f / HIDDEN_DIM);    // Hidden to hidden
    float init_range3 = sqrtf(2.0f / HIDDEN_DIM);    // Hidden to output

    // Initialize policy network weights
    // First layer - Input to first hidden layer
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < HIDDEN_DIM; j++) {
            nn.weights1[i][j] = (2.0f * random_float() - 1.0f) * init_range1;
            nn.value_weights1[i][j] = (2.0f * random_float() - 1.0f) * init_range1;
        }
    }
    
    // Hidden-to-hidden layer - Ensure consistent indexing [from][to]
    for (int i = 0; i < HIDDEN_DIM; i++) {
        for (int j = 0; j < HIDDEN_DIM; j++) {
            nn.weights_h2h[i][j] = (2.0f * random_float() - 1.0f) * init_range2;
            nn.value_weights_h2h[i][j] = (2.0f * random_float() - 1.0f) * init_range2;
        }
    }
    
    // Output layer - Second hidden layer to output
    for (int i = 0; i < HIDDEN_DIM; i++) {
        for (int j = 0; j < ACTION_DIM; j++) {
            nn.weights2[i][j] = (2.0f * random_float() - 1.0f) * init_range3;
        }
        nn.value_weights2[i][0] = (2.0f * random_float() - 1.0f) * init_range3;
    }

    // Initialize all biases
    for (int i = 0; i < HIDDEN_DIM; i++) {
        // First hidden layer biases
        nn.bias1[i] = 0.01f * (random_float() - 0.5f);
        nn.value_bias1[i] = 0.01f * (random_float() - 0.5f);
        
        // Second hidden layer biases
        nn.bias_h2[i] = 0.01f * (random_float() - 0.5f);
        nn.value_bias_h2[i] = 0.01f * (random_float() - 0.5f);
    }

    // Initialize output biases with slight bias for better initial performance
    nn.bias2[0] = -0.1f;  // Slight bias against going left
    nn.bias2[1] = 0.1f;   // Slight bias toward going right

    // Initialize value function to predict small positive values (optimistic initialization)
    nn.value_bias2[0] = 0.0f;

    // Verify network produces reasonable initial outputs
    float test_state[STATE_DIM] = {0, 0, 0.1f, 0}; // Slightly tilted right
    float probs[ACTION_DIM];
    policy_forward(test_state, probs);

    // Ensure the initial policy responds correctly to a tilted pole
    if (probs[1] < 0.55f) {  // Should favor moving right for a right-tilted pole
        nn.bias2[0] = -0.2f;
        nn.bias2[1] = 0.2f;
    }

    // Run the policy on a pole tilted left as well
    test_state[2] = -0.1f;  // Slightly tilted left
    policy_forward(test_state, probs);

    // Ensure the initial policy responds correctly to a left-tilted pole
    if (probs[0] < 0.55f) {  // Should favor moving left for a left-tilted pole
        nn.bias2[0] = 0.2f;
        nn.bias2[1] = -0.2f;
    }
}

// Calculate log probability of action
float log_prob(float probs[ACTION_DIM], int action) {
    return my_log(probs[action]);
}

// More robust compute_advantages function
void compute_advantages() {
    // First compute returns (discounted sum of future rewards)
    for (int t = memory.size - 1; t >= 0; t--) {
        float next_value;
        if (memory.dones[t]) {
            next_value = 0.0f;
        } else {
            next_value = value_forward(memory.next_states[t]);
            // Clamp value estimates for stability
            next_value = my_clamp(next_value, -50.0f, 50.0f);
        }
        memory.returns[t] = memory.rewards[t] + GAMMA * next_value;
        
        // Clamp returns to prevent extreme values
        memory.returns[t] = my_clamp(memory.returns[t], -100.0f, 100.0f);
    }

    // Compute GAE advantages with safety checks
    float gae = 0.0f;
    for (int t = memory.size - 1; t >= 0; t--) {
        float next_value;
        if (memory.dones[t]) {
            next_value = 0.0f;
        } else {
            next_value = value_forward(memory.next_states[t]);
            // Clamp value estimates for stability
            next_value = my_clamp(next_value, -50.0f, 50.0f);
        }
        
        // Clamp values to prevent extreme deltas
        float current_value = my_clamp(memory.values[t], -50.0f, 50.0f);
        float reward = my_clamp(memory.rewards[t], -50.0f, 50.0f);
        
        float delta = reward + GAMMA * next_value - current_value;
        
        // Clamp delta for stability
        delta = my_clamp(delta, -10.0f, 10.0f);
        
        // Clamp previous GAE to prevent accumulation of extreme values
        gae = my_clamp(gae, -10.0f, 10.0f);
        
        gae = delta + GAMMA * LAMBDA * gae;
        
        // Store clamped advantage
        memory.advantages[t] = my_clamp(gae, -10.0f, 10.0f);
    }

    // Normalize advantages for better training stability
    if (memory.size > 1) {
        // Calculate mean and std of advantages
        float sum = 0.0f, sum_sq = 0.0f;
        for (int t = 0; t < memory.size; t++) {
            sum += memory.advantages[t];
            sum_sq += memory.advantages[t] * memory.advantages[t];
        }

        float mean = sum / memory.size;
        float variance = (sum_sq / memory.size) - (mean * mean);
        
        // Ensure variance is positive
        if (variance < 1e-8f) variance = 1e-8f;
        
        float std_dev = my_sqrt(variance);

        // Normalize advantages with enhanced stability
        for (int t = 0; t < memory.size; t++) {
            memory.advantages[t] = (memory.advantages[t] - mean) / std_dev;
            
            // Final clamp to ensure reasonable values
            memory.advantages[t] = my_clamp(memory.advantages[t], -5.0f, 5.0f);
        }
    }
}
// Complete update_network function with robust numerical stability
void update_network() {
    // Adaptive learning rate with a lower bound for stability
    float current_lr = LEARNING_RATE * (1.0f - (float)stats.epoch / MAX_EPISODES);
    current_lr = my_clamp(current_lr, LEARNING_RATE * 0.1f, LEARNING_RATE);

    // Define maximum gradient magnitude for clipping
    const float MAX_GRAD = 1.0f;
    
    // Early termination if memory buffer is too small
    if (memory.size < 4) {
        return;  // Avoid updates with insufficient data
    }

    // Perform multiple epochs of updates
    for (int epoch = 0; epoch < PPO_EPOCHS; epoch++) {
        // Initialize parameter gradients to zero
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
        for (int i = 0; i < memory.size; i++) indices[i] = i;

        // Fisher-Yates shuffle
        for (int i = memory.size-1; i > 0; i--) {
            int j = random_int(0, i);
            int temp = indices[i];
            indices[i] = indices[j];
            indices[j] = temp;
        }

        // Process mini-batches with a minimum size to ensure stability
        int batch_size = my_max(4, memory.size / 4);
        for (int start = 0; start < memory.size; start += batch_size) {
            int end = my_min(start + batch_size, memory.size);
            
            // Skip processing if batch is too small
            if (end - start < 2) continue;

            for (int b = start; b < end; b++) {
                int t = indices[b];
                float state[STATE_DIM];
                my_memcpy(state, memory.states[t], sizeof(float)*STATE_DIM);

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
                    hidden1[i] = hidden1[i] > 0.0f ? hidden1[i] : 0.01f * hidden1[i];  // LeakyReLU
                }

                // Second hidden layer
                for (int i = 0; i < HIDDEN_DIM; i++) {
                    hidden2[i] = 0.0f;
                    for (int j = 0; j < HIDDEN_DIM; j++) {
                        hidden2[i] += hidden1[j] * nn.weights_h2h[j][i];
                    }
                    hidden2[i] += nn.bias_h2[i];
                    hidden2[i] = hidden2[i] > 0.0f ? hidden2[i] : 0.01f * hidden2[i];  // LeakyReLU
                }

                // Output layer
                for (int i = 0; i < ACTION_DIM; i++) {
                    output[i] = 0.0f;
                    for (int j = 0; j < HIDDEN_DIM; j++) {
                        output[i] += hidden2[j] * nn.weights2[j][i];
                    }
                    output[i] += nn.bias2[i];
                    // Clamp outputs to prevent extreme values
                    output[i] = my_clamp(output[i], -10.0f, 10.0f);
                }

                // Softmax calculation with numerical stability
                float max_val = output[0];
                for (int i = 1; i < ACTION_DIM; i++) {
                    if (output[i] > max_val) max_val = output[i];
                }

                // Subtract max for numerical stability
                float sum_exp = 0.0f;
                for (int i = 0; i < ACTION_DIM; i++) {
                    probs[i] = my_exp(output[i] - max_val);
                    sum_exp += probs[i];
                }

                // Avoid division by zero
                if (sum_exp < 1e-8f) sum_exp = 1e-8f;
                
                for (int i = 0; i < ACTION_DIM; i++) {
                    probs[i] /= sum_exp;
                    // Clamp probabilities to avoid numerical issues
                    if (probs[i] < 0.001f) probs[i] = 0.001f;
                    if (probs[i] > 0.999f) probs[i] = 0.999f;
                }

                // Renormalize after clamping
                sum_exp = probs[0] + probs[1];
                if (sum_exp < 1e-8f) sum_exp = 1.0f;  // Safety check
                probs[0] /= sum_exp;
                probs[1] /= sum_exp;

                // Log probability and PPO ratio with safeguards
                float current_log_prob = my_log(probs[memory.actions[t]]);
                
                // Clamp log prob difference to prevent extreme ratios
                float log_prob_diff = current_log_prob - memory.log_probs[t];
                log_prob_diff = my_clamp(log_prob_diff, -10.0f, 10.0f);  // Prevent extreme values
                
                float ratio = my_exp(log_prob_diff);
                ratio = my_clamp(ratio, 0.1f, 10.0f);  // Prevent ratio explosion

                float advantage = memory.advantages[t];
                advantage = my_clamp(advantage, -10.0f, 10.0f);  // Clip advantage for stability
                
                float clip_ratio = my_clamp(ratio, 1.0f-CLIP_EPSILON, 1.0f+CLIP_EPSILON);
                
                // Variable unused - removed warning
                // float policy_loss = -my_min(ratio*advantage, clip_ratio*advantage);

                // Policy gradient for output layer with clamping
                float doutput[ACTION_DIM] = {0};
                if (ratio < 1.0f - CLIP_EPSILON || ratio > 1.0f + CLIP_EPSILON) {
                    // Use clipped gradient
                    float clip_factor = clip_ratio / ratio;
                    clip_factor = my_clamp(clip_factor, 0.1f, 10.0f);  // Prevent extreme clip factors
                    
                    for (int i = 0; i < ACTION_DIM; i++) {
                        if (i == memory.actions[t]) {
                            doutput[i] = -clip_factor * advantage * (1.0f - probs[i]);
                        } else {
                            doutput[i] = clip_factor * advantage * probs[i];
                        }
                        // Clamp gradients
                        doutput[i] = my_clamp(doutput[i], -5.0f, 5.0f);
                    }
                } else {
                    // Use unclipped gradient
                    for (int i = 0; i < ACTION_DIM; i++) {
                        if (i == memory.actions[t]) {
                            doutput[i] = -advantage * (1.0f - probs[i]);
                        } else {
                            doutput[i] = advantage * probs[i];
                        }
                        // Clamp gradients
                        doutput[i] = my_clamp(doutput[i], -5.0f, 5.0f);
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
                    // Apply leaky ReLU derivative
                    dhidden2[i] = grad * (hidden2[i] > 0 ? 1.0f : 0.01f);
                    // Clamp gradients
                    dhidden2[i] = my_clamp(dhidden2[i], -5.0f, 5.0f);
                }

                // First hidden layer gradients
                float dhidden1[HIDDEN_DIM] = {0};
                for (int i = 0; i < HIDDEN_DIM; i++) {
                    float grad = 0;
                    for (int j = 0; j < HIDDEN_DIM; j++) {
                        grad += dhidden2[j] * nn.weights_h2h[i][j];
                    }
                    // Apply leaky ReLU derivative
                    dhidden1[i] = grad * (hidden1[i] > 0 ? 1.0f : 0.01f);
                    // Clamp gradients
                    dhidden1[i] = my_clamp(dhidden1[i], -5.0f, 5.0f);
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
                    v_hidden1[i] = v_hidden1[i] > 0.0f ? v_hidden1[i] : 0.01f * v_hidden1[i];  // LeakyReLU
                }
                
                // Second hidden layer
                for (int i = 0; i < HIDDEN_DIM; i++) {
                    v_hidden2[i] = 0.0f;
                    for (int j = 0; j < HIDDEN_DIM; j++) {
                        v_hidden2[i] += v_hidden1[j] * nn.value_weights_h2h[j][i];
                    }
                    v_hidden2[i] += nn.value_bias_h2[i];
                    v_hidden2[i] = v_hidden2[i] > 0.0f ? v_hidden2[i] : 0.01f * v_hidden2[i];  // LeakyReLU
                }
                
                // Output value
                float value = 0.0f;
                for (int i = 0; i < HIDDEN_DIM; i++) {
                    value += v_hidden2[i] * nn.value_weights2[i][0];
                }
                value += nn.value_bias2[0];
                
                // Clamp value to prevent extreme loss
                value = my_clamp(value, -50.0f, 50.0f);
                
                // Value loss gradient (MSE)
                float target_value = memory.returns[t];
                target_value = my_clamp(target_value, -50.0f, 50.0f);  // Clamp target
                
                float value_diff = value - target_value;
                value_diff = my_clamp(value_diff, -10.0f, 10.0f);  // Prevent extreme differences
                
                float dvalue = 2.0f * value_diff;
                dvalue = my_clamp(dvalue, -10.0f, 10.0f);  // Clamp gradient
                
                // Backpropagate through value network
                // Second hidden layer gradients
                float dv_hidden2[HIDDEN_DIM] = {0};
                for (int i = 0; i < HIDDEN_DIM; i++) {
                    dv_hidden2[i] = dvalue * nn.value_weights2[i][0];
                    dv_hidden2[i] *= (v_hidden2[i] > 0.0f ? 1.0f : 0.01f); // LeakyReLU derivative
                    dv_hidden2[i] = my_clamp(dv_hidden2[i], -5.0f, 5.0f);  // Clamp gradient
                }
                
                // First hidden layer gradients
                float dv_hidden1[HIDDEN_DIM] = {0};
                for (int i = 0; i < HIDDEN_DIM; i++) {
                    float grad = 0;
                    for (int j = 0; j < HIDDEN_DIM; j++) {
                        grad += dv_hidden2[j] * nn.value_weights_h2h[i][j];
                    }
                    dv_hidden1[i] = grad * (v_hidden1[i] > 0.0f ? 1.0f : 0.01f); // LeakyReLU derivative
                    dv_hidden1[i] = my_clamp(dv_hidden1[i], -5.0f, 5.0f);  // Clamp gradient
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

            // Apply global gradient clipping
            float policy_grad_norm = 0.0f;
            float value_grad_norm = 0.0f;
            
            // Calculate policy gradient norm
            for (int i = 0; i < STATE_DIM; i++) {
                for (int j = 0; j < HIDDEN_DIM; j++) {
                    policy_grad_norm += dw1[i][j] * dw1[i][j];
                }
            }
            for (int i = 0; i < HIDDEN_DIM; i++) {
                policy_grad_norm += db1[i] * db1[i];
                for (int j = 0; j < HIDDEN_DIM; j++) {
                    policy_grad_norm += dw_h2h[i][j] * dw_h2h[i][j];
                }
                policy_grad_norm += db_h2[i] * db_h2[i];
                for (int j = 0; j < ACTION_DIM; j++) {
                    policy_grad_norm += dw2[i][j] * dw2[i][j];
                }
            }
            for (int j = 0; j < ACTION_DIM; j++) {
                policy_grad_norm += db2[j] * db2[j];
            }
            
            // Calculate value gradient norm
            for (int i = 0; i < STATE_DIM; i++) {
                for (int j = 0; j < HIDDEN_DIM; j++) {
                    value_grad_norm += vdw1[i][j] * vdw1[i][j];
                }
            }
            for (int i = 0; i < HIDDEN_DIM; i++) {
                value_grad_norm += vdb1[i] * vdb1[i];
                for (int j = 0; j < HIDDEN_DIM; j++) {
                    value_grad_norm += vdw_h2h[i][j] * vdw_h2h[i][j];
                }
                value_grad_norm += vdb_h2[i] * vdb_h2[i];
                value_grad_norm += vdw2[i][0] * vdw2[i][0];
            }
            value_grad_norm += vdb2[0] * vdb2[0];
            
            policy_grad_norm = my_sqrt(policy_grad_norm);
            value_grad_norm = my_sqrt(value_grad_norm);
            
            // Scale down gradients if norm is too large
            float policy_scale = 1.0f;
            float value_scale = 1.0f;
            
            if (policy_grad_norm > MAX_GRAD) {
                policy_scale = MAX_GRAD / (policy_grad_norm + 1e-8f);
            }
            
            if (value_grad_norm > MAX_GRAD) {
                value_scale = MAX_GRAD / (value_grad_norm + 1e-8f);
            }
            
            // Apply gradients after each mini-batch with proper scaling
            float batch_scale = current_lr / (end - start);
            batch_scale = my_clamp(batch_scale, 0.0f, 0.01f);  // Limit maximum update size

            // Update policy network
            // First layer weights and biases
            for (int i = 0; i < STATE_DIM; i++) {
                for (int j = 0; j < HIDDEN_DIM; j++) {
                    nn.weights1[i][j] -= dw1[i][j] * batch_scale * policy_scale;
                    nn.value_weights1[i][j] -= vdw1[i][j] * batch_scale * value_scale;
                    
                    // Check for extreme values after update
                    nn.weights1[i][j] = my_clamp(nn.weights1[i][j], -20.0f, 20.0f);
                    nn.value_weights1[i][j] = my_clamp(nn.value_weights1[i][j], -20.0f, 20.0f);
                }
            }
            for (int i = 0; i < HIDDEN_DIM; i++) {
                nn.bias1[i] -= db1[i] * batch_scale * policy_scale;
                nn.value_bias1[i] -= vdb1[i] * batch_scale * value_scale;
                
                // Check for extreme values after update
                nn.bias1[i] = my_clamp(nn.bias1[i], -20.0f, 20.0f);
                nn.value_bias1[i] = my_clamp(nn.value_bias1[i], -20.0f, 20.0f);
            }

            // Hidden-to-hidden layer weights and biases
            for (int i = 0; i < HIDDEN_DIM; i++) {
                for (int j = 0; j < HIDDEN_DIM; j++) {
                    nn.weights_h2h[i][j] -= dw_h2h[i][j] * batch_scale * policy_scale;
                    nn.value_weights_h2h[i][j] -= vdw_h2h[i][j] * batch_scale * value_scale;
                    
                    // Check for extreme values after update
                    nn.weights_h2h[i][j] = my_clamp(nn.weights_h2h[i][j], -20.0f, 20.0f);
                    nn.value_weights_h2h[i][j] = my_clamp(nn.value_weights_h2h[i][j], -20.0f, 20.0f);
                }
            }
            for (int i = 0; i < HIDDEN_DIM; i++) {
                nn.bias_h2[i] -= db_h2[i] * batch_scale * policy_scale;
                nn.value_bias_h2[i] -= vdb_h2[i] * batch_scale * value_scale;
                
                // Check for extreme values after update
                nn.bias_h2[i] = my_clamp(nn.bias_h2[i], -20.0f, 20.0f);
                nn.value_bias_h2[i] = my_clamp(nn.value_bias_h2[i], -20.0f, 20.0f);
            }

            // Output layer weights and biases
            for (int i = 0; i < HIDDEN_DIM; i++) {
                for (int j = 0; j < ACTION_DIM; j++) {
                    nn.weights2[i][j] -= dw2[i][j] * batch_scale * policy_scale;
                    
                    // Check for extreme values after update
                    nn.weights2[i][j] = my_clamp(nn.weights2[i][j], -20.0f, 20.0f);
                }
                nn.value_weights2[i][0] -= vdw2[i][0] * batch_scale * value_scale;
                
                // Check for extreme values after update
                nn.value_weights2[i][0] = my_clamp(nn.value_weights2[i][0], -20.0f, 20.0f);
            }
            for (int i = 0; i < ACTION_DIM; i++) {
                nn.bias2[i] -= db2[i] * batch_scale * policy_scale;
                
                // Check for extreme values after update
                nn.bias2[i] = my_clamp(nn.bias2[i], -20.0f, 20.0f);
            }
            nn.value_bias2[0] -= vdb2[0] * batch_scale * value_scale;
            
            // Check for extreme values after update
            nn.value_bias2[0] = my_clamp(nn.value_bias2[0], -20.0f, 20.0f);
            
            // Periodically check for NaN values
            if (start == 0 && epoch == 0) {
                for (int i = 0; i < STATE_DIM; i++) {
                    for (int j = 0; j < HIDDEN_DIM; j++) {
                        // Using the fact that NaN is never equal to itself
                        if (nn.weights1[i][j] != nn.weights1[i][j]) {
                            // NaN detected - reset this weight
                            nn.weights1[i][j] = (2.0f * random_float() - 1.0f) * 0.1f;
                        }
                        if (nn.value_weights1[i][j] != nn.value_weights1[i][j]) {
                            nn.value_weights1[i][j] = (2.0f * random_float() - 1.0f) * 0.1f;
                        }
                    }
                }
            }
        }
    }
    
    // Final check for NaN values in key parameters
    int nan_count = 0;
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < min(5, HIDDEN_DIM); j++) {  // Check a subset for efficiency
            if (nn.weights1[i][j] != nn.weights1[i][j]) {
                nan_count++;
            }
        }
    }
    
    // If too many NaNs detected, reset the network
    if (nan_count > 3) {
        printf("WARNING: %d NaN values detected in network weights. Resetting network.\n", nan_count);
        init_network();
    }
}

int min(x,y){
    if (x>y){
        return y;
    }
    else{
        return x;
    }
}
