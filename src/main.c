#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <pthread.h>
#include <time.h>
#include <signal.h>
#include <ctype.h>
#include <errno.h>

// Configuration
#define SERIAL_PORT "/dev/cu.usbmodem"  // Base name for Feather's USB serial port on Mac
#define BAUD_RATE B115200
#define GRAVITY 9.8f
#define CART_MASS 1.0f
#define POLE_MASS 0.2f
#define POLE_HALF_LENGTH 0.75f


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
// Control parameters
#define MAX_ANGLE_RAD 0.35f
#define MAX_POSITION 5.0f
#define MAX_ANGULAR_VELOCITY 4.0f

#define FORCE_MAG 150 // Matches: MOTOR_MAX_SPEED in Arduino
#define MOTOR_MAX_SPEED 150        // Maximum motor speed value

#define TIME_STEP 0.02f // Matches: 1/CONTROL_FREQUENCY (1/50 = 0.02) in Arduino
#define CONTROL_INTERVAL_MS 20 // Matches: CONTROL_INTERVAL (1000/50 = 20) in Arduino
// NO control FREQ
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 


// Neural network parameters
#define STATE_DIM 4
#define ACTION_DIM 2
#define HIDDEN_DIM 64
#define MAX_EPISODES 1000
#define MAX_STEPS_PER_EPISODE 400
#define PPO_EPOCHS 12
#define GAMMA 0.98f
#define LAMBDA 0.95f
#define CLIP_EPSILON 0.1f
#define LEARNING_RATE 0.007f
#define EXPLORE_RATE 0.25f
#define WEIGHTS_FILE "cart_pole_weights_best.bin"
unsigned long last_action_time = 0;
#define MIN_ACTION_DURATION 200 // Minimum milliseconds between motor changes



// State structure
typedef struct {
    float pole_angle;
    float pole_angular_vel;
    float cart_position;
    float cart_velocity;
    int time_steps;
    int is_terminal;
} CartPoleState;

// Neural network structure
typedef struct {
    // Policy network
    float weights1[STATE_DIM][HIDDEN_DIM];
    float bias1[HIDDEN_DIM];
    float weights_h2h[HIDDEN_DIM][HIDDEN_DIM];
    float bias_h2[HIDDEN_DIM];
    float weights2[HIDDEN_DIM][ACTION_DIM];
    float bias2[ACTION_DIM];
    
    // Value network
    float value_weights1[STATE_DIM][HIDDEN_DIM];
    float value_bias1[HIDDEN_DIM];
    float value_weights_h2h[HIDDEN_DIM][HIDDEN_DIM];
    float value_bias_h2[HIDDEN_DIM];
    float value_weights2[HIDDEN_DIM][1];
    float value_bias2[1];
    float reward;
} NeuralNetwork;

// Memory buffer to store experiences
typedef struct {
    float states[MAX_STEPS_PER_EPISODE][STATE_DIM];
    float next_states[MAX_STEPS_PER_EPISODE][STATE_DIM];
    int actions[MAX_STEPS_PER_EPISODE];
    float rewards[MAX_STEPS_PER_EPISODE];
    float values[MAX_STEPS_PER_EPISODE];
    float log_probs[MAX_STEPS_PER_EPISODE];
    float advantages[MAX_STEPS_PER_EPISODE];
    float returns[MAX_STEPS_PER_EPISODE];
    int dones[MAX_STEPS_PER_EPISODE];
    int size;
} Memory;

// Global variables
CartPoleState state;
NeuralNetwork nn;
NeuralNetwork best_nn;
NeuralNetwork training_backup;
int waiting_for_start = 0;
Memory memory;
int serial_fd = -1;
int running = 1;
int training_mode = 0;
int episode_count = 0;
int current_episode_steps = 0;
int current_episode_reward = 0;
int best_reward = 0;
int best_steps = 0;
int best_epoch = 0;
int backup_valid = 0;
pthread_mutex_t state_mutex = PTHREAD_MUTEX_INITIALIZER;

// Function prototypes
void init_network(); // x
void load_weights(const char* filename); // x
void save_weights(const char* filename); // x
int open_serial_port(); // x
void send_command(const char* command);  // x
void send_motor_command(int speed); // x
void* serial_reader_thread(void* arg); // x
void* control_thread(void* arg); // x
void normalize_state(float normalized_state[STATE_DIM], CartPoleState* cart_state); // x
void policy_forward(float state[STATE_DIM], float probs[ACTION_DIM]); // x
float value_forward(float state[STATE_DIM]); // x
int sample_action(float probs[ACTION_DIM]); // x
int apply_action(int action); // x
float log_prob(float probs[ACTION_DIM], int action); // x

void reset_state(); // x
void start_episode(); // x
void stop_episode(); // x
void print_help(); // x
float calculate_reward(); // x
float random_float(); // x
void compute_advantages(); // x
void update_network(); // x

float my_abs(float x); // x
float my_max(float a, float b); // x
float my_min(float a, float b); // x
float my_clamp(float value, float min, float max); // x
float my_sqrt(float x); // x
float my_exp(float x); // x
float my_log(float x); // x
float leaky_relu(float x); // x

int is_terminal_state(); // x
void save_best_weights(); // x
void load_best_weights(); // x
void restore_training_weights(); // x
void print_training_stats(); // x
void my_memcpy(void *dest, void *src, int n); // x
int random_int(int min, int max);
int min(int x, int y);
void calculate_weight_stats(float weights[][HIDDEN_DIM], int rows, int cols, char *name);



// Ctrl+C handler
void handle_signal(int sig) {
    printf("\nReceived signal %d. Shutting down...\n", sig);
    // Stop episode and motors before exiting
    send_command("STOP");
    send_motor_command(0);
    running = 0;
    usleep(100000);  // Give some time for threads to clean up
}

int main(int argc, char* argv[]) {
    printf("CartPole Controller for Mac\n");
    printf("---------------------------\n");
    
    // Set up signal handler for graceful shutdown
    signal(SIGINT, handle_signal);
    
    // Initialize neural network
    init_network();
    
    // Try to load saved weights if available
    if (access(WEIGHTS_FILE, F_OK) != -1) {
        printf("Loading saved weights from %s\n", WEIGHTS_FILE);
        load_weights(WEIGHTS_FILE);
    } else {
        printf("No saved weights found. Using random initialization.\n");
    }
    
    // Open serial port
    if (open_serial_port() < 0) {
        printf("Failed to open serial port. Exiting.\n");
        return 1;
    }
    
    // Reset Arduino state
    reset_state();
    
    // Start reader thread for processing incoming data from Arduino
    pthread_t reader_thread_id, control_thread_id;
    if (pthread_create(&reader_thread_id, NULL, serial_reader_thread, NULL) != 0) {
        printf("Failed to create reader thread\n");
        return 1;
    }
    
    // Start control thread
    if (pthread_create(&control_thread_id, NULL, control_thread, NULL) != 0) {
        printf("Failed to create control thread\n");
        return 1;
    }
    
    // Print instructions
    print_help();
    
    // Main loop for user input
    char input[256];
    while (running) {
        printf("Command> ");
        fflush(stdout);
        
        if (fgets(input, sizeof(input), stdin) == NULL) {
            break;
        }
        
        // Remove newline
        input[strcspn(input, "\n")] = 0;
        
        // Process commands
        if (strcmp(input, "start") == 0 || strcmp(input, "s") == 0) {
            start_episode();
        }
        else if (strcmp(input, "stop") == 0 || strcmp(input, "x") == 0) {
            stop_episode();
        }
        else if (strcmp(input, "reset") == 0 || strcmp(input, "r") == 0) {
            reset_state();
        }
        else if (strcmp(input, "train") == 0 || strcmp(input, "t") == 0) {
            if (training_mode == 0) {
                training_mode = 1;
                printf("Training mode activated. Actions will be selected by the neural network.\n");
            }
            // Save current weights before starting training in case we want to revert
            if (!backup_valid) {
                my_memcpy(&training_backup, &nn, sizeof(NeuralNetwork));
                backup_valid = 1;
                printf("Training weights backed up.\n");
            }
        }
        // else if (strcmp(input, "inference") == 0 || strcmp(input, "i") == 0) {
        //     if (training_mode == 1) {
        //         // Switching from TRAIN to INFERENCE mode
        //         my_memcpy(&training_backup, &nn, sizeof(NeuralNetwork));
        //         backup_valid = 1;
                
        //         // Load best weights for inference if available
        //         if (best_epoch > 0) {
        //             my_memcpy(&nn, &best_nn, sizeof(NeuralNetwork));
        //             printf("Switched to INFERENCE mode using best weights (Epoch %d, Reward %d)\n", 
        //                   best_epoch, (int)best_nn.reward);
        //         }
        //     }
        //     else if (state.time_steps > 0) {
        //         // If in training mode, use neural network to select actions
        //         if (training_mode) {
        //             // Normalize the state for the neural network
        //             float normalized_state[STATE_DIM];
        //             normalize_state(normalized_state, &state);
                    
        //             // Store experience in memory buffer
        //             if (memory.size < MAX_STEPS_PER_EPISODE) {
        //                 // Store current state
        //                 my_memcpy(memory.states[memory.size], normalized_state, sizeof(float) * STATE_DIM);
                        
        //                 // Get action probabilities from policy
        //                 float action_probs[ACTION_DIM];
        //                 policy_forward(normalized_state, action_probs);
                        
        //                 // Add some exploration to training
        //                 int action;
        //                 float explore_rate = my_max(EXPLORE_RATE, 0.6f * (1.0f - (float)episode_count / 200.0f));
        //                 if (random_float() < explore_rate) {
        //                     // Take random action with probability explore_rate
        //                     action = (random_float() < 0.5f) ? 0 : 1;
        //                 } else {
        //                     // Sample from policy distribution
        //                     action = sample_action(action_probs);
        //                 }
        //                 printf("DEBUG: Selected action=%d", action);
        //                 // Store log probability of the chosen action
        //                 memory.log_probs[memory.size] = log_prob(action_probs, action);
                        
        //                 // Store action
        //                 memory.actions[memory.size] = action;
                        
        //                 // Store value estimate
        //                 memory.values[memory.size] = value_forward(normalized_state);
                        
        //                 // Increment memory size after storing experience
        //                 memory.size++;
                        
        //                 // Apply the selected action
        //                 apply_action(action);
        //             }
        //         } else {
        //             printf("Switched to INFERENCE mode (no best weights yet)\n");
        //         }
                
        //         training_mode = 0;
        //     }
        // }
        else if (strcmp(input, "inference") == 0 || strcmp(input, "i") == 0) {
            // Always backup current weights first
            my_memcpy(&training_backup, &nn, sizeof(NeuralNetwork));
            backup_valid = 1;
            
            // Only switch to best weights if we were in training mode and have best weights
            if (training_mode == 1 && best_epoch > 0) {
                my_memcpy(&nn, &best_nn, sizeof(NeuralNetwork));
                printf("Switched to INFERENCE mode using best weights (Epoch %d, Reward %d)\n", 
                      best_epoch, (int)best_nn.reward);
            } else {
                printf("Switched to INFERENCE mode using currently loaded weights\n");
            }
            
            // Clear memory buffer to prevent any accidental updates
            memory.size = 0;
            
            // Always set to inference mode
            training_mode = 0;
            
            printf("Inference mode activated. Will use greedy action selection (no exploration).\n");
        }

        else if (strcmp(input, "restore") == 0) {
            restore_training_weights();
        }
        else if (strcmp(input, "manual") == 0 || strcmp(input, "m") == 0) {
            training_mode = 0;
            printf("Manual mode activated. Use 'left', 'right', and 'stop' to control.\n");
        }
        else if (strcmp(input, "left") == 0 || strcmp(input, "l") == 0) {
            send_motor_command(-MOTOR_MAX_SPEED / 2);
        }
        else if (strcmp(input, "right") == 0 || strcmp(input, "r") == 0) {
            send_motor_command(MOTOR_MAX_SPEED / 2);
        }
        else if (strcmp(input, "motor") == 0) {
            printf("Enter motor speed (-255 to 255): ");
            fflush(stdout);
            if (fgets(input, sizeof(input), stdin) != NULL) {
                int speed = atoi(input);
                send_motor_command(speed);
            }
        }
        else if (strcmp(input, "save") == 0) {
            save_weights(WEIGHTS_FILE);
            printf("Weights saved to %s\n", WEIGHTS_FILE);
        }
        else if (strcmp(input, "load") == 0) {
            load_weights(WEIGHTS_FILE);
            printf("Weights loaded from %s\n", WEIGHTS_FILE);
        }
        else if (strcmp(input, "stats") == 0) {
            print_training_stats();
        }
        else if (strcmp(input, "quit") == 0 || strcmp(input, "q") == 0) {
            break;
        }
        else if (strcmp(input, "help") == 0 || strcmp(input, "h") == 0 || strcmp(input, "?") == 0) {
            print_help();
        }
        else if (strlen(input) > 0) {
            printf("Unknown command: %s\n", input);
            print_help();
        }
    }
    
    // Stop system and clean up
    send_command("STOP");
    send_motor_command(0);
    running = 0;
    
    // Wait for threads to finish
    pthread_join(reader_thread_id, NULL);
    pthread_join(control_thread_id, NULL);
    
    // Close serial port
    if (serial_fd >= 0) {
        close(serial_fd);
    }
    
    printf("Program terminated.\n");
    return 0;
}

void print_help() {
    printf("\nAvailable Commands:\n");
    printf("  start (s)     - Start a new episode\n");
    printf("  stop (x)      - Stop the current episode\n");
    printf("  reset (r)     - Reset the system state\n");
    printf("  train (t)     - Enable training mode (automatic control)\n");
    printf("  inference (i) - Switch to inference mode using best weights\n");
    printf("  manual (m)    - Enable manual mode\n");
    printf("  restore       - Restore training weights after inference\n");
    printf("  left (l)      - Move cart left (manual mode)\n");
    printf("  right (r)     - Move cart right (manual mode)\n");
    printf("  motor         - Set specific motor speed\n");
    printf("  save          - Save neural network weights\n");
    printf("  load          - Load neural network weights\n");
    printf("  stats         - Show current statistics\n");
    printf("  help (h, ?)   - Show this help\n");
    printf("  quit (q)      - Quit the program\n\n");
}

int open_serial_port() {
    char serial_path[128] = "/dev/cu.usbmodem1101";  // Your specific port
    struct termios tty;
    
    printf("Attempting to open %s\n", serial_path);
    
    // Open with minimal flags
    serial_fd = open(serial_path, O_RDWR | O_NOCTTY);
    
    if (serial_fd < 0) {
        printf("Error opening serial port: %s\n", strerror(errno));
        return -1;
    }
    
    printf("Port opened with fd: %d\n", serial_fd);

    // Configure serial port
    if (tcgetattr(serial_fd, &tty) != 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        close(serial_fd);
        return -1;
    }
    
    // Set Baud Rate
    cfsetospeed(&tty, BAUD_RATE);
    cfsetispeed(&tty, BAUD_RATE);
    
    // Setting other Port Stuff
    tty.c_cflag &= ~PARENB;     // Make 8n1
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;    // No flow control
    tty.c_cflag |= CREAD | CLOCAL;
    
    // Make raw
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;
    
    // Set Time-outs
    tty.c_cc[VMIN] = 0;         // Read doesn't block
    tty.c_cc[VTIME] = 1;        // 0.1 seconds read timeout
    
    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    
    // Flush any old data
    tcflush(serial_fd, TCIOFLUSH);
    
    // Give Arduino time to reset after connection
    sleep(2);
    
    return 0;
}

void send_command(const char* command) {
    if (serial_fd < 0) return;
    
    char buffer[256];
    snprintf(buffer, sizeof(buffer), "%s\n", command);
    
    write(serial_fd, buffer, strlen(buffer));
    printf("Sent: %s", buffer);
}

void send_motor_command(int speed) {
    if (serial_fd < 0) return;
    
    char buffer[256];
    snprintf(buffer, sizeof(buffer), "MOTOR:%d\n", speed);
    
    write(serial_fd, buffer, strlen(buffer));
    printf("Sent motor command: %d\n", speed);
}

void* serial_reader_thread(void* arg) {
    char buffer[1024];
    int buffer_pos = 0;
    
    // Add this line to confirm the thread starts
    printf("Serial reader thread started\n");
    
    while (running) {
        if (serial_fd < 0) {
            usleep(100000);  // Wait 100ms before trying again
            continue;
        }
        
        // Read incoming data
        char ch;
        int n = read(serial_fd, &ch, 1);
        
        if (n > 0) {
            // printf("Serial reader received data: %c\n", ch);
            // // Print every character received for debugging
            // if (ch >= 32 && ch <= 126) {
            //     printf("%c", ch); // Print readable characters directly
            // } else if (ch == '\n') {
            //     printf("[NL]\n"); // Show newlines
            // } else if (ch == '\r') {
            //     printf("[CR]"); // Show carriage returns
            // } else {
            //     printf("[%02X]", (unsigned char)ch); // Show hex for other bytes
            // }
            // fflush(stdout); // Force immediate output
            // ADD THIS DEBUG BLOCK - End
            
            if (ch == '\n' || ch == '\r') {
                if (buffer_pos > 0) {
                    buffer[buffer_pos] = '\0';  // Null-terminate the string
                    
                    // ADD THIS DEBUG LINE
                    printf("\nDEBUG: Complete line received: '%s'\n", buffer);
                    
                    // Process the completed line
                    if (strncmp(buffer, "STATE:", 6) == 0) {
                        // Parse state data from Arduino
                        float angle, angular_vel, position, velocity;
                        int matches = sscanf(buffer + 6, "%f,%f,%f,%f", 
                                           &angle, &angular_vel, &position, &velocity);
                        
                        // ADD THIS DEBUG LINE
                        printf("DEBUG: Parse result: %d matches\n", matches);
                        
                        if (matches == 4) {
                            pthread_mutex_lock(&state_mutex);
                            state.pole_angle = angle;
                            state.pole_angular_vel = angular_vel;
                            state.cart_position = position;
                            state.cart_velocity = velocity;
                            state.time_steps++;
                            
                            // Check for terminal state
                            state.is_terminal = is_terminal_state();
                            pthread_mutex_unlock(&state_mutex);
                            
                            // ADD THIS DEBUG LINE
                            printf("DEBUG: State updated: angle=%.6f, vel=%.6f, pos=%.6f, cart_vel=%.6f\n", 
                                   angle, angular_vel, position, velocity);
                        }
                    }
                    else if (strncmp(buffer, "TERMINAL", 8) == 0) {
                        pthread_mutex_lock(&state_mutex);
                        state.is_terminal = 1;
                        pthread_mutex_unlock(&state_mutex);
                        printf("Arduino reports terminal state reached\n");
                    }
                    else {
                        // Print other messages from Arduino
                        printf("Arduino: %s\n", buffer);
                    }
                    
                    buffer_pos = 0;  // Reset for next line
                }
            }
            else if (buffer_pos < sizeof(buffer) - 1) {
                buffer[buffer_pos++] = ch;  // Add character to buffer
            }
        }
        else {
            // No data available, sleep a bit
            usleep(10000);  // 10ms
        }
    }
    
    printf("Serial reader thread terminated\n");
    return NULL;
}

void* control_thread(void* arg) {
    printf("DEBUG: In training mode control loop, memory.size=%d\n", memory.size);
    struct timespec last_control_time, current_time;
    clock_gettime(CLOCK_MONOTONIC, &last_control_time);
    
    while (running) {
        // Get current time
        clock_gettime(CLOCK_MONOTONIC, &current_time);
        
        // Calculate elapsed milliseconds
        long elapsed_ms = (current_time.tv_sec - last_control_time.tv_sec) * 1000 +
                         (current_time.tv_nsec - last_control_time.tv_nsec) / 1000000;
        
        // Run control logic at fixed interval
        if (elapsed_ms >= CONTROL_INTERVAL_MS) {
            // Update control time
            last_control_time = current_time;
            
            pthread_mutex_lock(&state_mutex);
            CartPoleState current_state = state;
            pthread_mutex_unlock(&state_mutex);
            
            // Calculate reward for this state
            float reward = calculate_reward();
            current_episode_reward += (int)reward;
            
            // Check if we're in a terminal state
            if (current_state.is_terminal) {
                // Episode ended
                current_episode_steps = current_state.time_steps;
                printf("Episode %d ended after %d steps. Total reward: %d\n", 
                    episode_count, current_episode_steps, current_episode_reward);
                printf("Episode ended due to terminal state...\n");
                // Stop motors
                send_motor_command(0);
                
                // Update best reward and save weights if this is a new best
                if (current_episode_reward > best_reward) {
                    best_reward = current_episode_reward;
                    best_steps = current_episode_steps;
                    save_best_weights();
                    printf("NEW BEST REWARD! Saved weights.\n");
                }

                // PRINT STATS
                print_training_stats();
                
                if (training_mode) {
                    // Process collected experience
                    if (memory.size > 0) {
                        compute_advantages();
                        update_network();
                        printf("Network updated with %d steps of experience\n", memory.size);
                    }
                }
                
                // MOVE THIS CODE OUTSIDE THE TRAINING_MODE CHECK
                // so it runs in both training and inference modes
                printf("\n===== EPISODE ENDED =====\n");
                printf("Type 's' or 'start' when ready to begin next episode\n");
                
                // Just reset state but don't start
                reset_state();
            }
            else if (current_state.time_steps > 0) {
                // If in training mode, use neural network to select actions
                if (training_mode) {
                    // Normalize the state for the neural network
                    float normalized_state[STATE_DIM];
                    normalize_state(normalized_state, &current_state);
                    
                    // Store experience in memory buffer
                    if (memory.size < MAX_STEPS_PER_EPISODE) {
                        // Store current state
                        my_memcpy(memory.states[memory.size], normalized_state, sizeof(float) * STATE_DIM);
                        
                        // Get action probabilities from policy
                        float action_probs[ACTION_DIM];
                        policy_forward(normalized_state, action_probs);
                        
                        // Add some exploration to training
                        int action;
                        float explore_rate = my_max(EXPLORE_RATE, 0.6f * (1.0f - (float)episode_count / 200.0f));
                        if (random_float() < explore_rate) {
                            // Take random action with probability explore_rate
                            action = (random_float() < 0.5f) ? 0 : 1;
                        } else {
                            // Sample from policy distribution
                            action = sample_action(action_probs);
                        }
                        
                        // Store log probability of the chosen action
                        memory.log_probs[memory.size] = log_prob(action_probs, action);
                        
                        // Store action
                        memory.actions[memory.size] = action;
                        
                        // Store value estimate
                        memory.values[memory.size] = value_forward(normalized_state);
                        
                        // Increment memory size after storing experience
                        memory.size++;
                        
                        // Apply the selected action
                        apply_action(action);
                    }
                }
                else {
                    // INFERENCE MODE - Greedy action selection without memory storage
                    float normalized_state[STATE_DIM];
                    normalize_state(normalized_state, &current_state);
                    
                    // Get action probabilities from policy
                    float action_probs[ACTION_DIM];
                    policy_forward(normalized_state, action_probs);
                    
                    // Greedy selection - choose action with highest probability
                    int action = (action_probs[1] > action_probs[0]) ? 1 : 0;
                    
                    // Apply the selected action without storing anything
                    apply_action(action);
                }
            }
        }
        else {
            // Sleep a bit to prevent busy-waiting
            usleep(1000);  // 1ms
        }
    }
    
    printf("Control thread terminated\n");
    return NULL;
}

void init_network() {
    // Kaiming initialization scale factors for better convergence
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
    
    // Hidden-to-hidden layer
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

    // Initialize biases
    for (int i = 0; i < HIDDEN_DIM; i++) {
        // First hidden layer biases
        nn.bias1[i] = 0.01f * (random_float() - 0.5f);
        nn.value_bias1[i] = 0.01f * (random_float() - 0.5f);
        
        // Second hidden layer biases
        nn.bias_h2[i] = 0.01f * (random_float() - 0.5f);
        nn.value_bias_h2[i] = 0.01f * (random_float() - 0.5f);
    }

    // Bias the policy network to make reasonable initial decisions
    nn.bias2[0] = -0.1f;  // Slight bias toward left
    nn.bias2[1] = 0.1f;   // Slight bias toward right

    // Initialize value function with small positive bias (optimistic initialization)
    nn.value_bias2[0] = 0.1f;
    
    // Test the network with a pole tilted right
    float test_state[STATE_DIM] = {0, 0, 0.1f, 0}; // Slightly tilted right
    float probs[ACTION_DIM];
    policy_forward(test_state, probs);

    // Ensure the initial policy responds correctly to a tilted pole
    if (probs[1] < 0.55f) {  // Should favor moving right for a right-tilted pole
        nn.bias2[0] = -0.2f;
        nn.bias2[1] = 0.2f;
    }
}

void normalize_state(float normalized_state[STATE_DIM], CartPoleState* cart_state) {
    // Position: [-2.4, 2.4] -> [-1, 1]
    normalized_state[0] = cart_state->cart_position / 10.0f;
    
    // Velocity: clip and normalize to [-1, 1]
    normalized_state[1] = cart_state->cart_velocity;
    if (normalized_state[1] > 10.0f) normalized_state[1] = 10.0f;
    if (normalized_state[1] < -10.0f) normalized_state[1] = -10.0f;
    normalized_state[1] /= 10.0f;
    
    // Angle: [-MAX_ANGLE_RAD, MAX_ANGLE_RAD] -> [-1, 1]
    normalized_state[2] = cart_state->pole_angle / MAX_ANGLE_RAD;
    
    // Angular velocity: clip and normalize to [-1, 1]
    normalized_state[3] = cart_state->pole_angular_vel;
    if (normalized_state[3] > MAX_ANGULAR_VELOCITY) normalized_state[3] = MAX_ANGULAR_VELOCITY;
    if (normalized_state[3] < -MAX_ANGULAR_VELOCITY) normalized_state[3] = -MAX_ANGULAR_VELOCITY;
    normalized_state[3] /= MAX_ANGULAR_VELOCITY;
    
    // Clamp all values to prevent extreme inputs
    for (int i = 0; i < STATE_DIM; i++) {
        normalized_state[i] = my_clamp(normalized_state[i], -1.0f, 1.0f);
    }
}

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
        hidden1[i] = leaky_relu(hidden1[i]);  // Leaky ReLU
    }

    // First hidden to second hidden layer
    for (int i = 0; i < HIDDEN_DIM; i++) {
        hidden2[i] = 0.0f;
        for (int j = 0; j < HIDDEN_DIM; j++) {
            hidden2[i] += hidden1[j] * nn.weights_h2h[j][i];
        }
        hidden2[i] += nn.bias_h2[i];
        hidden2[i] = leaky_relu(hidden2[i]);  // Leaky ReLU
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

int sample_action(float probs[ACTION_DIM]) {
    // Sample action based on probabilities
    float r = random_float();
    if (r < probs[0]) {
        return 0;  // Left
    } else {
        return 1;  // Right
    }
}

// int apply_action(int action) {
//     // Convert action to motor command
//     int motor_speed;
    
//     if (action == 0) {  // Left
//         motor_speed = -FORCE_MAG;
//     } else {  // Right
//         motor_speed = FORCE_MAG;
//     }
    
//     // Scale to motor range
//     motor_speed = (motor_speed * MOTOR_MAX_SPEED) / 255;
    
//     // Send command to Arduino
//     send_motor_command(motor_speed);
    
//     return motor_speed;
// }
int apply_action(int action) {
    struct timespec current_time_spec;
    clock_gettime(CLOCK_MONOTONIC, &current_time_spec);
    unsigned long current_time = current_time_spec.tv_sec * 1000 + current_time_spec.tv_nsec / 1000000;
    
    // Only allow new actions after minimum duration
    if (current_time - last_action_time < MIN_ACTION_DURATION) {
        return 0; // Return without changing motors
    }
  
  last_action_time = current_time;
    
    // Rest of your code...
    int motor_speed;
    if (action == 0) {  // Left
      motor_speed = -FORCE_MAG;
    } else {  // Right
      motor_speed = FORCE_MAG;
    }
    
    // Send command to Arduino
    send_motor_command(motor_speed);
    
    return motor_speed;
}

void reset_state() {
    // Reset system state
    send_command("RESET");
    
    // Reset local state tracking
    pthread_mutex_lock(&state_mutex);
    state.pole_angle = 0.0f;
    state.pole_angular_vel = 0.0f;
    state.cart_position = 0.0f;
    state.cart_velocity = 0.0f;
    state.time_steps = 0;
    state.is_terminal = 0;
    pthread_mutex_unlock(&state_mutex);
    
    // Reset episode stats
    current_episode_steps = 0;
    current_episode_reward = 0;
}

void start_episode() {
    // Increment episode counter
    episode_count++;
    
    // Reset state before starting new episode
    reset_state();
    
    // Start the episode
    send_command("START");
    printf("Episode %d started\n", episode_count);
}

void stop_episode() {
    // Stop the current episode and motors
    send_command("STOP");
    send_motor_command(0);
    
    // Print episode stats
    printf("Episode %d manually stopped after %d steps. Total reward: %d\n", 
           episode_count, current_episode_steps, current_episode_reward);
}

float calculate_reward() {
    // Calculate reward based on current state
    pthread_mutex_lock(&state_mutex);
    float angle = state.pole_angle;
    float position = state.cart_position;
    pthread_mutex_unlock(&state_mutex);
    
    // Reward for keeping pole upright
    float angle_reward = 1.0f - (fabs(angle) / MAX_ANGLE_RAD);
    angle_reward = angle_reward * angle_reward * angle_reward;  // Cubic reward
    
    // Reward for keeping cart near center
    float position_reward = 1.0f - (fabs(position) / 10.0f);
    position_reward = position_reward * position_reward;  // Squared reward
    
    // Combine rewards (emphasize keeping pole upright)
    return 15.0f * angle_reward + 5.0f * position_reward + 0.1f;  // Small bonus for surviving
}

void load_weights(const char* filename) {
    FILE* fp = fopen(filename, "rb");
    if (fp) {
        fread(&nn, sizeof(NeuralNetwork), 1, fp);
        fclose(fp);
    } else {
        printf("Error opening weights file for reading: %s\n", filename);
    }
}

void save_weights(const char* filename) {
    FILE* fp = fopen(filename, "wb");
    if (fp) {
        fwrite(&nn, sizeof(NeuralNetwork), 1, fp);
        fclose(fp);
    } else {
        printf("Error opening weights file for writing: %s\n", filename);
    }
}

float random_float() {
    return (float)rand() / RAND_MAX;
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
    float current_lr = LEARNING_RATE * (1.0f - (float)episode_count / MAX_EPISODES);
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
        int indices[MAX_STEPS_PER_EPISODE];
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

// Proper min function with typed parameters
int min(int x, int y) {
    return (x < y) ? x : y;
}

// check if the state is terminal (pole fallen or cart out of bounds)
int is_terminal_state() {
    printf("Terminal check: angle=%f, position=%f\n", state.pole_angle, state.cart_position);
    // Terminal if pole angle exceeds max (MAX_ANGLE_RAD radians)
    if (my_abs(state.pole_angle) > MAX_ANGLE_RAD) return 1;
    
    // Terminal if cart position exceeds MAX_POSITION inches
    if (my_abs(state.cart_position) > MAX_POSITION) return 1;
    
    // Additional termination condition - if angular velocity is too high
    if (my_abs(state.pole_angular_vel) > MAX_ANGULAR_VELOCITY) return 1;

    return 0;
}

void save_best_weights() {
    my_memcpy(&best_nn, &nn, sizeof(NeuralNetwork));
    best_epoch = episode_count;
    best_steps = current_episode_steps;
    best_nn.reward = current_episode_reward;  // Store the reward
    
    // Visual feedback
    // *LEDR_ptr = 0x3FF;  // Turn on all LEDs briefly
    printf("Saved best weights from epoch %d (reward: %d, steps: %d)\n", 
           best_epoch, current_episode_reward, best_steps);
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

// Function to print training statistics
void print_training_stats() {
    char buffer[30]; // Declaring but using printf directly (buffer unused)
    
    // Print a header for clarity
    printf("\n===== CARTRL TRAINING STATISTICS =====\n");
    
    // Print basic episode stats
    printf("Episode: %d\n", episode_count);
    printf("Current Step: %d\n", current_episode_steps);
    printf("Current Reward: %d\n", current_episode_reward);
    printf("Best Reward: %d\n", best_reward);
    
    // Print memory buffer usage
    printf("Memory Buffer: %d/%d steps\n", memory.size, MAX_STEPS_PER_EPISODE);
    
    // Print learning parameters
    float current_lr = LEARNING_RATE * (1.0f - (float)episode_count / MAX_EPISODES);
    if (current_lr < LEARNING_RATE * 0.1f) current_lr = LEARNING_RATE * 0.1f;
    printf("Current Learning Rate: %.6f\n", current_lr);
    printf("Mode: %s\n", training_mode ? "TRAINING" : "INFERENCE");
    
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

// Helper function to clamp values
float my_clamp(float value, float min, float max) {
    return value < min ? min : (value > max ? max : value);
}

// Random integer generator [min, max]
int random_int(int min, int max) {
    int range = max - min + 1;
    return min + (int)(random_float() * range);
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
