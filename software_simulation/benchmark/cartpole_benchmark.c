#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <string.h>

// Simulation parameters
#define GRAVITY 9.8f
#define CART_MASS 1.0f
#define POLE_MASS 0.2f
#define POLE_HALF_LENGTH 0.75f
#define FORCE_MAG 20.0f
#define TIME_STEP 0.02f
#define MAX_ANGLE_RAD 0.55f
#define MAX_ANGULAR_VELOCITY 4.0f

// PPO parameters
#define STATE_DIM 4
#define ACTION_DIM 2


#define HIDDEN_DIM 64
#define MAX_EPISODES 151
#define MAX_STEPS_PER_EPOCH 400
#define PPO_EPOCHS 12
#define CLIP_EPSILON 0.2f
#define LAMBDA 0.95f

// Training configuration
#define REWARD_THRESHOLD 2100
#define WEIGHTS_FILE_PREFIX "best_weights"

typedef struct {
    float cart_position;
    float cart_velocity;
    float pole_angle;
    float pole_angular_vel;
} CartPoleState;

typedef struct {
    float weights1[STATE_DIM][HIDDEN_DIM];
    float bias1[HIDDEN_DIM];
    float weights_h2h[HIDDEN_DIM][HIDDEN_DIM];
    float bias_h2[HIDDEN_DIM];
    float weights2[HIDDEN_DIM][ACTION_DIM];
    float bias2[ACTION_DIM];
    
    float value_weights1[STATE_DIM][HIDDEN_DIM];
    float value_bias1[HIDDEN_DIM];
    float value_weights_h2h[HIDDEN_DIM][HIDDEN_DIM];
    float value_bias_h2[HIDDEN_DIM];
    float value_weights2[HIDDEN_DIM][1];
    float value_bias2[1];
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
    int dones[MAX_STEPS_PER_EPOCH];
    int size;
} Memory;

// Global variables
CartPoleState state;
NeuralNetwork nn;
Memory memory;
int current_mode = 0;  // 0 = train, 1 = inference


// Hyperparameter search space
#define NUM_LEARNING_RATES 5
#define NUM_GAMMAS 1
#define NUM_EXPLORE_RATES 4

// Hyperparameter sets
float learning_rates[NUM_LEARNING_RATES] = {
    0.02f,   // Very high LR (risky but may find fast learners)
    0.0005f,  // Very low LR for stable learning
    0.001f,   // Low LR
    0.005f,   // Moderate LR (common baseline)
    0.01f    // High LR
};

float gammas[NUM_GAMMAS] = {
    0.95f    // Moderate-term focus (common baseline)
};

float explore_rates[NUM_EXPLORE_RATES] = {
    0.1f,     // Low exploration
    0.2f,     // Moderate exploration
    0.3f,     // High exploration
    0.4f      // Very high exploration
};

// Function prototypes
void init_network();
void update_physics(int action);
float calculate_reward();
int is_terminal_state();
void normalize_state(float normalized_state[STATE_DIM], CartPoleState *cart_state);
int sample_action(float probs[ACTION_DIM]);
void policy_forward(float state[STATE_DIM], float probs[ACTION_DIM]);
float value_forward(float state[STATE_DIM]);
void compute_advantages(float gamma, float lambda);
void update_network(float learning_rate, float gamma);
void save_weights(const char* filename);
void load_weights(const char* filename);
void run_episode(float explore_rate, float gamma, int is_training, int *total_reward);
float random_float();
int random_int(int min, int max);
float my_abs(float x);
float my_clamp(float value, float min, float max);

int main() {
    srand(time(NULL));
    // Test all hyperparameter combinations
    for (int lr_idx = 0; lr_idx < NUM_LEARNING_RATES; lr_idx++) {
        for (int gamma_idx = 0; gamma_idx < NUM_GAMMAS; gamma_idx++) {
            for (int explore_idx = 0; explore_idx < NUM_EXPLORE_RATES; explore_idx++) {
                float lr = learning_rates[lr_idx];
                float gamma = gammas[gamma_idx];
                float explore_rate = explore_rates[explore_idx];
                
                printf("\nTesting hyperparameters: LR=%.4f, Gamma=%.2f, Explore=%.2f\n",
                       lr, gamma, explore_rate);
                
                init_network();
                int best_reward = 0;

                for (int episode = 0; episode < MAX_EPISODES; episode++) {
                    int train_reward, inference_reward;
                    
                    // Training episode
                    current_mode = 0;
                    run_episode(explore_rate, gamma, 1, &train_reward);
                    
                    // Inference episode
                    current_mode = 1;
                    run_episode(0.0f, gamma, 0, &inference_reward);

                    // Update best reward and save weights if threshold crossed
                    if (inference_reward > best_reward) {
                        best_reward = inference_reward;
                        if (best_reward >= REWARD_THRESHOLD) {
                            char filename[256];
                            snprintf(filename, sizeof(filename), "%s_lr%.4f_gamma%.2f_exp%.2f_ep%d.bin",
                                   WEIGHTS_FILE_PREFIX, lr, gamma, explore_rate, episode);
                            save_weights(filename);
                            printf("Saved weights: %s\n", filename);
                        }
                    }

                    // Print progress
                    if (episode % 50 == 0 || inference_reward >= REWARD_THRESHOLD) {
                        printf("Episode %4d: Train Reward=%4d, Inference Reward=%4d (Best=%4d)\n",
                               episode, train_reward, inference_reward, best_reward);
                    }
                }
            }
        }
    }
    
    return 0;
}

void run_episode(float explore_rate, float gamma, int is_training, int *total_reward) {
    // Initialize state
    state.cart_position = random_float() * 0.2f - 0.1f;
    state.cart_velocity = random_float() * 0.2f - 0.1f;
    state.pole_angle = random_float() * 0.2f - 0.1f;
    state.pole_angular_vel = random_float() * 0.2f - 0.1f;
    
    memory.size = 0;
    *total_reward = 0;

    for (int step = 0; step < MAX_STEPS_PER_EPOCH; step++) {
        float normalized_state[STATE_DIM];
        normalize_state(normalized_state, &state);

        float action_probs[ACTION_DIM];
        policy_forward(normalized_state, action_probs);
        
        int action;
        if (is_training && (random_float() < explore_rate)) {
            action = random_int(0, ACTION_DIM - 1);
        } else {
            action = sample_action(action_probs);
        }

        // Store experience for training
        if (is_training && memory.size < MAX_STEPS_PER_EPOCH) {
            memcpy(memory.states[memory.size], normalized_state, sizeof(float)*STATE_DIM);
            memory.actions[memory.size] = action;
            memory.log_probs[memory.size] = log(action_probs[action]);
            memory.values[memory.size] = value_forward(normalized_state);
            memory.size++;
        }

        // Update physics
        update_physics(action);
        
        // Calculate reward
        float reward = calculate_reward();
        *total_reward += (int)reward;

        if (is_training && memory.size > 0) {
            memory.rewards[memory.size-1] = reward;
            memcpy(memory.next_states[memory.size-1], normalized_state, sizeof(float)*STATE_DIM);
            memory.dones[memory.size-1] = is_terminal_state();
        }

        if (is_terminal_state()) break;
    }

    if (is_training) {
        compute_advantages(gamma, LAMBDA);
        update_network(learning_rates[0], gamma);
    }
}


void update_physics(int action) {
    float force = (action == 0) ? -FORCE_MAG : FORCE_MAG;
    float total_mass = CART_MASS + POLE_MASS;
    float pole_mass_length = POLE_MASS * POLE_HALF_LENGTH;
    
    float cos_theta = cos(state.pole_angle);
    float sin_theta = sin(state.pole_angle);
    
    float temp = (force + pole_mass_length * state.pole_angular_vel * state.pole_angular_vel * sin_theta) / total_mass;
    float angular_accel = (GRAVITY * sin_theta - cos_theta * temp) / 
                         (POLE_HALF_LENGTH * (4.0f/3.0f - POLE_MASS * cos_theta * cos_theta / total_mass));
    float linear_accel = temp - pole_mass_length * angular_accel * cos_theta / total_mass;
    
    // Update state using Euler integration
    state.cart_position += TIME_STEP * state.cart_velocity;
    state.cart_velocity += TIME_STEP * linear_accel;
    state.pole_angle += TIME_STEP * state.pole_angular_vel;
    state.pole_angular_vel += TIME_STEP * angular_accel;
    
    // Normalize angle
    while (state.pole_angle > M_PI) state.pole_angle -= 2.0f * M_PI;
    while (state.pole_angle < -M_PI) state.pole_angle += 2.0f * M_PI;
}


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// float calculate_reward() {
//     if (is_terminal_state()) return -20.0f;
    
//     float angle_reward = 1.0f - fabs(state.pole_angle) / MAX_ANGLE_RAD;
//     angle_reward = angle_reward * angle_reward * angle_reward;
    
//     float position_reward = 1.0f - fabs(state.cart_position) / 2.0f;
//     position_reward = position_reward * position_reward * position_reward;
    
//     float vel_penalty = -0.05f * fabs(state.cart_velocity);
//     float ang_vel_penalty = -0.1f * fabs(state.pole_angular_vel);
    
//     float center_bonus = 0.0f;
//     if (fabs(state.cart_position) < 0.5f) {
//         center_bonus = 0.5f * (1.0f - fabs(state.cart_position) / 0.5f);
//     }
    
//     return 15.0f * angle_reward + 10.0f * position_reward +  center_bonus + vel_penalty + ang_vel_penalty + 0.1f;
// }

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

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


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// float calculate_reward() {
//     static float max_angle_seen = 0.0f;
//     static float max_position_seen = 0.0f;
    
//     if (is_terminal_state()) return -30.0f;
    
//     // Update max observed values
//     max_angle_seen = fmax(max_angle_seen, fabs(state.pole_angle));
//     max_position_seen = fmax(max_position_seen, fabs(state.cart_position));
    
//     // Normalize based on observed range
//     float angle_reward = 1.0f - (fabs(state.pole_angle) / max_angle_seen);
//     float position_reward = 1.0f - (fabs(state.cart_position) / max_position_seen);
    
//     return 15.0f * angle_reward + 10.0f * position_reward;
// }

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


// float calculate_reward() {
//     if (is_terminal_state()) return -50.0f;
    
//     // Primary objective: pole angle
//     float angle_obj = 1.0f - pow(fabs(state.pole_angle) / MAX_ANGLE_RAD, 2);
    
//     // Secondary objective: position
//     float position_obj = exp(-0.5f * state.cart_position * state.cart_position);
    
//     // Tertiary objective: smoothness
//     float smoothness_obj = exp(-0.1f * (fabs(state.cart_velocity) + 
//                               fabs(state.pole_angular_vel)));
    
//     return 20.0f * angle_obj + 10.0f * position_obj + 5.0f * smoothness_obj;
// }

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -






int is_terminal_state() {
    return (fabs(state.pole_angle) > MAX_ANGLE_RAD) || 
           (fabs(state.cart_position) > 2.0f) ||
           (fabs(state.pole_angular_vel) > MAX_ANGULAR_VELOCITY);
}

void normalize_state(float normalized_state[STATE_DIM], CartPoleState *cart_state) {
    normalized_state[0] = cart_state->cart_position / 2.4f;
    normalized_state[1] = my_clamp(cart_state->cart_velocity / 10.0f, -1.0f, 1.0f);
    normalized_state[2] = cart_state->pole_angle / MAX_ANGLE_RAD;
    normalized_state[3] = my_clamp(cart_state->pole_angular_vel / 10.0f, -1.0f, 1.0f);
}

int sample_action(float probs[ACTION_DIM]) {
    float r = random_float();
    return (r < probs[0]) ? 0 : 1;
}

void policy_forward(float state[STATE_DIM], float probs[ACTION_DIM]) {
    float hidden1[HIDDEN_DIM], hidden2[HIDDEN_DIM];
    
    // First hidden layer
    for (int i = 0; i < HIDDEN_DIM; i++) {
        hidden1[i] = nn.bias1[i];
        for (int j = 0; j < STATE_DIM; j++) {
            hidden1[i] += state[j] * nn.weights1[j][i];
        }
        hidden1[i] = fmax(0.01f * hidden1[i], hidden1[i]); // Leaky ReLU
    }
    
    // Second hidden layer
    for (int i = 0; i < HIDDEN_DIM; i++) {
        hidden2[i] = nn.bias_h2[i];
        for (int j = 0; j < HIDDEN_DIM; j++) {
            hidden2[i] += hidden1[j] * nn.weights_h2h[j][i];
        }
        hidden2[i] = fmax(0.01f * hidden2[i], hidden2[i]); // Leaky ReLU
    }
    
    // Output layer
    float output[ACTION_DIM] = {0};
    for (int i = 0; i < ACTION_DIM; i++) {
        output[i] = nn.bias2[i];
        for (int j = 0; j < HIDDEN_DIM; j++) {
            output[i] += hidden2[j] * nn.weights2[j][i];
        }
    }
    
    // Softmax
    float max_output = fmax(output[0], output[1]);
    float sum_exp = exp(output[0] - max_output) + exp(output[1] - max_output);
    probs[0] = exp(output[0] - max_output) / sum_exp;
    probs[1] = exp(output[1] - max_output) / sum_exp;
}

float value_forward(float state[STATE_DIM]) {
    float hidden1[HIDDEN_DIM], hidden2[HIDDEN_DIM];
    
    // First hidden layer
    for (int i = 0; i < HIDDEN_DIM; i++) {
        hidden1[i] = nn.value_bias1[i];
        for (int j = 0; j < STATE_DIM; j++) {
            hidden1[i] += state[j] * nn.value_weights1[j][i];
        }
        hidden1[i] = fmax(0.01f * hidden1[i], hidden1[i]); // Leaky ReLU
    }
    
    // Second hidden layer
    for (int i = 0; i < HIDDEN_DIM; i++) {
        hidden2[i] = nn.value_bias_h2[i];
        for (int j = 0; j < HIDDEN_DIM; j++) {
            hidden2[i] += hidden1[j] * nn.value_weights_h2h[j][i];
        }
        hidden2[i] = fmax(0.01f * hidden2[i], hidden2[i]); // Leaky ReLU
    }
    
    // Output value
    float value = nn.value_bias2[0];
    for (int j = 0; j < HIDDEN_DIM; j++) {
        value += hidden2[j] * nn.value_weights2[j][0];
    }
    return value;
}

void compute_advantages(float gamma, float lambda) {
    // Calculate returns
    for (int t = memory.size - 1; t >= 0; t--) {
        float next_value = memory.dones[t] ? 0 : value_forward(memory.next_states[t]);
        memory.returns[t] = memory.rewards[t] + gamma * next_value;
    }

    // Calculate GAE advantages
    float gae = 0;
    for (int t = memory.size - 1; t >= 0; t--) {
        float delta = memory.rewards[t] + gamma * value_forward(memory.next_states[t]) - memory.values[t];
        gae = delta + gamma * lambda * gae;
        memory.advantages[t] = gae;
    }

    // Normalize advantages
    float mean = 0, variance = 0;
    for (int t = 0; t < memory.size; t++) mean += memory.advantages[t];
    mean /= memory.size;
    for (int t = 0; t < memory.size; t++) {
        float diff = memory.advantages[t] - mean;
        variance += diff * diff;
    }
    variance = sqrt(variance / memory.size);
    if (variance < 1e-8) variance = 1e-8;
    for (int t = 0; t < memory.size; t++) {
        memory.advantages[t] = (memory.advantages[t] - mean) / variance;
    }
}

void update_network(float learning_rate, float gamma) {
    const int batch_size = 64;
    const int num_batches = memory.size / batch_size;
    if (num_batches == 0) return;

    for (int epoch = 0; epoch < PPO_EPOCHS; epoch++) {
        // Shuffle indices
        int indices[MAX_STEPS_PER_EPOCH];
        for (int i = 0; i < memory.size; i++) indices[i] = i;
        for (int i = memory.size - 1; i > 0; i--) {
            int j = random_int(0, i);
            int temp = indices[i];
            indices[i] = indices[j];
            indices[j] = temp;
        }

        for (int batch = 0; batch < num_batches; batch++) {
            int start = batch * batch_size;
            int end = (batch + 1) * batch_size;
            end = end > memory.size ? memory.size : end;

            // Policy update
            for (int i = start; i < end; i++) {
                int t = indices[i];
                float* state = memory.states[t];
                int action = memory.actions[t];
                float advantage = memory.advantages[t];
                float old_log_prob = memory.log_probs[t];

                float current_probs[ACTION_DIM];
                policy_forward(state, current_probs);
                float current_log_prob = log(current_probs[action]);

                float ratio = exp(current_log_prob - old_log_prob);
                float clipped_ratio = my_clamp(ratio, 1.0f - CLIP_EPSILON, 1.0f + CLIP_EPSILON);
                float policy_loss = -fmin(ratio * advantage, clipped_ratio * advantage);

                // Simplified gradient update (in practice would backpropagate)
                float update_scale = learning_rate * policy_loss / batch_size;
                for (int j = 0; j < HIDDEN_DIM; j++) {
                    nn.weights2[j][action] -= update_scale * 0.01f;
                }
            }

            // Value update
            for (int i = start; i < end; i++) {
                int t = indices[i];
                float* state = memory.states[t];
                float target = memory.returns[t];
                float predicted = value_forward(state);
                float value_loss = 0.5f * (predicted - target) * (predicted - target);

                // Simplified gradient update
                float update_scale = learning_rate * (predicted - target) / batch_size;
                for (int j = 0; j < HIDDEN_DIM; j++) {
                    nn.value_weights2[j][0] -= update_scale * 0.01f;
                }
            }
        }
    }
}

void init_network() {
    float scale = sqrt(2.0f / (STATE_DIM + HIDDEN_DIM));
    
    // Initialize policy network
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < HIDDEN_DIM; j++) {
            nn.weights1[i][j] = scale * (random_float() - 0.5f);
        }
    }
    for (int i = 0; i < HIDDEN_DIM; i++) {
        nn.bias1[i] = 0.01f * (random_float() - 0.5f);
        for (int j = 0; j < HIDDEN_DIM; j++) {
            nn.weights_h2h[i][j] = scale * (random_float() - 0.5f);
        }
    }
    for (int i = 0; i < HIDDEN_DIM; i++) {
        nn.bias_h2[i] = 0.01f * (random_float() - 0.5f);
        for (int j = 0; j < ACTION_DIM; j++) {
            nn.weights2[i][j] = scale * (random_float() - 0.5f);
        }
    }
    nn.bias2[0] = -0.1f;
    nn.bias2[1] = 0.1f;
    
    // Initialize value network similarly...
}

void save_weights(const char* filename) {
    FILE* fp = fopen(filename, "wb");
    if (fp) {
        fwrite(&nn, sizeof(NeuralNetwork), 1, fp);
        fclose(fp);
    }
}

void load_weights(const char* filename) {
    FILE* fp = fopen(filename, "rb");
    if (fp) {
        fread(&nn, sizeof(NeuralNetwork), 1, fp);
        fclose(fp);
    }
}

float random_float() {
    return (float)rand() / RAND_MAX;
}

int random_int(int min, int max) {
    return min + rand() % (max - min + 1);
}


float my_abs(float x) {
    return x < 0 ? -x : x;
}

float my_clamp(float value, float min, float max) {
    return value < min ? min : (value > max ? max : value);
}