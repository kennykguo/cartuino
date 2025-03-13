#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>

// Hyperparameters
#define STATE_DIM 4
#define ACTION_DIM 2
#define HIDDEN_DIM 32
#define BATCH_SIZE 128
#define MAX_TIMESTEPS 500

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

NeuralNetwork nn;

// Forward declarations
float relu(float x);
void policy_forward(float state[STATE_DIM], float probs[ACTION_DIM]);
float value_forward(float state[STATE_DIM]);
void init_network();
void benchmark_inference();

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

// Initialize the neural network with small random weights
void init_network() {
    int i, j;
    
    // Get current time for random seed
    srand(time(NULL));

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

// Benchmarking Inference
void benchmark_inference() {
    int num_runs = 1000;  // Number of times to run inference
    clock_t start, end;
    float total_time = 0.0f;

    // Random state for testing
    float state[STATE_DIM] = {0.5f, -0.2f, 0.1f, 0.4f};
    float probs[ACTION_DIM];
    
    // Benchmark the policy network
    start = clock();
    for (int i = 0; i < num_runs; i++) {
        policy_forward(state, probs);
    }
    end = clock();
    total_time = ((float)(end - start)) / CLOCKS_PER_SEC;
    printf("Policy network inference time: %.6f seconds for %d runs\n", total_time, num_runs);
    
    // Benchmark the value network
    start = clock();
    for (int i = 0; i < num_runs; i++) {
        value_forward(state);
    }
    end = clock();
    total_time = ((float)(end - start)) / CLOCKS_PER_SEC;
    printf("Value network inference time: %.6f seconds for %d runs\n", total_time, num_runs);
}

int main() {
    // Initialize the neural network with random weights
    init_network();

    // Run the benchmark for inference
    benchmark_inference();

    return 0;
}
