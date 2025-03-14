#include <stdio.h>
#include <stdlib.h>
#include <time.h>

void multiply_matrices(float* A, float* B, float* C, int N) {
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            C[i * N + j] = 0.0f;
            for (int k = 0; k < N; k++) {
                C[i * N + j] += A[i * N + k] * B[k * N + j];
            }
        }
    }
}

void initialize_matrix(float* matrix, int N) {
    for (int i = 0; i < N * N; i++) {
        matrix[i] = (float)(rand() % 10);  // Random numbers between 0 and 9
    }
}

int main() {
    int N = 100; // Matrix size (N x N)

    // Declare matrices on the stack
    float A[N * N], B[N * N], C[N * N];
    
    // Initialize matrices with random values
    srand(time(NULL)); // Initialize random number generator
    initialize_matrix(A, N);
    initialize_matrix(B, N);

    // Benchmark matrix multiplication
    clock_t start = clock();
    multiply_matrices(A, B, C, N);
    clock_t end = clock();

    float time_taken = ((float)(end - start)) / CLOCKS_PER_SEC;
    printf("Matrix multiplication of %dx%d matrices took %.4f seconds.\n", N, N, time_taken);

    return 0;
}
