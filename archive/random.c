#define PIXEL_BUF_CTRL_BASE 0xFF203020
#define KEY_BASE            0xFF200050
#define LEDR_BASE           0xFF200000
#define HEX3_HEX0_BASE      0xFF200020
#define TIMER_BASE          0xFF202000 

#include <stdlib.h>
#include <math.h>
#include <stddef.h>

#define VGA_WIDTH 320
#define VGA_HEIGHT 240
#define LINE_COLOUR 0x1E3D

// cart-pole simulation parameters
#define GRAVITY 9.8f
#define CART_MASS 1.0f
#define POLE_MASS 0.1f
#define POLE_HALF_LENGTH 0.5f
#define FORCE_MAG 10.0f
#define TIME_STEP 0.02f
#define RANDOM_FORCE_MAX 1.0f
#define PIXEL_SCALE 50.0f

// rl parameters
#define MAX_EPOCHS 1000
#define MAX_STEPS_PER_EPOCH 500

// display parameters
#define CART_WIDTH 40
#define CART_HEIGHT 20
#define POLE_WIDTH 6
#define POLE_LENGTH 100

// color definitions (16-bit RGB565 format)
#define BLACK 0x0000
#define WHITE 0xFFFF
#define RED 0xF800
#define GREEN 0x07E0
#define BLUE 0x001F
#define YELLOW 0xFFE0
#define CYAN 0x07FF
#define MAGENTA 0xF81F

// state variables for cart-pole system
typedef struct {
    float cart_position;
    float cart_velocity;
    float pole_angle;
    float pole_angular_vel;
} CartPoleState;

// performance tracking
typedef struct {
    int epoch;
    int step;
    int total_reward;
    int best_reward;
} PerformanceStats;

// global variables
volatile int pixel_buffer_start;
short int Buffer1[240][512]; 
short int Buffer2[240][512];
volatile int *KEY_ptr;
volatile int *LEDR_ptr;
volatile unsigned int *HEX3_HEX0_ptr;

// global simulation state
CartPoleState state;
CartPoleState prev_state;
PerformanceStats stats;
int simulation_running = 0;
unsigned int random_seed;

// function prototypes
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
unsigned int xorshift32();
void draw_char(int x, int y, char c, short int color);
void draw_text(int x, int y, char* str, short int color);
void int_to_str(int num, char* str);

// draw a single character (for simple text display)
void draw_char(int x, int y, char c, short int color) {
    // define font data for digits 0-9
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
    
    // letters and special characters
    const unsigned char font_A[5] = {0x7E, 0x11, 0x11, 0x11, 0x7E}; // A
    const unsigned char font_B[5] = {0x7F, 0x49, 0x49, 0x49, 0x36}; // B
    const unsigned char font_C[5] = {0x3E, 0x41, 0x41, 0x41, 0x22}; // C
    const unsigned char font_D[5] = {0x7F, 0x41, 0x41, 0x22, 0x1C}; // D
    const unsigned char font_E[5] = {0x7F, 0x49, 0x49, 0x49, 0x41}; // E
    const unsigned char font_G[5] = {0x3E, 0x41, 0x49, 0x49, 0x7A}; // G
    const unsigned char font_H[5] = {0x7F, 0x08, 0x08, 0x08, 0x7F}; // H
    const unsigned char font_K[5] = {0x7F, 0x08, 0x14, 0x22, 0x41}; // K
    const unsigned char font_N[5] = {0x7F, 0x04, 0x08, 0x10, 0x7F}; // N
    const unsigned char font_O[5] = {0x3E, 0x41, 0x41, 0x41, 0x3E}; // O
    const unsigned char font_P[5] = {0x7F, 0x09, 0x09, 0x09, 0x06}; // P
    const unsigned char font_R[5] = {0x7F, 0x09, 0x19, 0x29, 0x46}; // R
    const unsigned char font_S[5] = {0x46, 0x49, 0x49, 0x49, 0x31}; // S
    const unsigned char font_T[5] = {0x01, 0x01, 0x7F, 0x01, 0x01}; // T
    const unsigned char font_W[5] = {0x3F, 0x40, 0x38, 0x40, 0x3F}; // W
    const unsigned char font_Y[5] = {0x07, 0x08, 0x70, 0x08, 0x07}; // Y
    const unsigned char font_colon[5] = {0x00, 0x36, 0x36, 0x00, 0x00}; // :
    const unsigned char font_dash[5] = {0x08, 0x08, 0x08, 0x08, 0x08}; // -
    
    const unsigned char* font_ptr = NULL;
    
    // select font data based on character
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
    else if (c == 'G' || c == 'g') font_ptr = font_G;
    else if (c == 'H' || c == 'h') font_ptr = font_H;
    else if (c == 'K' || c == 'k') font_ptr = font_K;
    else if (c == 'N' || c == 'n') font_ptr = font_N;
    else if (c == 'O' || c == 'o') font_ptr = font_O;
    else if (c == 'P' || c == 'p') font_ptr = font_P;
    else if (c == 'R' || c == 'r') font_ptr = font_R;
    else if (c == 'S' || c == 's') font_ptr = font_S;
    else if (c == 'T' || c == 't') font_ptr = font_T;
    else if (c == 'W' || c == 'w') font_ptr = font_W;
    else if (c == 'Y' || c == 'y') font_ptr = font_Y;
    else if (c == ':') font_ptr = font_colon;
    else if (c == '-') font_ptr = font_dash;
    
    // draw the character
    if (font_ptr != NULL) {
        for (int row = 0; row < 8; row++) {
            for (int col = 0; col < 5; col++) {
                if (font_ptr[col] & (1 << row)) {
                    plot_pixel(x + col, y + row, color);
                }
            }
        }
    } else {
        // for unsupported characters, draw a small rectangle
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

// integer to string conversion
void int_to_str(int num, char* str) {
    // handle special case for 0
    if (num == 0) {
        str[0] = '0';
        str[1] = '\0';
        return;
    }
    
    // handle negative numbers
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
    
    // reverse the string
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

int main(void) {
    volatile int * pixel_ctrl_ptr = (int *)PIXEL_BUF_CTRL_BASE;
    KEY_ptr = (int *)KEY_BASE;
    LEDR_ptr = (int *)LEDR_BASE;
    HEX3_HEX0_ptr = (unsigned int *)HEX3_HEX0_BASE;
    
    // use timer value as random seed
    volatile int *TIMER_ptr = (int *)TIMER_BASE;
    random_seed = *TIMER_ptr;
    
    // initialize cart-pole state
    state.cart_position = 0.0f;
    state.cart_velocity = 0.0f;
    state.pole_angle = 0.01f;  // start with a small angle to make it unstable
    state.pole_angular_vel = 0.0f;
    
    prev_state = state;
    
    // initialize performance stats
    stats.epoch = 0;
    stats.step = 0;
    stats.total_reward = 0;
    stats.best_reward = 0;
    
    // set front pixel buffer to Buffer 1
    *(pixel_ctrl_ptr + 1) = (int) &Buffer1;
    
    // swap the front/back buffers
    wait_for_vsync();
    
    // initialize a pointer to the pixel buffer
    pixel_buffer_start = *pixel_ctrl_ptr;
    clear_screen();
    
    // set back pixel buffer to Buffer 2
    *(pixel_ctrl_ptr + 1) = (int) &Buffer2;
    pixel_buffer_start = *(pixel_ctrl_ptr + 1);
    clear_screen();
    
    // show start screen
    draw_start_screen();
    
    // main loop - wait for KEY0 to start
    while (1) {
        // check if KEY0 is pressed to start simulation
        if (*KEY_ptr & 0x1) {
            // wait for key release (simple debouncing)
            while (*KEY_ptr & 0x1);
            
            simulation_running = 1;
            stats.epoch++;
            break;
        }
        
        wait_for_vsync();
        pixel_buffer_start = *(pixel_ctrl_ptr + 1);
        
        // keep drawing the start screen while waiting
        clear_screen();
        draw_start_screen();
    }
    
    // main simulation loop
    while (1) {
        // swap buffers
        wait_for_vsync();
        pixel_buffer_start = *(pixel_ctrl_ptr + 1);
        
        // clear the screen
        clear_screen();
        
        // save current state for erasing
        prev_state = state;
        
        if (simulation_running) {
            // choose an action using RL algorithm
            int action = choose_action();
            
            // apply random disturbance periodically
            if (stats.step % 20 == 0) {
                apply_random_disturbance();
            }
            
            // update physics based on action
            update_physics(action);
            
            // update reward
            int reward = (int)calculate_reward();
            stats.total_reward += reward;
            
            // update step counter
            stats.step++;
            
            // check if state is terminal or max steps reached
            if (is_terminal_state() || stats.step >= MAX_STEPS_PER_EPOCH) {
                // update best reward
                if (stats.total_reward > stats.best_reward) {
                    stats.best_reward = stats.total_reward;
                }
                
                // reset state for new epoch
                state.cart_position = 0.0f;
                state.cart_velocity = 0.0f;
                state.pole_angle = (xorshift32() % 100) / 1000.0f - 0.05f;
                state.pole_angular_vel = 0.0f;
                
                // update epoch counter
                stats.epoch++;
                stats.step = 0;
                stats.total_reward = 0;
                
                // turn on LED0 to indicate new epoch
                *LEDR_ptr = 0x1;
            } else {
                // update LEDs to show step count
                *LEDR_ptr = (stats.step & 0x3FF);
            }
        }
        
        // check if KEY0 is pressed to restart
        if (*KEY_ptr & 0x1) {
            // wait for key release
            while (*KEY_ptr & 0x1);
            
            // reset state and start new epoch
            state.cart_position = 0.0f;
            state.cart_velocity = 0.0f;
            state.pole_angle = (xorshift32() % 100) / 1000.0f - 0.05f;
            state.pole_angular_vel = 0.0f;
            
            stats.epoch++;
            stats.step = 0;
            stats.total_reward = 0;
            
            simulation_running = 1;
        }
        
        // draw the cart-pole and stats
        draw_cart_pole();
        draw_stats();
    }
    
    return 0;
}

// plot a pixel at (x, y) with the specified color
void plot_pixel(int x, int y, short int line_color) {
    volatile short int *one_pixel_address;
    
    one_pixel_address = (volatile short int*)(pixel_buffer_start + (y << 10) + (x << 1));
    *one_pixel_address = line_color;
}

// wait for vertical sync (buffer swap)
void wait_for_vsync() {
    volatile int * pixel_ctrl_ptr = (int *)PIXEL_BUF_CTRL_BASE;
    
    // start the synchronization process
    *pixel_ctrl_ptr = 1;
    
    // poll the status bit until it becomes 0
    volatile int *status_ptr = (int *)(PIXEL_BUF_CTRL_BASE + 0xC);
    while ((*status_ptr & 0x01) != 0);
}

// clear the screen to black
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
    // calculate screen coordinates for cart
    int cart_screen_x = VGA_WIDTH / 2 + (int)(state.cart_position * PIXEL_SCALE);
    int cart_screen_y = VGA_HEIGHT - CART_HEIGHT - 10;
    
    // draw the ground
    draw_line(0, VGA_HEIGHT - 5, VGA_WIDTH, VGA_HEIGHT - 5, WHITE);
    
    // draw the cart
    for (int y = 0; y < CART_HEIGHT; y++) {
        for (int x = 0; x < CART_WIDTH; x++) {
            plot_pixel(cart_screen_x - CART_WIDTH/2 + x, cart_screen_y + y, BLUE);
        }
    }
    
    // calculate pole endpoint coordinates
    float pole_endpoint_x = state.cart_position + sinf(state.pole_angle) * POLE_HALF_LENGTH * 2;
    float pole_endpoint_y = -cosf(state.pole_angle) * POLE_HALF_LENGTH * 2;
    
    int pole_screen_endpoint_x = VGA_WIDTH / 2 + (int)(pole_endpoint_x * PIXEL_SCALE);
    int pole_screen_endpoint_y = cart_screen_y + (int)(pole_endpoint_y * PIXEL_SCALE);
    
    // draw the pole as a thick line
    for (int i = -POLE_WIDTH/2; i <= POLE_WIDTH/2; i++) {
        draw_line(cart_screen_x + i, cart_screen_y, 
                  pole_screen_endpoint_x + i, pole_screen_endpoint_y, RED);
    }
    
    // draw a circle at the pivot point
    int radius = 4;
    for (int y = -radius; y <= radius; y++) {
        for (int x = -radius; x <= radius; x++) {
            if (x*x + y*y <= radius*radius) {
                plot_pixel(cart_screen_x + x, cart_screen_y + y, YELLOW);
            }
        }
    }
}

// draw the start screen
void draw_start_screen() {
    int title_x = VGA_WIDTH / 2 - 60;
    int title_y = VGA_HEIGHT / 3;
    
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
    
    // draw a simple cart-pole graphic
    int demo_cart_x = VGA_WIDTH / 2;
    int demo_cart_y = VGA_HEIGHT - 60;
    
    // draw cart
    for (int y = 0; y < 15; y++) {
        for (int x = 0; x < 40; x++) {
            plot_pixel(demo_cart_x - 20 + x, demo_cart_y + y, BLUE);
        }
    }
    
    // draw pole (at an angle)
    int pole_end_x = demo_cart_x + 25;
    int pole_end_y = demo_cart_y - 50;
    
    for (int i = -2; i <= 2; i++) {
        draw_line(demo_cart_x + i, demo_cart_y, pole_end_x + i, pole_end_y, RED);
    }
    
    // draw ground
    draw_line(demo_cart_x - 80, demo_cart_y + 20, 
              demo_cart_x + 80, demo_cart_y + 20, WHITE);
              
    // write "PRESS KEY0 TO START" at the bottom
    int text_x = VGA_WIDTH / 2 - 80;
    int text_y = VGA_HEIGHT - 20;
    
    draw_text(text_x, text_y, "PRESS KEY0 TO START", GREEN);
}

// draw performance stats
void draw_stats() {
    char buffer[20];
    
    // display epoch counter in top right
    draw_text(VGA_WIDTH - 60, 10, "EPOCH", GREEN);
    int_to_str(stats.epoch, buffer);
    draw_text(VGA_WIDTH - 20, 10, buffer, GREEN);
    
    // display step counter
    draw_text(VGA_WIDTH - 50, 20, "STEP", YELLOW);
    int_to_str(stats.step, buffer);
    draw_text(VGA_WIDTH - 20, 20, buffer, YELLOW);
    
    // display reward
    draw_text(10, 10, "REWARD", CYAN);
    int_to_str(stats.total_reward, buffer);
    draw_text(50, 10, buffer, CYAN);
    
    // display best reward
    draw_text(10, 20, "BEST", MAGENTA);
    int_to_str(stats.best_reward, buffer);
    draw_text(40, 20, buffer, MAGENTA);
    
    // display state information
    draw_text(10, VGA_HEIGHT - 10, "POS", WHITE);
    int pos = (int)(state.cart_position * 100);
    int_to_str(pos, buffer);
    draw_text(30, VGA_HEIGHT - 10, buffer, WHITE);
    
    draw_text(60, VGA_HEIGHT - 10, "ANG", WHITE);
    int ang = (int)(state.pole_angle * 57.3); // convert to degrees
    int_to_str(ang, buffer);
    draw_text(85, VGA_HEIGHT - 10, buffer, WHITE);
}

// choose an action (0 = left, 1 = right) using a simple policy
int choose_action() {
    // in initial training phase, use a simple heuristic
    if (state.pole_angle > 0 || (state.pole_angle == 0 && state.pole_angular_vel > 0)) {
        return 1;  // right
    } else {
        return 0;  // left
    }
}

// calculate the reward for the current state
float calculate_reward() {
    // higher reward for keeping the pole upright and centered
    float angle_component = 1.0f - fabsf(state.pole_angle) / 3.14159f;
    float position_component = 1.0f - fabsf(state.cart_position) / 2.0f;
    
    return 10.0f * angle_component + 5.0f * position_component;
}

// check if the state is terminal (pole fallen or cart out of bounds)
int is_terminal_state() {
    // terminal if pole angle exceeds ±12 degrees (0.2094 radians)
    if (fabsf(state.pole_angle) > 0.2094f) return 1;
    
    // terminal if cart position exceeds ±2.0 meters
    if (fabsf(state.cart_position) > 2.0f) return 1;
    
    return 0;
}

// physics simulation
void update_physics(int action) {
    // apply force based on action (0 = left, 1 = right)
    float force = (action == 0) ? -FORCE_MAG : FORCE_MAG;
    
    // cart-pole physics calculations based on differential equations
    float cosine = cosf(state.pole_angle);
    float sine = sinf(state.pole_angle);
    
    float total_mass = CART_MASS + POLE_MASS;
    
    // calculate acceleration of the cart
    float temp = (force + POLE_MASS * POLE_HALF_LENGTH * state.pole_angular_vel * state.pole_angular_vel * sine) / total_mass;
    
    // calculate angular acceleration of the pole
    float pole_accel = (GRAVITY * sine - cosine * temp) / 
                       (POLE_HALF_LENGTH * (4.0f / 3.0f - POLE_MASS * cosine * cosine / total_mass));
    
    // calculate cart acceleration
    float cart_accel = temp - POLE_MASS * POLE_HALF_LENGTH * pole_accel * cosine / total_mass;
    
    // update state using euler integration
    state.cart_position += TIME_STEP * state.cart_velocity;
    state.cart_velocity += TIME_STEP * cart_accel;
    state.pole_angle += TIME_STEP * state.pole_angular_vel;
    state.pole_angular_vel += TIME_STEP * pole_accel;
    
    // normalize pole angle to [-PI, PI]
    while (state.pole_angle > 3.14159f) state.pole_angle -= 2.0f * 3.14159f;
    while (state.pole_angle < -3.14159f) state.pole_angle += 2.0f * 3.14159f;
}

// apply a random disturbance to the pole
void apply_random_disturbance() {
    // generate a random force between -RANDOM_FORCE_MAX and RANDOM_FORCE_MAX
    float random_force = ((float)xorshift32() / 4294967295.0f) * (2.0f * RANDOM_FORCE_MAX) - RANDOM_FORCE_MAX;

    state.pole_angular_vel += random_force;
}

// simple xorshift32 PRNG
unsigned int xorshift32() {
    random_seed ^= random_seed << 13;
    random_seed ^= random_seed >> 17;
    random_seed ^= random_seed << 5;
    return random_seed;
}