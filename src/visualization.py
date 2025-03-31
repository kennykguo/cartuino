import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import pygame
import struct
import argparse
import time
import os
import math
from pygame.locals import *

# Match the exact constants from your C implementation
STATE_DIM = 4
ACTION_DIM = 2
HIDDEN_DIM = 64

# Cart-Pole parameters from your implementation
GRAVITY = 9.8
CART_MASS = 1.0
POLE_MASS = 0.2
POLE_HALF_LENGTH = 0.75
MAX_ANGLE_RAD = 0.75
MAX_POSITION = 5.0
MAX_ANGULAR_VELOCITY = 8.0
TIME_STEP = 0.04

# Visualization parameters
SCREEN_WIDTH = 1200
SCREEN_HEIGHT = 400
SCALE = 50  # pixels per inch
FPS = 60

# Visual colors
BACKGROUND_COLOR = (240, 240, 240)
CART_COLOR = (50, 50, 50)
POLE_COLOR = (200, 100, 50)
TEXT_COLOR = (10, 10, 10)
TRACK_COLOR = (180, 180, 180)
INDICATOR_COLOR = (255, 0, 0)
GRID_COLOR = (200, 200, 200)

# Neural Network Architecture (exactly matching your C implementation)
class ActorCritic(nn.Module):
    def __init__(self):
        super(ActorCritic, self).__init__()
        
        # Policy Network (Actor)
        self.actor_layer1 = nn.Linear(STATE_DIM, HIDDEN_DIM)
        self.actor_hidden = nn.Linear(HIDDEN_DIM, HIDDEN_DIM)
        self.actor_layer2 = nn.Linear(HIDDEN_DIM, ACTION_DIM)
        
        # Value Network (Critic)
        self.critic_layer1 = nn.Linear(STATE_DIM, HIDDEN_DIM)
        self.critic_hidden = nn.Linear(HIDDEN_DIM, HIDDEN_DIM)
        self.critic_layer2 = nn.Linear(HIDDEN_DIM, 1)
        
    def forward(self, state):
        # Actor (Policy Network)
        actor_hidden1 = F.leaky_relu(self.actor_layer1(state), 0.01)
        actor_hidden2 = F.leaky_relu(self.actor_hidden(actor_hidden1), 0.01)
        action_probs = F.softmax(self.actor_layer2(actor_hidden2), dim=-1)
        
        # Critic (Value Network)
        critic_hidden1 = F.leaky_relu(self.critic_layer1(state), 0.01)
        critic_hidden2 = F.leaky_relu(self.critic_hidden(critic_hidden1), 0.01)
        value = self.critic_layer2(critic_hidden2)
        
        return action_probs, value
    
    def act(self, state):
        state = torch.FloatTensor(state).unsqueeze(0)
        action_probs, _ = self.forward(state)
        
        # Take the most likely action in inference mode
        action = torch.argmax(action_probs).item()
        
        return action, action_probs.detach().numpy()[0]

# Function to import weights from binary file
def import_weights(model, filename='cart_pole_weights.bin'):
    """Import weights from binary file into PyTorch model"""
    if not os.path.exists(filename):
        print(f"Error: Weight file '{filename}' not found.")
        return False
    
    try:
        # Create arrays to hold the weights
        weights = {
            # Actor (Policy) Network
            'weights1': np.zeros((STATE_DIM, HIDDEN_DIM), dtype=np.float32),
            'bias1': np.zeros(HIDDEN_DIM, dtype=np.float32),
            'weights_h2h': np.zeros((HIDDEN_DIM, HIDDEN_DIM), dtype=np.float32),
            'bias_h2': np.zeros(HIDDEN_DIM, dtype=np.float32),
            'weights2': np.zeros((HIDDEN_DIM, ACTION_DIM), dtype=np.float32),
            'bias2': np.zeros(ACTION_DIM, dtype=np.float32),
            
            # Critic (Value) Network
            'value_weights1': np.zeros((STATE_DIM, HIDDEN_DIM), dtype=np.float32),
            'value_bias1': np.zeros(HIDDEN_DIM, dtype=np.float32),
            'value_weights_h2h': np.zeros((HIDDEN_DIM, HIDDEN_DIM), dtype=np.float32),
            'value_bias_h2': np.zeros(HIDDEN_DIM, dtype=np.float32),
            'value_weights2': np.zeros((HIDDEN_DIM, 1), dtype=np.float32),
            'value_bias2': np.zeros(1, dtype=np.float32),
        }
        
        # Read binary file
        with open(filename, 'rb') as f:
            # Read actor network weights in same order as C struct
            for i in range(STATE_DIM):
                for j in range(HIDDEN_DIM):
                    weights['weights1'][i][j] = struct.unpack('f', f.read(4))[0]
            
            for i in range(HIDDEN_DIM):
                weights['bias1'][i] = struct.unpack('f', f.read(4))[0]
            
            for i in range(HIDDEN_DIM):
                for j in range(HIDDEN_DIM):
                    weights['weights_h2h'][i][j] = struct.unpack('f', f.read(4))[0]
            
            for i in range(HIDDEN_DIM):
                weights['bias_h2'][i] = struct.unpack('f', f.read(4))[0]
            
            for i in range(HIDDEN_DIM):
                for j in range(ACTION_DIM):
                    weights['weights2'][i][j] = struct.unpack('f', f.read(4))[0]
            
            for i in range(ACTION_DIM):
                weights['bias2'][i] = struct.unpack('f', f.read(4))[0]
            
            # Read critic network weights
            for i in range(STATE_DIM):
                for j in range(HIDDEN_DIM):
                    weights['value_weights1'][i][j] = struct.unpack('f', f.read(4))[0]
            
            for i in range(HIDDEN_DIM):
                weights['value_bias1'][i] = struct.unpack('f', f.read(4))[0]
            
            for i in range(HIDDEN_DIM):
                for j in range(HIDDEN_DIM):
                    weights['value_weights_h2h'][i][j] = struct.unpack('f', f.read(4))[0]
            
            for i in range(HIDDEN_DIM):
                weights['value_bias_h2'][i] = struct.unpack('f', f.read(4))[0]
            
            for i in range(HIDDEN_DIM):
                weights['value_weights2'][i][0] = struct.unpack('f', f.read(4))[0]
            
            weights['value_bias2'][0] = struct.unpack('f', f.read(4))[0]
            
            # There's one more float for reward field in the C struct
            reward = struct.unpack('f', f.read(4))[0]
        
        # Convert weights to PyTorch tensors and transpose (since C and PyTorch have different layouts)
        model.actor_layer1.weight.data = torch.FloatTensor(weights['weights1'].T)
        model.actor_layer1.bias.data = torch.FloatTensor(weights['bias1'])
        model.actor_hidden.weight.data = torch.FloatTensor(weights['weights_h2h'].T)
        model.actor_hidden.bias.data = torch.FloatTensor(weights['bias_h2'])
        model.actor_layer2.weight.data = torch.FloatTensor(weights['weights2'].T)
        model.actor_layer2.bias.data = torch.FloatTensor(weights['bias2'])
        
        model.critic_layer1.weight.data = torch.FloatTensor(weights['value_weights1'].T)
        model.critic_layer1.bias.data = torch.FloatTensor(weights['value_bias1'])
        model.critic_hidden.weight.data = torch.FloatTensor(weights['value_weights_h2h'].T)
        model.critic_hidden.bias.data = torch.FloatTensor(weights['value_bias_h2'])
        model.critic_layer2.weight.data = torch.FloatTensor(weights['value_weights2'].T)
        model.critic_layer2.bias.data = torch.FloatTensor(weights['value_bias2'])
        
        print(f"Successfully loaded weights from {filename}")
        print(f"Stored reward in file: {reward}")
        return True
    
    except Exception as e:
        print(f"Error importing weights: {e}")
        return False

# Cart-Pole physics simulation
class CartPole:
    def __init__(self):
        self.gravity = GRAVITY
        self.cart_mass = CART_MASS
        self.pole_mass = POLE_MASS
        self.pole_half_length = POLE_HALF_LENGTH
        self.dt = TIME_STEP
        
        self.max_angle_rad = MAX_ANGLE_RAD
        self.max_position = MAX_POSITION
        self.max_angular_velocity = MAX_ANGULAR_VELOCITY
        
        self.force_mag = 150  # Matches FORCE_MAG/MOTOR_MAX_SPEED
        
        self.reset()
    
    def reset(self):
        # Initialize state with small random perturbations
        self.x = np.random.uniform(-0.1, 0.1)  # Cart position (inches)
        self.x_dot = 0.0                        # Cart velocity (inches/s)
        self.theta = np.random.uniform(-0.05, 0.05)  # Pole angle (radians)
        self.theta_dot = 0.0                    # Pole angular velocity (radians/s)
        self.steps = 0
        self.total_reward = 0
        
        self.state = np.array([self.x, self.x_dot, self.theta, self.theta_dot])
        return self.normalize_state()
    
    def step(self, action):
        # Apply force based on action
        force = self.force_mag if action == 1 else -self.force_mag
        
        # Physics calculations (same as your C implementation)
        total_mass = self.cart_mass + self.pole_mass
        pole_mass_length = self.pole_mass * self.pole_half_length
        
        # Calculate acceleration
        temp = (force + pole_mass_length * self.theta_dot**2 * np.sin(self.theta)) / total_mass
        theta_acc = (self.gravity * np.sin(self.theta) - np.cos(self.theta) * temp) / (
            self.pole_half_length * (4.0/3.0 - self.pole_mass * np.cos(self.theta)**2 / total_mass))
        x_acc = temp - pole_mass_length * theta_acc * np.cos(self.theta) / total_mass
        
        # Integrate using Euler method
        self.x += self.dt * self.x_dot
        self.x_dot += self.dt * x_acc
        self.theta += self.dt * self.theta_dot
        self.theta_dot += self.dt * theta_acc
        
        # Update state
        self.state = np.array([self.x, self.x_dot, self.theta, self.theta_dot])
        self.steps += 1
        
        # Calculate reward (similar to your calculate_reward function)
        angle_reward = (1.0 - abs(self.theta) / self.max_angle_rad)**3
        position_reward = (1.0 - abs(self.x) / 10.0)**2
        reward = 15.0 * angle_reward + 5.0 * position_reward + 0.1
        self.total_reward += reward
        
        # Check for terminal conditions
        done = bool(
            abs(self.x) > self.max_position or
            abs(self.theta) > self.max_angle_rad or
            abs(self.theta_dot) > self.max_angular_velocity or
            self.steps >= 1000  # Maximum steps
        )
        
        return self.normalize_state(), reward, done
    
    def normalize_state(self):
        """Match the normalize_state function in your C code"""
        norm_state = np.zeros(STATE_DIM, dtype=np.float32)
        
        # Position: [-10, 10] -> [-1, 1]
        norm_state[0] = self.state[0] / 10.0
        
        # Velocity: clip and normalize to [-1, 1]
        norm_state[1] = np.clip(self.state[1], -10.0, 10.0) / 10.0
        
        # Angle: [-MAX_ANGLE_RAD, MAX_ANGLE_RAD] -> [-1, 1]
        norm_state[2] = self.state[2] / MAX_ANGLE_RAD
        
        # Angular velocity: clip and normalize to [-1, 1]
        norm_state[3] = np.clip(self.state[3], -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY) / MAX_ANGULAR_VELOCITY
        
        # Clamp all values
        norm_state = np.clip(norm_state, -1.0, 1.0)
        
        return norm_state

# Initialize pygame
pygame.init()
pygame.font.init()
font = pygame.font.SysFont('Arial', 16)
large_font = pygame.font.SysFont('Arial', 24, bold=True)

def draw_cart_pole(screen, env, action_probs=None):
    """Draw the cart-pole system on the pygame screen"""
    screen.fill(BACKGROUND_COLOR)
    
    # Constants for drawing
    cart_width = 80
    cart_height = 40
    wheel_radius = 8
    pole_width = 10
    
    # Draw grid lines and scale
    for i in range(-int(MAX_POSITION), int(MAX_POSITION)+1):
        # Vertical grid lines
        if i == 0:  # Center line
            pygame.draw.line(screen, (100, 100, 100), 
                           (SCREEN_WIDTH//2 + i*SCALE, 0), 
                           (SCREEN_WIDTH//2 + i*SCALE, SCREEN_HEIGHT), 
                           2)
        else:
            pygame.draw.line(screen, GRID_COLOR, 
                           (SCREEN_WIDTH//2 + i*SCALE, 0), 
                           (SCREEN_WIDTH//2 + i*SCALE, SCREEN_HEIGHT))
        
        # Position labels
        pos_text = font.render(f"{i}", True, TEXT_COLOR)
        screen.blit(pos_text, (SCREEN_WIDTH//2 + i*SCALE - 5, SCREEN_HEIGHT-30))
    
    # Draw track
    track_y = SCREEN_HEIGHT - 100
    pygame.draw.line(screen, TRACK_COLOR, (0, track_y), (SCREEN_WIDTH, track_y), 5)
    
    # Draw position limits
    limit_left = SCREEN_WIDTH//2 - MAX_POSITION * SCALE
    limit_right = SCREEN_WIDTH//2 + MAX_POSITION * SCALE
    
    pygame.draw.line(screen, INDICATOR_COLOR, (limit_left, track_y-50), (limit_left, track_y+20), 2)
    pygame.draw.line(screen, INDICATOR_COLOR, (limit_right, track_y-50), (limit_right, track_y+20), 2)
    
    # Calculate cart position in pixels
    cart_x = SCREEN_WIDTH//2 + env.x * SCALE
    cart_y = track_y - cart_height//2
    
    # Draw cart
    pygame.draw.rect(screen, CART_COLOR, 
                   (cart_x - cart_width//2, cart_y, cart_width, cart_height))
    
    # Draw wheels
    pygame.draw.circle(screen, (50, 50, 50), 
                     (cart_x - cart_width//4, cart_y + cart_height), wheel_radius)
    pygame.draw.circle(screen, (50, 50, 50), 
                     (cart_x + cart_width//4, cart_y + cart_height), wheel_radius)
    
    # Calculate pole end position
    pole_len = env.pole_half_length * 2 * SCALE
    pole_end_x = cart_x + pole_len * math.sin(env.theta)
    pole_end_y = cart_y + cart_height//2 - pole_len * math.cos(env.theta)
    
    # Draw pole
    pygame.draw.line(screen, POLE_COLOR, 
                   (cart_x, cart_y + cart_height//2), 
                   (pole_end_x, pole_end_y), pole_width)
    
    # Draw pole angle indicator (for small angles it can be hard to see)
    angle_indicator_len = 40
    indicator_end_x = pole_end_x + angle_indicator_len * math.cos(env.theta)
    indicator_end_y = pole_end_y + angle_indicator_len * math.sin(env.theta)
    pygame.draw.line(screen, (0, 0, 255), 
                   (pole_end_x, pole_end_y), 
                   (indicator_end_x, indicator_end_y), 2)
    
    # Draw state information
    pos_text = font.render(f"Position: {env.x:.2f} in", True, TEXT_COLOR)
    vel_text = font.render(f"Velocity: {env.x_dot:.2f} in/s", True, TEXT_COLOR)
    angle_text = font.render(f"Angle: {env.theta:.2f} rad ({env.theta * 180/math.pi:.1f}°)", True, TEXT_COLOR)
    ang_vel_text = font.render(f"Ang Velocity: {env.theta_dot:.2f} rad/s", True, TEXT_COLOR)
    steps_text = font.render(f"Steps: {env.steps}", True, TEXT_COLOR)
    reward_text = font.render(f"Total Reward: {env.total_reward:.1f}", True, TEXT_COLOR)
    
    screen.blit(pos_text, (20, 20))
    screen.blit(vel_text, (20, 45))
    screen.blit(angle_text, (20, 70))
    screen.blit(ang_vel_text, (20, 95))
    screen.blit(steps_text, (20, 120))
    screen.blit(reward_text, (20, 145))
    
    # Draw action probabilities if available
    if action_probs is not None:
        # Action probability bars
        bar_width = 100
        bar_height = 20
        
        left_prob = action_probs[0]
        right_prob = action_probs[1]
        
        # Left action
        pygame.draw.rect(screen, (200, 200, 200), (SCREEN_WIDTH - 150, 20, bar_width, bar_height))
        pygame.draw.rect(screen, (50, 50, 250), (SCREEN_WIDTH - 150, 20, int(bar_width * left_prob), bar_height))
        left_text = font.render(f"Left: {left_prob:.2f}", True, TEXT_COLOR)
        screen.blit(left_text, (SCREEN_WIDTH - 150 - 60, 22))
        
        # Right action
        pygame.draw.rect(screen, (200, 200, 200), (SCREEN_WIDTH - 150, 50, bar_width, bar_height))
        pygame.draw.rect(screen, (250, 50, 50), (SCREEN_WIDTH - 150, 50, int(bar_width * right_prob), bar_height))
        right_text = font.render(f"Right: {right_prob:.2f}", True, TEXT_COLOR)
        screen.blit(right_text, (SCREEN_WIDTH - 150 - 60, 52))
    
    # Draw control instructions
    instructions = [
        "SPACE: Reset",
        "P: Pause/Resume",
        "M: Toggle Manual/AI",
        "Q/ESC: Quit",
        "R: Random perturbation",
        "UP/DOWN: Speed control",
        "LEFT/RIGHT: Manual control"
    ]
    
    for i, instruction in enumerate(instructions):
        instruction_text = font.render(instruction, True, TEXT_COLOR)
        screen.blit(instruction_text, (SCREEN_WIDTH - 200, SCREEN_HEIGHT - 170 + i*25))

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Cart-Pole Visualizer for Trained Weights')
    parser.add_argument('--weights', type=str, default='cart_pole_weights.bin', 
                        help='Path to weights file (default: cart_pole_weights.bin)')
    parser.add_argument('--fps', type=int, default=60, 
                        help='Frames per second (default: 60)')
    parser.add_argument('--manual', action='store_true', 
                        help='Start in manual control mode (default: False)')
    args = parser.parse_args()
    
    # Set up display
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption('Cart-Pole Visualizer')
    clock = pygame.time.Clock()
    
    # Create model and environment
    model = ActorCritic()
    env = CartPole()
    
    # Load weights
    if not import_weights(model, args.weights):
        print("Failed to load weights. Using random initialization.")
    
    # Initialize state
    state = env.reset()
    running = True
    paused = False
    manual_control = args.manual
    action = 0
    action_probs = np.array([0.5, 0.5])  # Default equal probabilities
    
    # Control speed
    sim_speed = 1.0  # Simulation speed multiplier
    
    # Main loop
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE or event.key == pygame.K_q:
                    running = False
                elif event.key == pygame.K_SPACE:
                    # Reset simulation
                    state = env.reset()
                    action = 0
                    action_probs = np.array([0.5, 0.5])
                elif event.key == pygame.K_p:
                    # Toggle pause
                    paused = not paused
                elif event.key == pygame.K_m:
                    # Toggle manual control
                    manual_control = not manual_control
                elif event.key == pygame.K_r:
                    # Apply random perturbation
                    env.theta += np.random.uniform(-0.1, 0.1)
                    env.x += np.random.uniform(-0.5, 0.5)
                    state = env.normalize_state()
                elif event.key == pygame.K_UP:
                    # Increase simulation speed
                    sim_speed = min(sim_speed * 1.5, 5.0)
                    print(f"Simulation speed: {sim_speed}x")
                elif event.key == pygame.K_DOWN:
                    # Decrease simulation speed
                    sim_speed = max(sim_speed / 1.5, 0.1)
                    print(f"Simulation speed: {sim_speed}x")
        
        # Manual control with arrow keys
        if manual_control:
            keys = pygame.key.get_pressed()
            if keys[pygame.K_LEFT]:
                action = 0  # Left
                action_probs = np.array([1.0, 0.0])
            elif keys[pygame.K_RIGHT]:
                action = 1  # Right
                action_probs = np.array([0.0, 1.0])
            else:
                action = -1  # No action
                action_probs = np.array([0.5, 0.5])
        
        # Update physics
        if not paused:
            # Use model for control if not in manual mode
            if not manual_control:
                action, action_probs = model.act(state)
            
            # Apply either AI or manual action
            if action != -1:  # -1 means no action in manual mode
                next_state, reward, done = env.step(action)
                state = next_state
            
            # Reset if done
            if done:
                print(f"Episode ended after {env.steps} steps with total reward {env.total_reward:.2f}")
                state = env.reset()
        
        # Draw current state
        draw_cart_pole(screen, env, action_probs)
        
        # Draw status indicators
        status_text = ""
        if paused:
            status_text += "PAUSED | "
        status_text += "Mode: " + ("MANUAL" if manual_control else "AI")
        status_text += f" | Speed: {sim_speed:.1f}x"
        status = large_font.render(status_text, True, (200, 0, 0) if paused else (0, 100, 0))
        screen.blit(status, (SCREEN_WIDTH//2 - status.get_width()//2, 10))
        
        # Update display
        pygame.display.flip()
        
        # Control frame rate based on simulation speed
        clock.tick(int(args.fps * sim_speed))
    
    pygame.quit()

if __name__ == "__main__":
    main()