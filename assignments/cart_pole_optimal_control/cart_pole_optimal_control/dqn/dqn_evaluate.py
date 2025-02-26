from dqn_agent import DQNAgent
import gymnasium as gym
import torch
import numpy as np
import matplotlib.pyplot as plt
from collections import deque

# Initialize environment with visualization
env = gym.make("CartPole-v1", render_mode="human")  
state_dim = env.observation_space.shape[0] + 1  # Ensure state_dim = 5 (4 original + earthquake force)
action_dim = env.action_space.n

# Load trained model
agent = DQNAgent(state_dim, action_dim)
agent.q_network.load_state_dict(torch.load("dqn_cartpole_trained_default.pth"))
agent.q_network.eval()

# Earthquake Force Parameters
num_waves = 5
freq_range = [0.5, 4.0]  # Frequency range in Hz
base_amplitude = 0  # Base force amplitude in N
env_timestep = 0.02  # Default time step for CartPole environment
frequencies = np.random.uniform(freq_range[0], freq_range[1], num_waves)
phase_shifts = np.random.uniform(0, 2 * np.pi, num_waves)

def generate_earthquake_force(time):
    """Generate earthquake-like force using superposition of sine waves."""
    force = 0.0
    for freq, phase in zip(frequencies, phase_shifts):
        amplitude = base_amplitude * np.random.uniform(0.8, 1.2)
        force += amplitude * np.sin(2 * np.pi * freq * time + phase)
    force += np.random.normal(0, base_amplitude * 0.1)  # Add noise
    return force

# Run evaluation episodes
num_episodes = 10
for episode in range(1, num_episodes + 1):
    state, _ = env.reset()
    total_reward = 0
    time_step = 0
    
    angle_deviation = deque(maxlen=1000)
    cart_position = deque(maxlen=1000)
    control_effort = deque(maxlen=1000)
    earthquake_force = deque(maxlen=1000)

    for t in range(1000):
        earthquake = generate_earthquake_force(time_step * env_timestep)  # Store in a separate variable
        state_with_force = np.append(state[:4], earthquake)
        action = agent.select_action(state_with_force, evaluate=True)

        # Step the environment
        next_state, reward, terminated, truncated, _ = env.step(action)

        # Manually modify cart position based on earthquake force (only if the environment allows direct modification)
        next_state[0] += earthquake * env_timestep  # Apply earthquake force effect

        done = terminated or truncated

        # Extract relevant state info
        cart_x = abs(next_state[0])  # Cart position
        pole_theta = abs(next_state[2])  # Pole angle deviation

        # Store performance data
        cart_position.append(cart_x)
        angle_deviation.append(pole_theta)
        control_effort.append(10 if action == 1 else -10)  # Force applied per action
        earthquake_force.append(earthquake)  # Append correctly now

        state = next_state
        time_step += 1
        total_reward += reward

        if done:
            break

    print(f"Evaluation Episode {episode}, Total Reward: {total_reward}")
    
    # Plot after each episode
    fig, axs = plt.subplots(2, 2, figsize=(10, 6))
    axs[0, 0].plot(cart_position, label="Cart Position (m)", color='blue')
    axs[0, 0].set_title("Cart Position")
    axs[0, 0].legend()

    axs[0, 1].plot(angle_deviation, label="Pole Angle Deviation (°)", color='red')
    axs[0, 1].set_title("Pole Angle Deviation")
    axs[0, 1].legend()
    
    axs[1, 0].plot(earthquake_force, label="Earthquake Force (N)", color='green')
    axs[1, 0].set_title("Earthquake Disturbance")
    axs[1, 0].legend()

    axs[1, 1].plot(control_effort, label="Control Force (N)", color='magenta')
    axs[1, 1].set_title("Control Effort")
    axs[1, 1].legend()


    plt.tight_layout()
    plt.show()

env.close()
