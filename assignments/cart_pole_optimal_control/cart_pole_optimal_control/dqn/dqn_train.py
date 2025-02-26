from dqn_agent import DQNAgent
import gymnasium as gym
import numpy as np
import torch
import random
import matplotlib.pyplot as plt

# Initialize environment
env = gym.make("CartPole-v1")
state_dim = env.observation_space.shape[0] + 1  # Now 5 states (4 original + 1 earthquake force)
action_dim = env.action_space.n

# Initialize DQN Agent
agent = DQNAgent(state_dim, action_dim)

# Earthquake Force Parameters
num_waves = 5
freq_range = [0.5, 4.0]  # Frequency range in Hz
base_amplitude = 15  # Base force amplitude in N
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

# Training Loop
num_episodes = 15000
total_rewards = []
steps_per_episode = []
epsilon_values = []

time_step = 0
for episode in range(1, num_episodes + 1):
    state, _ = env.reset()
    total_reward = 0
    steps = 0
    
    for t in range(1000):
        earthquake_force = generate_earthquake_force(time_step * env_timestep)

        # Apply earthquake force to the environment
        env.unwrapped.force_mag = env.unwrapped.force_mag + earthquake_force  

        # Append earthquake force to the state
        state_with_force = np.append(state, earthquake_force)  

        action = agent.select_action(state_with_force, evaluate=False)
        next_state, _, terminated, truncated, _ = env.step(action)
        done = terminated or truncated

        # Extract key values from next_state
        cart_position = abs(next_state[0])  # Cart position (closer to 0 is better)
        pole_angle = abs(next_state[2])  # Pole angle (smaller is better)

        # **Custom Reward Function**
        base_reward = 1.0  # Small reward for surviving
        pole_stability = 1.0 - (2.5 * pole_angle)  # Penalize pole angle deviation
        cart_stability = 1.0 - (0.5 * cart_position)  # Penalize cart displacement
        
        # Calculate total reward
        reward = base_reward + pole_stability + cart_stability

        # Clip reward to avoid negative values
        reward = max(reward, 0)

        # Append earthquake force to the next state
        next_state_with_force = np.append(next_state, earthquake_force)

        agent.store_transition(state_with_force, action, reward, next_state_with_force, done)
        agent.train()

        state = next_state
        time_step += 1
        total_reward += reward
        steps += 1

        if done:
            break

    agent.update_target_model()
    total_rewards.append(total_reward)
    steps_per_episode.append(steps)
    epsilon_values.append(agent.epsilon) 
    print(f"Episode {episode}, Total Reward: {total_reward:.4f}, Steps: {steps}, Exploration Rate (ε): {agent.epsilon:.6f}")

# Save model at the end of training
agent.save_model()

env.close()

# Plot Training Performance
plt.figure()
plt.subplot(3, 1, 1)
plt.plot(total_rewards, label="Total Reward")
plt.xlabel("Episode")
plt.ylabel("Total Reward")
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(steps_per_episode, label="Steps per Episode")
plt.xlabel("Episode")
plt.ylabel("Steps")
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(epsilon_values, label="Exploration Rate (ε)")
plt.xlabel("Episode")
plt.ylabel("Epsilon")
plt.legend()

plt.show()
