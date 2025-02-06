#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm
import seaborn as sns

class DifferentialDriveParticleFilter:
    def __init__(self, n_particles=1000):
        self.n_particles = n_particles
        np.random.seed(42)  # For reproducibility
        
        # Initialize true state and particles [x, y, theta]
        self.true_state = np.array([0.0, 0.0, 0.0])
        self.particles = np.zeros((n_particles, 3))
        self.particles[:, 0:2] = np.random.normal(0, 2, size=(n_particles, 2))  # Position
        self.particles[:, 2] = np.random.uniform(-np.pi, np.pi, n_particles)    # Heading
        self.weights = np.ones(n_particles) / n_particles
        
        # Motion model noise parameters
        self.alpha1 = 0.1  # Noise in rotation
        self.alpha2 = 0.1  # Noise in translation
        self.alpha3 = 0.1  # Noise in rotation due to translation
        self.alpha4 = 0.1  # Noise in translation due to rotation
        
        # GPS measurement noise (meters)
        self.gps_xy_std = 1.0
        
        # Store history for visualization
        self.true_trajectory = [self.true_state.copy()]
        self.estimated_trajectory = [self.particles.mean(axis=0)]
        
    def sample_motion_model(self, state, control, dt):
        """
        Sample from motion model for differential drive robot
        control: [v, omega] - linear and angular velocity
        """
        v, omega = control
        x, y, theta = state
        
        # Sample noisy controls
        v_noise = np.random.normal(0, self.alpha1 * v**2 + self.alpha2 * omega**2)
        omega_noise = np.random.normal(0, self.alpha3 * v**2 + self.alpha4 * omega**2)
        
        v_hat = v + v_noise
        omega_hat = omega + omega_noise
        
        # Avoid division by zero
        if abs(omega_hat) < 1e-6:
            x_new = x + v_hat * np.cos(theta) * dt
            y_new = y + v_hat * np.sin(theta) * dt
            theta_new = theta
        else:
            x_new = x + (-v_hat/omega_hat * np.sin(theta) + 
                        v_hat/omega_hat * np.sin(theta + omega_hat * dt))
            y_new = y + (v_hat/omega_hat * np.cos(theta) - 
                        v_hat/omega_hat * np.cos(theta + omega_hat * dt))
            theta_new = theta + omega_hat * dt
        
        # Normalize angle
        theta_new = np.mod(theta_new + np.pi, 2*np.pi) - np.pi
        
        return np.array([x_new, y_new, theta_new])
        
    def motion_update(self, control, dt):
        """
        Update particles based on control input
        control: [v, omega] - linear and angular velocity
        """
        # Move true state
        self.true_state = self.sample_motion_model(self.true_state, control, dt)
        
        # Move particles
        for i in range(self.n_particles):
            self.particles[i] = self.sample_motion_model(self.particles[i], control, dt)
        
        # Store trajectories
        self.true_trajectory.append(self.true_state.copy())
        self.estimated_trajectory.append(self.particles.mean(axis=0))
    
    def measurement_update(self, gps_measurement):
        """
        Update particle weights based on GPS measurement
        gps_measurement: [x, y] observed position
        """
        # Calculate likelihood for each particle (GPS measurement model)
        for i in range(self.n_particles):
            # Gaussian likelihood for x and y independently
            likelihood_x = norm.pdf(self.particles[i, 0] - gps_measurement[0], 
                                  0, self.gps_xy_std)
            likelihood_y = norm.pdf(self.particles[i, 1] - gps_measurement[1], 
                                  0, self.gps_xy_std)
            self.weights[i] *= likelihood_x * likelihood_y
        
        # Normalize weights
        self.weights /= np.sum(self.weights)
        
        # Resample if effective sample size is too low
        n_eff = 1.0 / np.sum(self.weights ** 2)
        if n_eff < self.n_particles / 2:
            self.resample()
    
    def resample(self):
        """
        Resample particles based on their weights using systematic resampling
        """
        positions = (np.random.random() + np.arange(self.n_particles)) / self.n_particles
        cumsum = np.cumsum(self.weights)
        
        indices = np.zeros(self.n_particles, dtype=int)
        j = 0
        for i in range(self.n_particles):
            while cumsum[j] < positions[i]:
                j += 1
            indices[i] = j
        
        self.particles = self.particles[indices]
        self.weights = np.ones(self.n_particles) / self.n_particles
    
    def visualize(self, step):
        """
        Visualize particles, true state, and estimated state
        """
        plt.clf()
        
        # Plot particles with arrows to show heading
        plt.quiver(self.particles[:, 0], self.particles[:, 1], 
                  np.cos(self.particles[:, 2]), np.sin(self.particles[:, 2]),
                  color='gray', alpha=0.3, scale=30, label='Particles')
        
        # Plot true trajectory and current pose
        true_traj = np.array(self.true_trajectory)
        plt.plot(true_traj[:, 0], true_traj[:, 1], 'g-', label='True Path')
        plt.quiver(self.true_state[0], self.true_state[1],
                  np.cos(self.true_state[2]), np.sin(self.true_state[2]),
                  color='green', scale=10, label='True Pose')
        
        # Plot estimated trajectory and current pose
        est_traj = np.array(self.estimated_trajectory)
        plt.plot(est_traj[:, 0], est_traj[:, 1], 'r--', label='Estimated Path')
        mean_pose = np.mean(self.particles, axis=0)
        plt.quiver(mean_pose[0], mean_pose[1],
                  np.cos(mean_pose[2]), np.sin(mean_pose[2]),
                  color='red', scale=10, label='Estimated Pose')
        
        plt.title(f'Differential Drive Robot Particle Filter Step {step}\n' + 
                 r'$p(x,y,\theta_t|z_{1:t}, u_{1:t})$')
        plt.xlabel('X Position (m)')
        plt.ylabel('Y Position (m)')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.xlim(-5, 5)
        plt.ylim(-5, 5)
        plt.pause(0.1)

def main():
    # Create particle filter
    pf = DifferentialDriveParticleFilter()
    plt.figure(figsize=(10, 10))
    
    # Simulation parameters
    dt = 0.1
    n_steps = 100
    
    # Control inputs for figure-8 trajectory
    time = np.arange(0, n_steps*dt, dt)
    v = 1.0  # Constant linear velocity
    omega = 2.0 * np.sin(time)  # Time-varying angular velocity
    
    for step in range(n_steps):
        # Get control input for this step
        control = np.array([v, omega[step]])
        
        # Motion update
        pf.motion_update(control, dt)
        
        # Generate noisy GPS measurement
        gps_measurement = pf.true_state[0:2] + np.random.normal(0, pf.gps_xy_std, size=2)
        
        # Measurement update
        pf.measurement_update(gps_measurement)
        
        # Visualize
        pf.visualize(step)
    
    plt.show()

if __name__ == '__main__':
    main() 