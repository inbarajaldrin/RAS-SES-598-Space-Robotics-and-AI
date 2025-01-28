![image](https://github.com/user-attachments/assets/b77c1fbb-7a51-4c9f-bc19-c43c981701d6)


# Cart-Pole Optimal Control Assignment

This ROS 2 package implements an LQR (Linear Quadratic Regulator) controller for balancing an inverted pendulum on a cart. The system demonstrates optimal control of an underactuated system, where we control only the cart's position to stabilize both the cart position and pole angle.

## System Description

The cart-pole system consists of:
- A cart that can move horizontally along a rail
- A pole attached to the cart with a revolute joint
- Control input: Force applied to the cart
- State vector: [x, ẋ, θ, θ̇] (cart position, cart velocity, pole angle, pole angular velocity)

## Prerequisites

- ROS 2 Jazzy
- Gazebo Garden
- Required ROS 2 packages:
  ```bash
  sudo apt-get install ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-interfaces
  ```
- Python Control Systems Library:
  ```bash
  sudo apt-get install python3-control
  ```

## Installation

1. Clone this repository into your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone <repository_url>
   ```

2. Build the package:
   ```bash
   cd ~/ros2_ws
   colcon build --symlink-install
   ```

3. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Usage

Launch the simulation with:
```bash
ros2 launch cart_pole_optimal_control cart_pole_gazebo.launch.py
```

### Tuning Controller Parameters

The LQR controller can be tuned using ROS 2 parameters:
```bash
# Adjust position control weight
ros2 param set /cart_pole_lqr Q_x 2.0

# Adjust angle control weight
ros2 param set /cart_pole_lqr Q_theta 20.0

# Adjust control effort penalty
ros2 param set /cart_pole_lqr R 0.5
```

### System Parameters
- `mass_cart`: Mass of the cart (default: 1.0 kg)
- `mass_pole`: Mass of the pole (default: 0.1 kg)
- `pole_length`: Length of the pole (default: 1.0 m)
- `gravity`: Gravitational acceleration (default: 9.81 m/s²)

### Control Parameters
- `Q_x`: Weight for cart position error
- `Q_x_dot`: Weight for cart velocity error
- `Q_theta`: Weight for pole angle error
- `Q_theta_dot`: Weight for pole angular velocity error
- `R`: Weight for control effort

## Testing the Controller

1. Monitor system state:
   ```bash
   ros2 topic echo /cart_pole/joint_states
   ```

2. Apply disturbances:
   ```bash
   ros2 topic pub --once /cart_pole/cart_slider_cmd std_msgs/msg/Float64 "data: 10.0"
   ```

## File Structure

- `cart_pole_lqr.py`: Main LQR controller implementation
- `model.sdf`: Gazebo model definition
- `cart_pole_gazebo.launch.py`: Launch file for Gazebo simulation
- `package.xml`: Package dependencies
- `setup.py`: Package setup and entry points

## Theory

The LQR controller minimizes the quadratic cost function:

$$ J = \int_{0}^{\infty} (x^T Q x + u^T R u) dt $$

where:
- $x = [x, \dot{x}, \theta, \dot{\theta}]^T$ is the state vector
- $u$ is the control input (force on cart)
- $Q \in \mathbb{R}^{4\times4}$ is the state cost matrix
- $R \in \mathbb{R}$ is the control cost scalar

The nonlinear equations of motion for the cart-pole system are:

$$ \ddot{x} = \frac{F + ml\sin(\theta)(\dot{\theta}^2 - \frac{g}{l}\cos(\theta))}{M + m\sin^2(\theta)} $$

$$ \ddot{\theta} = \frac{-F\cos(\theta) - ml\dot{\theta}^2\cos(\theta)\sin(\theta) + (M+m)g\sin(\theta)}{l(M + m\sin^2(\theta))} $$

where:
- $M$ is the cart mass
- $m$ is the pole mass
- $l$ is the pole length
- $g$ is the gravitational acceleration
- $F$ is the applied force

The system is linearized around the unstable equilibrium point $\theta = 0$ (upright position). For small deviations from vertical, $\sin(\theta) \approx \theta$ and $\cos(\theta) \approx 1$, giving the linear state-space model:

$$ \dot{x} = Ax + Bu $$

where:

$$ A = \begin{bmatrix} 
0 & 1 & 0 & 0 \\
0 & 0 & \frac{mg}{M} & 0 \\
0 & 0 & 0 & 1 \\
0 & 0 & \frac{(M+m)g}{Ml} & 0
\end{bmatrix} $$

$$ B = \begin{bmatrix}
0 \\
\frac{1}{M} \\
0 \\
-\frac{1}{Ml}
\end{bmatrix} $$

The LQR controller computes optimal feedback gains $K$ such that $u = -Kx$ minimizes the cost function. The resulting closed-loop system is:

$$ \dot{x} = (A - BK)x $$

The gain matrix $K$ is computed by solving the algebraic Riccati equation:

$$ A^T P + PA - PBR^{-1}B^T P + Q = 0 $$

where $P$ is the solution to the Riccati equation and $K = R^{-1}B^T P$.

## Contributing

Feel free to submit issues and pull requests for improvements.

## License

This work is licensed under a [Creative Commons Attribution 4.0 International License](http://creativecommons.org/licenses/by/4.0/). 
