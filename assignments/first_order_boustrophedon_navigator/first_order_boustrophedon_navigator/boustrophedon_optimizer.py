#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
import numpy as np
import math
import optuna
from collections import deque

class BoustrophedonOptimizer(Node):
    def __init__(self):
        super().__init__('boustrophedon_optimizer')

        # Declare parameter search ranges and trials
        self.KP_LINEAR_RANGE = (5.0, 15.0)
        self.KD_LINEAR_RANGE = (0.1, 1.0)
        self.KP_ANGULAR_RANGE = (2.0, 10.0)
        self.KD_ANGULAR_RANGE = (0.1, 1.0)
        self.n_trials = 5000

        # Spacing parameter
        self.spacing = 0.3

        # Initialize cross-track error storage and waypoints
        self.cross_track_errors = deque(maxlen=1000)
        self.pose = Pose()
        self.waypoints = self.generate_waypoints()
        self.current_waypoint = 0

        # Error tracking for derivative control
        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0

    def generate_waypoints(self):
        waypoints = []
        y = 8.0
        while y >= 3.0:
            if len(waypoints) % 2 == 0:
                waypoints.append((2.0, y))
                waypoints.append((9.0, y))
            else:
                waypoints.append((9.0, y))
                waypoints.append((2.0, y))
            y -= self.spacing
        return waypoints

    def calculate_cross_track_error(self):
        if self.current_waypoint < 1:
            return 0.0

        start = np.array(self.waypoints[self.current_waypoint - 1])
        end = np.array(self.waypoints[self.current_waypoint])
        pos = np.array([self.pose.x, self.pose.y])

        path_vector = end - start
        path_length = np.linalg.norm(path_vector)
        if path_length < 1e-6:
            return np.linalg.norm(pos - start)

        path_unit = path_vector / path_length
        pos_vector = pos - start

        projection_length = np.dot(pos_vector, path_unit)
        projection_length = max(0, min(path_length, projection_length))
        projected_point = start + projection_length * path_unit

        error_vector = pos - projected_point
        error_sign = np.sign(np.cross(path_unit, error_vector / np.linalg.norm(error_vector)))
        error = np.linalg.norm(error_vector) * error_sign

        self.cross_track_errors.append(abs(error))
        return error

    def simulate_controller(self, Kp_linear, Kd_linear, Kp_angular, Kd_angular):
        self.cross_track_errors.clear()
        self.current_waypoint = 0

        # Reset the pose to the center of the workspace for each trial
        self.pose.x = 5.0
        self.pose.y = 5.0
        self.pose.theta = 0.0
        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0

        for _ in range (100):  # Simulate 100 steps (adjust as needed)
            if self.current_waypoint >= len(self.waypoints):
                break

            target_x, target_y = self.waypoints[self.current_waypoint]
            distance = math.sqrt((self.pose.x - target_x) ** 2 + (self.pose.y - target_y) ** 2)
            target_angle = math.atan2(target_y - self.pose.y, target_x - self.pose.x)
            angular_error = target_angle - self.pose.theta

            while angular_error > math.pi:
                angular_error -= 2 * math.pi
            while angular_error < -math.pi:
                angular_error += 2 * math.pi

            # Compute derivatives of errors
            linear_error_derivative = (distance - self.prev_linear_error) / 0.1
            angular_error_derivative = (angular_error - self.prev_angular_error) / 0.1

            # PD control
            linear_velocity = Kp_linear * distance + Kd_linear * linear_error_derivative
            angular_velocity = Kp_angular * angular_error + Kd_angular * angular_error_derivative

            self.pose.x += linear_velocity * 0.1 * math.cos(self.pose.theta)
            self.pose.y += linear_velocity * 0.1 * math.sin(self.pose.theta)
            self.pose.theta += angular_velocity * 0.1

            # Clamp pose values to ensure they stay within bounds
            self.pose.x = max(0.0, min(11.0, self.pose.x))
            self.pose.y = max(0.0, min(11.0, self.pose.y))

            self.calculate_cross_track_error()

            if distance < 0.1:
                self.current_waypoint += 1

            # Update previous errors
            self.prev_linear_error = distance
            self.prev_angular_error = angular_error

        avg_error = sum(self.cross_track_errors) / len(self.cross_track_errors) if self.cross_track_errors else float('inf')
        return avg_error

    def optimize_gains(self):
        def objective(trial):
            Kp_linear = trial.suggest_float('Kp_linear', *self.KP_LINEAR_RANGE)
            Kd_linear = trial.suggest_float('Kd_linear', *self.KD_LINEAR_RANGE)
            Kp_angular = trial.suggest_float('Kp_angular', *self.KP_ANGULAR_RANGE)
            Kd_angular = trial.suggest_float('Kd_angular', *self.KD_ANGULAR_RANGE)

            avg_error = self.simulate_controller(Kp_linear, Kd_linear, Kp_angular, Kd_angular)
            self.get_logger().info(f'Trial with params: Kp_linear={Kp_linear}, Kd_linear={Kd_linear}, '
                                   f'Kp_angular={Kp_angular}, Kd_angular={Kd_angular}, Avg Error={avg_error:.3f}')
            return avg_error

        # Baseline comparison
        baseline_error = self.simulate_controller(10.0, 0.1, 5.0, 0.2)
        self.get_logger().info(f'Baseline parameters average cross-track error: {baseline_error:.3f}')

        study = optuna.create_study(direction="minimize")
        study.optimize(objective, n_trials=self.n_trials)

        best_params = study.best_params
        best_value = study.best_value

        self.get_logger().info(f'Best parameters found: {best_params}')
        self.get_logger().info(f'Best average cross-track error: {best_value:.3f}')

        with open('best_params.txt', 'w') as f:
            f.write(f'Best parameters: {best_params}\n')
            f.write(f'Best average cross-track error: {best_value:.3f}\n')

def main(args=None):
    rclpy.init(args=args)
    optimizer = BoustrophedonOptimizer()

    try:
        optimizer.optimize_gains()
    except KeyboardInterrupt:
        pass
    finally:
        optimizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
