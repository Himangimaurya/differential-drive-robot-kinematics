
"""
Velocity Constraints for Differential Drive Robot
"""

import numpy as np
from robot_parameters import RobotParameters


class VelocityConstraints:

    def __init__(self, parameters):

        self.parameters = parameters
        self.parameters.validate()

        self.previous_angular_velocity = 0.0

    # 1. Maximum linear velocity
    def limit_linear_velocity(self, velocity):

        maximum = self.parameters.max_linear_velocity

        return np.clip(velocity, -maximum, maximum)

    # 2. Maximum rotational acceleration
    def limit_angular_velocity(self, desired_omega, dt):

        if dt <= 0:
            raise ValueError("Time interval must be positive.")

        alpha_max = self.parameters.max_rotational_acceleration

        max_change = alpha_max * dt

        lower_limit = self.previous_angular_velocity - max_change
        upper_limit = self.previous_angular_velocity + max_change

        limited_omega = np.clip(
            desired_omega,
            lower_limit,
            upper_limit
        )

        self.previous_angular_velocity = limited_omega

        return limited_omega

    # 3. Maximum wheel angular velocity
    def limit_wheel_speeds(self, linear_velocity, angular_velocity):

        r = self.parameters.wheel_radius
        L = self.parameters.wheelbase

        max_wheel_speed = (
            self.parameters.max_wheel_angular_velocity
        )

        # Maximum feasible robot angular velocity
        max_robot_omega = (2 * r * max_wheel_speed) / L

        angular_velocity = np.clip(
            angular_velocity,
            -max_robot_omega,
            max_robot_omega
        )

        # Feasible linear velocity range
        wheel_linear_limit = r * max_wheel_speed

        minimum_v = (
            -wheel_linear_limit
            + abs(angular_velocity) * L / 2
        )

        maximum_v = (
            wheel_linear_limit
            - abs(angular_velocity) * L / 2
        )

        linear_velocity = np.clip(
            linear_velocity,
            minimum_v,
            maximum_v
        )

        # Calculate final wheel angular velocities
        omega_right = (
            linear_velocity + angular_velocity * L / 2
        ) / r

        omega_left = (
            linear_velocity - angular_velocity * L / 2
        ) / r

        return (
            linear_velocity,
            angular_velocity,
            omega_left,
            omega_right
        )

    # 4. Apply all constraints
    def apply_constraints(
        self,
        linear_velocity,
        angular_velocity,
        dt
    ):

        # Apply linear velocity limit
        linear_velocity = self.limit_linear_velocity(
            linear_velocity
        )

        # Apply rotational acceleration limit
        angular_velocity = self.limit_angular_velocity(
            angular_velocity,
            dt
        )

        # Apply wheel speed limits
        (
            linear_velocity,
            angular_velocity,
            omega_left,
            omega_right
        ) = self.limit_wheel_speeds(
            linear_velocity,
            angular_velocity
        )

        return (
            linear_velocity,
            angular_velocity,
            omega_left,
            omega_right
        )

    # 5. Reset angular velocity history
    def reset(self):

        self.previous_angular_velocity = 0.0


# ==========================================
# TESTING
# ==========================================

if __name__ == "__main__":

    robot = RobotParameters()

    constraints = VelocityConstraints(robot)

    dt = 0.1

    print("VELOCITY CONSTRAINT TESTING")
    print("---------------------------")

    # Test 1: Linear velocity
    velocity = constraints.limit_linear_velocity(0.8)

    print("\nTEST 1: Linear Velocity Limit")
    print("Requested: 0.8 m/s")
    print(f"Limited: {velocity:.3f} m/s")

    assert abs(velocity) <= robot.max_linear_velocity

    # Test 2: Angular acceleration
    angular_velocity = constraints.limit_angular_velocity(
        2.0, dt
    )

    print("\nTEST 2: Angular Acceleration Limit")
    print("Requested angular velocity: 2.0 rad/s")
    print(f"Limited angular velocity: {angular_velocity:.3f} rad/s")

    assert abs(angular_velocity) <= (
        robot.max_rotational_acceleration * dt
    )

    # Test 3: Wheel speed
    constraints.reset()

    v, omega, left, right = constraints.limit_wheel_speeds(
        0.8, 2.0
    )

    print("\nTEST 3: Wheel Speed Limit")
    print(f"Limited linear velocity: {v:.3f} m/s")
    print(f"Limited angular velocity: {omega:.3f} rad/s")
    print(f"Left wheel speed: {left:.3f} rad/s")
    print(f"Right wheel speed: {right:.3f} rad/s")

    assert abs(left) <= robot.max_wheel_angular_velocity + 1e-9
    assert abs(right) <= robot.max_wheel_angular_velocity + 1e-9

    print("\nAll velocity constraint tests passed!")
