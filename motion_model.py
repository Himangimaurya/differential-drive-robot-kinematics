
"""
Differential Drive Robot Motion Model
Unicycle Model and Discrete Position Updates
"""

import numpy as np


class RobotMotionModel:

    def __init__(self, initial_x=0.0, initial_y=0.0,
                 initial_theta=0.0):

        self.x = initial_x
        self.y = initial_y
        self.theta = initial_theta

        self.trajectory = []

        self.record_position()

    # Record current robot position
    def record_position(self):

        self.trajectory.append(
            [self.x, self.y, self.theta]
        )

    # Update robot position using unicycle model
    def update(self, linear_velocity, angular_velocity, dt):

        if dt <= 0:
            raise ValueError("Time interval must be positive.")

        self.x += (
            linear_velocity * np.cos(self.theta) * dt
        )

        self.y += (
            linear_velocity * np.sin(self.theta) * dt
        )

        self.theta += angular_velocity * dt

        # Keep orientation within [-pi, pi]
        self.theta = (
            self.theta + np.pi
        ) % (2 * np.pi) - np.pi

        self.record_position()

        return self.x, self.y, self.theta

    # Reset robot position
    def reset(self, x=0.0, y=0.0, theta=0.0):

        self.x = x
        self.y = y
        self.theta = theta

        self.trajectory = []

        self.record_position()


# Simulation tests
if __name__ == "__main__":

    dt = 0.1
    simulation_time = 5.0

    steps = int(simulation_time / dt)

    # Test 1: Straight movement
    robot = RobotMotionModel()

    for _ in range(steps):
        robot.update(0.3, 0.0, dt)

    print("TEST 1: STRAIGHT MOVEMENT")
    print(f"Final X: {robot.x:.3f} m")
    print(f"Final Y: {robot.y:.3f} m")
    print(f"Final Theta: {robot.theta:.3f} rad")

    assert abs(robot.x - 1.5) < 1e-9
    assert abs(robot.y) < 1e-9

    # Test 2: Circular movement
    robot.reset()

    for _ in range(steps):
        robot.update(0.3, 0.5, dt)

    print("\nTEST 2: CIRCULAR MOVEMENT")
    print(f"Final X: {robot.x:.3f} m")
    print(f"Final Y: {robot.y:.3f} m")
    print(f"Final Theta: {robot.theta:.3f} rad")

    # Test 3: In-place rotation
    robot.reset()

    for _ in range(steps):
        robot.update(0.0, 1.0, dt)

    print("\nTEST 3: IN-PLACE ROTATION")
    print(f"Final X: {robot.x:.3f} m")
    print(f"Final Y: {robot.y:.3f} m")
    print(f"Final Theta: {robot.theta:.3f} rad")

    assert abs(robot.x) < 1e-9
    assert abs(robot.y) < 1e-9

    print("\nAll motion model tests completed successfully!")
