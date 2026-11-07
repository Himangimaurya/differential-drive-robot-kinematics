
"""
Robot Configuration
Differential Drive Robot Kinematics Project
"""

from dataclasses import dataclass


@dataclass
class RobotParameters:

    # Physical dimensions
    wheel_radius: float = 0.05
    wheelbase: float = 0.30

    # Wheel speed limit
    max_wheel_angular_velocity: float = 10.0

    # Robot velocity limits
    max_linear_velocity: float = 0.50
    max_rotational_acceleration: float = 1.0

    def validate(self):
        """Validate robot configuration parameters."""

        if self.wheel_radius <= 0:
            raise ValueError("Wheel radius must be positive.")

        if self.wheelbase <= 0:
            raise ValueError("Wheelbase must be positive.")

        if self.max_wheel_angular_velocity <= 0:
            raise ValueError("Maximum wheel speed must be positive.")

        if self.max_linear_velocity <= 0:
            raise ValueError("Maximum linear velocity must be positive.")

        if self.max_rotational_acceleration <= 0:
            raise ValueError(
                "Maximum rotational acceleration must be positive."
            )

        return True


if __name__ == "__main__":

    robot = RobotParameters()

    robot.validate()

    print("ROBOT CONFIGURATION")
    print("-------------------")
    print(f"Wheel radius: {robot.wheel_radius} m")
    print(f"Wheelbase: {robot.wheelbase} m")
    print(
        f"Maximum wheel angular velocity: "
        f"{robot.max_wheel_angular_velocity} rad/s"
    )
    print(f"Maximum linear velocity: {robot.max_linear_velocity} m/s")
    print(
        f"Maximum rotational acceleration: "
        f"{robot.max_rotational_acceleration} rad/s^2"
    )

    print("\nConfiguration validated successfully!")

