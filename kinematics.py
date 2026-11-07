
"""
Differential Drive Kinematics
Forward and Inverse Kinematics
"""

from robot_parameters import RobotParameters


class DifferentialDriveKinematics:

    def __init__(self, parameters):

        self.parameters = parameters
        self.parameters.validate()

    # Convert wheel angular velocity to linear velocity
    def wheel_angular_to_linear(self, omega_left, omega_right):

        r = self.parameters.wheel_radius

        v_left = r * omega_left
        v_right = r * omega_right

        return v_left, v_right

    # Forward kinematics
    def forward_kinematics(self, omega_left, omega_right):

        v_left, v_right = self.wheel_angular_to_linear(
            omega_left, omega_right
        )

        L = self.parameters.wheelbase

        linear_velocity = (v_right + v_left) / 2

        angular_velocity = (v_right - v_left) / L

        return linear_velocity, angular_velocity

    # Inverse kinematics
    def inverse_kinematics(self, linear_velocity, angular_velocity):

        L = self.parameters.wheelbase
        r = self.parameters.wheel_radius

        v_right = linear_velocity + (angular_velocity * L / 2)

        v_left = linear_velocity - (angular_velocity * L / 2)

        omega_left = v_left / r
        omega_right = v_right / r

        return omega_left, omega_right


# Numerical testing
if __name__ == "__main__":

    robot = RobotParameters()

    kinematics = DifferentialDriveKinematics(robot)

    print("DIFFERENTIAL DRIVE KINEMATICS")
    print("-----------------------------")

    # Test 1: Forward kinematics
    left_wheel = 5.0
    right_wheel = 5.0

    v, omega = kinematics.forward_kinematics(
        left_wheel, right_wheel
    )

    print("\nTEST 1: Forward Kinematics")
    print(f"Left wheel angular velocity: {left_wheel} rad/s")
    print(f"Right wheel angular velocity: {right_wheel} rad/s")
    print(f"Robot linear velocity: {v:.3f} m/s")
    print(f"Robot angular velocity: {omega:.3f} rad/s")

    # Test 2: Inverse kinematics
    desired_v = 0.30
    desired_omega = 0.50

    left, right = kinematics.inverse_kinematics(
        desired_v, desired_omega
    )

    print("\nTEST 2: Inverse Kinematics")
    print(f"Desired linear velocity: {desired_v} m/s")
    print(f"Desired angular velocity: {desired_omega} rad/s")
    print(f"Left wheel angular velocity: {left:.3f} rad/s")
    print(f"Right wheel angular velocity: {right:.3f} rad/s")

    # Test 3: Numerical verification
    recovered_v, recovered_omega = (
        kinematics.forward_kinematics(left, right)
    )

    print("\nTEST 3: Numerical Verification")
    print(f"Recovered linear velocity: {recovered_v:.3f} m/s")
    print(f"Recovered angular velocity: {recovered_omega:.3f} rad/s")

    assert abs(recovered_v - desired_v) < 1e-9
    assert abs(recovered_omega - desired_omega) < 1e-9

    print("\nAll kinematics tests passed!")
