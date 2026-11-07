
"""
Evaluation and Testing Module
Differential Drive Robot Kinematics Project
"""

import numpy as np

from robot_parameters import RobotParameters
from kinematics import DifferentialDriveKinematics
from motion_model import RobotMotionModel
from velocity_constraints import VelocityConstraints


def test_forward_inverse_kinematics():

    robot = RobotParameters()
    model = DifferentialDriveKinematics(robot)

    desired_v = 0.30
    desired_omega = 0.50

    left, right = model.inverse_kinematics(
        desired_v, desired_omega
    )

    recovered_v, recovered_omega = (
        model.forward_kinematics(left, right)
    )

    assert np.isclose(recovered_v, desired_v)
    assert np.isclose(recovered_omega, desired_omega)

    print("PASS: Forward and inverse kinematics")


def test_straight_movement():

    robot = RobotMotionModel()

    for _ in range(50):
        robot.update(0.3, 0.0, 0.1)

    assert np.isclose(robot.x, 1.5)
    assert np.isclose(robot.y, 0.0)

    print("PASS: Straight movement")


def test_left_and_right_turns():

    left_turn = RobotMotionModel()
    right_turn = RobotMotionModel()

    for _ in range(50):
        left_turn.update(0.2, 0.5, 0.1)
        right_turn.update(0.2, -0.5, 0.1)

    assert left_turn.theta > 0
    assert right_turn.theta < 0

    print("PASS: Left and right turns")


def test_in_place_rotation():

    robot = RobotMotionModel()

    for _ in range(50):
        robot.update(0.0, 0.5, 0.1)

    assert np.isclose(robot.x, 0.0)
    assert np.isclose(robot.y, 0.0)
    assert robot.theta > 0

    print("PASS: In-place rotation")


def test_maximum_linear_velocity():

    parameters = RobotParameters()
    constraints = VelocityConstraints(parameters)

    limited_velocity = constraints.limit_linear_velocity(0.8)

    assert abs(limited_velocity) <= (
        parameters.max_linear_velocity
    )

    print("PASS: Maximum linear velocity")


def test_angular_acceleration():

    parameters = RobotParameters()
    constraints = VelocityConstraints(parameters)

    dt = 0.1
    previous_omega = 0.0

    for _ in range(20):

        omega = constraints.limit_angular_velocity(2.0, dt)

        acceleration = abs(omega - previous_omega) / dt

        assert acceleration <= (
            parameters.max_rotational_acceleration + 1e-9
        )

        previous_omega = omega

    print("PASS: Angular acceleration limit")


def test_maximum_wheel_speed():

    parameters = RobotParameters()
    constraints = VelocityConstraints(parameters)

    v, omega, left, right = constraints.limit_wheel_speeds(
        0.8, 2.0
    )

    maximum = parameters.max_wheel_angular_velocity

    assert abs(left) <= maximum + 1e-9
    assert abs(right) <= maximum + 1e-9

    print("PASS: Maximum wheel speed")


def run_all_tests():

    print("\nDIFFERENTIAL DRIVE ROBOT")
    print("EVALUATION AND VALIDATION")
    print("=" * 40)

    tests = [
        test_forward_inverse_kinematics,
        test_straight_movement,
        test_left_and_right_turns,
        test_in_place_rotation,
        test_maximum_linear_velocity,
        test_angular_acceleration,
        test_maximum_wheel_speed
    ]

    passed = 0

    for test in tests:

        test()
        passed += 1

    print("=" * 40)
    print(f"Tests passed: {passed}/{len(tests)}")
    print("All evaluation tests completed successfully!")


if __name__ == "__main__":

    run_all_tests()
