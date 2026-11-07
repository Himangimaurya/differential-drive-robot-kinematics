
"""
Main Simulation
Differential Drive Robot Kinematics & Velocity Limits

Compares constrained and unconstrained robot motion.
Generates comparative plots, CSV data and JSON reports.
"""

import os
import csv
import json
import numpy as np

from robot_parameters import RobotParameters
from kinematics import DifferentialDriveKinematics
from motion_model import RobotMotionModel
from velocity_constraints import VelocityConstraints
from visualization import RobotVisualizer


# ==========================================
# SIMULATION CONFIGURATION
# ==========================================

SIMULATION_DURATION = 16.0
DT = 0.05
OUTPUT_DIR = "outputs"


# ==========================================
# VELOCITY COMMANDS
# ==========================================

def get_velocity_command(time):

    if time < 4.0:
        return 0.3, 0.0

    elif time < 8.0:
        return 0.8, 2.0

    elif time < 10.0:
        return 0.0, 3.0

    else:
        return 0.4, -1.5


# ==========================================
# RUN ROBOT SIMULATION
# ==========================================

def run_simulation(parameters, constrained=True):

    kinematics = DifferentialDriveKinematics(parameters)

    motion = RobotMotionModel()

    constraints = VelocityConstraints(parameters)

    steps = int(SIMULATION_DURATION / DT)

    results = {
        "time": [],
        "x": [],
        "y": [],
        "theta": [],
        "linear_velocity": [],
        "angular_velocity": [],
        "left_wheel": [],
        "right_wheel": [],
        "angular_acceleration": []
    }

    previous_omega = 0.0

    for step in range(steps):

        current_time = step * DT

        # Get desired velocity commands
        desired_v, desired_omega = get_velocity_command(
            current_time
        )

        if constrained:

            v, omega, left, right = (
                constraints.apply_constraints(
                    desired_v,
                    desired_omega,
                    DT
                )
            )

        else:

            v = desired_v
            omega = desired_omega

            left, right = kinematics.inverse_kinematics(
                v, omega
            )

        # Calculate angular acceleration
        alpha = (omega - previous_omega) / DT

        previous_omega = omega

        # Update robot position
        x, y, theta = motion.update(v, omega, DT)

        # Store simulation results
        results["time"].append((step + 1) * DT)

        results["x"].append(x)
        results["y"].append(y)
        results["theta"].append(theta)

        results["linear_velocity"].append(v)
        results["angular_velocity"].append(omega)

        results["left_wheel"].append(left)
        results["right_wheel"].append(right)

        results["angular_acceleration"].append(alpha)

    results["trajectory"] = np.array(motion.trajectory)

    return results


# ==========================================
# PERFORMANCE METRICS
# ==========================================

def calculate_metrics(results):

    trajectory = results["trajectory"]

    differences = np.diff(
        trajectory[:, :2],
        axis=0
    )

    distance = np.sum(
        np.linalg.norm(differences, axis=1)
    )

    metrics = {

        "simulation_duration_seconds": SIMULATION_DURATION,

        "total_distance_travelled_m": float(distance),

        "maximum_linear_velocity_mps": float(
            np.max(np.abs(results["linear_velocity"]))
        ),

        "maximum_angular_velocity_radps": float(
            np.max(np.abs(results["angular_velocity"]))
        ),

        "maximum_angular_acceleration_radps2": float(
            np.max(np.abs(results["angular_acceleration"]))
        ),

        "maximum_left_wheel_speed_radps": float(
            np.max(np.abs(results["left_wheel"]))
        ),

        "maximum_right_wheel_speed_radps": float(
            np.max(np.abs(results["right_wheel"]))
        ),

        "final_x_m": float(trajectory[-1, 0]),

        "final_y_m": float(trajectory[-1, 1]),

        "final_orientation_rad": float(trajectory[-1, 2])
    }

    return metrics


# ==========================================
# SAVE CSV RESULTS
# ==========================================

def save_csv(unconstrained, constrained):

    filepath = os.path.join(
        OUTPUT_DIR,
        "simulation_results.csv"
    )

    columns = [
        "time",
        "unconstrained_x",
        "unconstrained_y",
        "constrained_x",
        "constrained_y",
        "unconstrained_linear_velocity",
        "constrained_linear_velocity",
        "unconstrained_angular_velocity",
        "constrained_angular_velocity",
        "unconstrained_angular_acceleration",
        "constrained_angular_acceleration",
        "unconstrained_left_wheel",
        "unconstrained_right_wheel",
        "constrained_left_wheel",
        "constrained_right_wheel"
    ]

    with open(filepath, "w", newline="") as file:

        writer = csv.DictWriter(
            file,
            fieldnames=columns
        )

        writer.writeheader()

        for i in range(len(unconstrained["time"])):

            writer.writerow({

                "time": unconstrained["time"][i],

                "unconstrained_x": unconstrained["x"][i],
                "unconstrained_y": unconstrained["y"][i],

                "constrained_x": constrained["x"][i],
                "constrained_y": constrained["y"][i],

                "unconstrained_linear_velocity":
                    unconstrained["linear_velocity"][i],

                "constrained_linear_velocity":
                    constrained["linear_velocity"][i],

                "unconstrained_angular_velocity":
                    unconstrained["angular_velocity"][i],

                "constrained_angular_velocity":
                    constrained["angular_velocity"][i],

                "unconstrained_angular_acceleration":
                    unconstrained["angular_acceleration"][i],

                "constrained_angular_acceleration":
                    constrained["angular_acceleration"][i],

                "unconstrained_left_wheel":
                    unconstrained["left_wheel"][i],

                "unconstrained_right_wheel":
                    unconstrained["right_wheel"][i],

                "constrained_left_wheel":
                    constrained["left_wheel"][i],

                "constrained_right_wheel":
                    constrained["right_wheel"][i]
            })

    print(f"CSV saved: {filepath}")


# ==========================================
# SAVE JSON PERFORMANCE REPORT
# ==========================================

def save_report(unconstrained_metrics, constrained_metrics):

    report = {

        "project": "Differential Drive Robot Kinematics",

        "unconstrained": unconstrained_metrics,

        "constrained": constrained_metrics
    }

    filepath = os.path.join(
        OUTPUT_DIR,
        "performance_report.json"
    )

    with open(filepath, "w") as file:

        json.dump(report, file, indent=4)

    print(f"Performance report saved: {filepath}")


# ==========================================
# MAIN SIMULATION
# ==========================================

def main():

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    print("\n======================================")
    print(" DIFFERENTIAL DRIVE ROBOT SIMULATION")
    print("======================================")

    # Load robot configuration
    parameters = RobotParameters()

    parameters.validate()

    print("\nRobot configuration loaded.")

    # Run unconstrained simulation
    print("\nRunning unconstrained simulation...")

    unconstrained = run_simulation(
        parameters,
        constrained=False
    )

    # Run constrained simulation
    print("Running constrained simulation...")

    constrained = run_simulation(
        parameters,
        constrained=True
    )

    print("\nBoth simulations completed.")

    # Calculate performance metrics
    unconstrained_metrics = calculate_metrics(
        unconstrained
    )

    constrained_metrics = calculate_metrics(
        constrained
    )

    # Display comparison
    print("\n======================================")
    print(" PERFORMANCE COMPARISON")
    print("======================================")

    print("\nUNCONSTRAINED ROBOT")

    for key, value in unconstrained_metrics.items():
        print(f"{key}: {value:.4f}")

    print("\nCONSTRAINED ROBOT")

    for key, value in constrained_metrics.items():
        print(f"{key}: {value:.4f}")

    # ======================================
    # COMPARATIVE VISUALIZATION
    # ======================================

    visualizer = RobotVisualizer()

    print("\nGenerating comparative graphs...")

    # Trajectory comparison
    visualizer.plot_trajectory_comparison(
        unconstrained,
        constrained,
        OUTPUT_DIR
    )

    # Velocity and acceleration comparisons
    visualizer.plot_velocity_comparison(
        unconstrained,
        constrained,
        OUTPUT_DIR
    )

    # ======================================
    # SAVE SIMULATION DATA
    # ======================================

    print("\nSaving simulation results...")

    save_csv(
        unconstrained,
        constrained
    )

    save_report(
        unconstrained_metrics,
        constrained_metrics
    )

    print("\n======================================")
    print(" SIMULATION COMPLETED SUCCESSFULLY")
    print("======================================")

    print(f"\nAll results stored in: {OUTPUT_DIR}/")


# ==========================================
# PROGRAM ENTRY POINT
# ==========================================

if __name__ == "__main__":

    main()
