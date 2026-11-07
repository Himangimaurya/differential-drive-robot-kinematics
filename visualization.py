
"""
Differential Drive Robot Visualization
Comparative plots and robot drawing.
"""

import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle


class RobotVisualizer:

    def __init__(self):
        self.fig, self.ax = plt.subplots(figsize=(9, 7))

    # Draw robot at a particular position
    def draw_robot(self, x, y, theta, wheelbase=0.30):

        self.ax.clear()

        body = Circle(
            (x, y),
            radius=0.12,
            fill=False,
            linewidth=2
        )

        self.ax.add_patch(body)

        # Orientation arrow
        self.ax.arrow(
            x, y,
            0.20 * np.cos(theta),
            0.20 * np.sin(theta),
            head_width=0.05,
            head_length=0.05,
            color="red"
        )

        # Draw wheels
        for sign in [-1, 1]:

            wheel_x = (
                x - sign * (wheelbase / 2) * np.sin(theta)
            )

            wheel_y = (
                y + sign * (wheelbase / 2) * np.cos(theta)
            )

            wheel = Rectangle(
                (wheel_x - 0.04, wheel_y - 0.025),
                0.08,
                0.05,
                angle=np.degrees(theta),
                color="black"
            )

            self.ax.add_patch(wheel)

        self.ax.set_aspect("equal")
        self.ax.grid(True)

        self.ax.set_xlabel("X Position (m)")
        self.ax.set_ylabel("Y Position (m)")
        self.ax.set_title("Differential Drive Robot")

        self.ax.set_xlim(x - 2, x + 2)
        self.ax.set_ylim(y - 2, y + 2)

        plt.pause(0.01)

    # Plot trajectory
    def plot_trajectory(self, trajectory):

        trajectory = np.array(trajectory)

        self.ax.plot(
            trajectory[:, 0],
            trajectory[:, 1],
            label="Robot trajectory"
        )

        self.ax.legend()

    # Save trajectory comparison
    def plot_trajectory_comparison(
        self,
        unconstrained,
        constrained,
        output_dir="outputs"
    ):

        os.makedirs(output_dir, exist_ok=True)

        plt.figure(figsize=(10, 7))

        plt.plot(
            unconstrained["x"],
            unconstrained["y"],
            "--",
            label="Unconstrained"
        )

        plt.plot(
            constrained["x"],
            constrained["y"],
            label="Constrained"
        )

        plt.xlabel("X Position (m)")
        plt.ylabel("Y Position (m)")

        plt.title("Constrained vs Unconstrained Trajectory")

        plt.axis("equal")
        plt.grid(True)
        plt.legend()

        plt.tight_layout()

        plt.savefig(
            os.path.join(output_dir, "trajectory_comparison.png"),
            dpi=300
        )

        plt.show()
        plt.close()

    # Compare velocity and acceleration data
    def plot_velocity_comparison(
        self,
        unconstrained,
        constrained,
        output_dir="outputs"
    ):

        os.makedirs(output_dir, exist_ok=True)

        time = constrained["time"]

        # 1. Linear velocity comparison
        plt.figure(figsize=(10, 5))

        plt.plot(
            time,
            unconstrained["linear_velocity"],
            "--",
            label="Unconstrained"
        )

        plt.plot(
            time,
            constrained["linear_velocity"],
            label="Constrained"
        )

        plt.axhline(
            0.5,
            linestyle=":",
            label="Maximum linear velocity"
        )

        plt.axhline(-0.5, linestyle=":")

        plt.xlabel("Time (s)")
        plt.ylabel("Linear Velocity (m/s)")

        plt.title("Linear Velocity Comparison")

        plt.grid(True)
        plt.legend()
        plt.tight_layout()

        plt.savefig(
            os.path.join(output_dir, "linear_velocity_comparison.png"),
            dpi=300
        )

        plt.show()
        plt.close()

        # 2. Angular velocity comparison
        plt.figure(figsize=(10, 5))

        plt.plot(
            time,
            unconstrained["angular_velocity"],
            "--",
            label="Unconstrained"
        )

        plt.plot(
            time,
            constrained["angular_velocity"],
            label="Constrained"
        )

        plt.xlabel("Time (s)")
        plt.ylabel("Angular Velocity (rad/s)")

        plt.title("Angular Velocity Comparison")

        plt.grid(True)
        plt.legend()
        plt.tight_layout()

        plt.savefig(
            os.path.join(output_dir, "angular_velocity_comparison.png"),
            dpi=300
        )

        plt.show()
        plt.close()

        # 3. Angular acceleration comparison
        plt.figure(figsize=(10, 5))

        plt.plot(
            time,
            unconstrained["angular_acceleration"],
            "--",
            label="Unconstrained"
        )

        plt.plot(
            time,
            constrained["angular_acceleration"],
            label="Constrained"
        )

        plt.axhline(
            1.0,
            linestyle=":",
            label="Maximum acceleration"
        )

        plt.axhline(-1.0, linestyle=":")

        plt.xlabel("Time (s)")
        plt.ylabel("Angular Acceleration (rad/s²)")

        plt.title("Angular Acceleration Comparison")

        plt.grid(True)
        plt.legend()
        plt.tight_layout()

        plt.savefig(
            os.path.join(output_dir, "angular_acceleration_comparison.png"),
            dpi=300
        )

        plt.show()
        plt.close()

        # 4. Wheel velocity comparison
        plt.figure(figsize=(10, 5))

        plt.plot(
            time,
            unconstrained["left_wheel"],
            "--",
            label="Unconstrained Left"
        )

        plt.plot(
            time,
            unconstrained["right_wheel"],
            "--",
            label="Unconstrained Right"
        )

        plt.plot(
            time,
            constrained["left_wheel"],
            label="Constrained Left"
        )

        plt.plot(
            time,
            constrained["right_wheel"],
            label="Constrained Right"
        )

        plt.axhline(
            10.0,
            linestyle=":",
            label="Maximum wheel speed"
        )

        plt.axhline(-10.0, linestyle=":")

        plt.xlabel("Time (s)")
        plt.ylabel("Wheel Angular Velocity (rad/s)")

        plt.title("Wheel Velocity Comparison")

        plt.grid(True)
        plt.legend()
        plt.tight_layout()

        plt.savefig(
            os.path.join(output_dir, "wheel_velocity_comparison.png"),
            dpi=300
        )

        plt.show()
        plt.close()


if __name__ == "__main__":

    visualizer = RobotVisualizer()

    visualizer.draw_robot(
        x=1.0,
        y=0.5,
        theta=np.pi / 4
    )

    plt.show()
