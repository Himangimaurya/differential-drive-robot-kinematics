import os
import numpy as np
import matplotlib.pyplot as plt

from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Circle

from main import run_simulation, SIMULATION_DURATION, DT
from robot_parameters import RobotParameters


class RobotAnimation:

    def __init__(self, unconstrained, constrained):

        self.unconstrained = unconstrained
        self.constrained = constrained

        self.fig, self.axes = plt.subplots(1, 2, figsize=(13, 6))

        self.robot_radius = 0.12
        self.wheelbase = 0.30

        self.setup_plot()

    def setup_plot(self):

        datasets = [
            (self.unconstrained, "Unconstrained Motion"),
            (self.constrained, "Constrained Motion")
        ]

        for ax, (data, title) in zip(self.axes, datasets):

            ax.set_title(title)
            ax.set_xlabel("X Position (m)")
            ax.set_ylabel("Y Position (m)")
            ax.set_aspect("equal")
            ax.grid(True)

            ax.plot(
                data["x"],
                data["y"],
                "--",
                alpha=0.3,
                label="Complete trajectory"
            )

            ax.legend()

        all_x = np.concatenate([
            self.unconstrained["x"],
            self.constrained["x"]
        ])

        all_y = np.concatenate([
            self.unconstrained["y"],
            self.constrained["y"]
        ])

        margin = 0.5

        for ax in self.axes:

            ax.set_xlim(all_x.min() - margin, all_x.max() + margin)
            ax.set_ylim(all_y.min() - margin, all_y.max() + margin)

        self.robots = []
        self.trails = []
        self.headings = []
        self.wheels = []

        for ax in self.axes:

            robot = Circle(
                (0, 0),
                self.robot_radius,
                color="royalblue",
                alpha=0.8
            )

            ax.add_patch(robot)

            trail, = ax.plot([], [], linewidth=2)

            heading, = ax.plot([], [], color="red", linewidth=2)

            left_wheel, = ax.plot([], [], color="black", linewidth=5)
            right_wheel, = ax.plot([], [], color="black", linewidth=5)

            self.robots.append(robot)
            self.trails.append(trail)
            self.headings.append(heading)
            self.wheels.append((left_wheel, right_wheel))

    def update(self, frame):

        datasets = [
            self.unconstrained,
            self.constrained
        ]

        for index, data in enumerate(datasets):

            x = data["x"]
            y = data["y"]
            theta = data["theta"]

            current_x = x[frame]
            current_y = y[frame]
            current_theta = theta[frame]

            self.robots[index].center = (current_x, current_y)

            self.trails[index].set_data(
                x[:frame + 1],
                y[:frame + 1]
            )

            # Robot heading
            heading_length = 0.25

            heading_x = current_x + heading_length * np.cos(current_theta)
            heading_y = current_y + heading_length * np.sin(current_theta)

            self.headings[index].set_data(
                [current_x, heading_x],
                [current_y, heading_y]
            )

            # Wheel positions
            offset_x = (self.wheelbase / 2) * np.sin(current_theta)
            offset_y = (self.wheelbase / 2) * np.cos(current_theta)

            left_x = current_x - offset_x
            left_y = current_y + offset_y

            right_x = current_x + offset_x
            right_y = current_y - offset_y

            wheel_length = 0.12

            dx = wheel_length * np.cos(current_theta)
            dy = wheel_length * np.sin(current_theta)

            left_wheel, right_wheel = self.wheels[index]

            left_wheel.set_data(
                [left_x - dx / 2, left_x + dx / 2],
                [left_y - dy / 2, left_y + dy / 2]
            )

            right_wheel.set_data(
                [right_x - dx / 2, right_x + dx / 2],
                [right_y - dy / 2, right_y + dy / 2]
            )

        return []

    def create_animation(self, output_path):

        animation = FuncAnimation(
            self.fig,
            self.update,
            frames=len(self.unconstrained["time"]),
            interval=50,
            blit=False
        )

        animation.save(
            output_path,
            writer=PillowWriter(fps=20)
        )

        plt.close(self.fig)

        print(f"Animation saved successfully: {output_path}")


if __name__ == "__main__":

    parameters = RobotParameters()

    unconstrained = run_simulation(
        parameters,
        constrained=False
    )

    constrained = run_simulation(
        parameters,
        constrained=True
    )

    os.makedirs("outputs", exist_ok=True)

    robot_animation = RobotAnimation(
        unconstrained,
        constrained
    )

    robot_animation.create_animation(
        "outputs/robot_motion_comparison.gif"
    )