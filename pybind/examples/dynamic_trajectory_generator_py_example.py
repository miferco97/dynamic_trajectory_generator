# Copyright 2025 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Example for the dynamic_trajectory_generator_py bindings.

Generates a polynomial trajectory through a set of 3D waypoints, samples it
from start to end at a fixed time step and plots the resulting position,
velocity and acceleration profiles along with a 3D view of the path.

Requires matplotlib (``pip install matplotlib`` or
``pip install -e .[examples]`` from the pybind/ directory).
"""

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

from typing import Sequence

from dynamic_trajectory_generator_py import DynamicTrajectory
import matplotlib.pyplot as plt
import numpy as np


DT_EVAL_S = 0.01
MAX_VELOCITY_MPS = 5.0
INITIAL_POSITION_M = np.array([0.0, 0.0, 0.0])
INITIAL_YAW_RAD = 0.0
PATH_FACING = True

WAYPOINTS_M: Sequence[np.ndarray] = [
    np.array([5.0, 0.0, 2.0]),
    np.array([5.0, 5.0, 3.0]),
    np.array([0.0, 5.0, 3.0]),
    np.array([0.0, 0.0, 2.0]),
    np.array([3.0, 3.0, 1.5]),
]


def sample_trajectory(
    traj: DynamicTrajectory,
    dt: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Evaluate the trajectory from ``min_time`` to ``max_time`` at a fixed step.

    :param traj: Generator whose trajectory has already been produced.
    :param dt: Evaluation period in seconds. Must be strictly positive.
    :return: Tuple ``(times, positions, velocities, accelerations, yaws)`` with
        shapes ``(N,), (N, 3), (N, 3), (N, 3), (N,)`` respectively.
    """
    times = np.arange(traj.min_time, traj.max_time, dt)
    positions = np.zeros((times.size, 3))
    velocities = np.zeros((times.size, 3))
    accelerations = np.zeros((times.size, 3))
    yaws = np.zeros(times.size)

    for index, t in enumerate(times):
        ref = traj.evaluate(float(t))
        positions[index] = ref.position
        velocities[index] = ref.velocity
        accelerations[index] = ref.acceleration
        yaws[index] = ref.yaw

    return times, positions, velocities, accelerations, yaws


def plot_trajectory(
    times: np.ndarray,
    positions: np.ndarray,
    velocities: np.ndarray,
    accelerations: np.ndarray,
    waypoints: Sequence[np.ndarray],
    initial_position: np.ndarray,
) -> None:
    """
    Plot the sampled trajectory in the same layout as ``tests/plot_results.py``.

    Figure 1: five stacked subplots (position, velocity, acceleration,
    |velocity|, |acceleration|). Figure 2: 3D view with the waypoints and the
    initial position overlaid.
    """
    x, y, z = positions[:, 0], positions[:, 1], positions[:, 2]
    vx, vy, vz = velocities[:, 0], velocities[:, 1], velocities[:, 2]
    ax_, ay_, az_ = accelerations[:, 0], accelerations[:, 1], accelerations[:, 2]
    speed = np.linalg.norm(velocities, axis=1)
    acc_magnitude = np.linalg.norm(accelerations, axis=1)

    fig, axs = plt.subplots(5, 1, figsize=(10, 18), sharex=True)

    axs[0].plot(times, x, label='x', color='red')
    axs[0].plot(times, y, label='y', color='green')
    axs[0].plot(times, z, label='z', color='blue')
    axs[0].set_title('Position')
    axs[0].set_ylabel('Position (m)')
    axs[0].legend()
    axs[0].grid()

    axs[1].plot(times, vx, label='vx', color='red')
    axs[1].plot(times, vy, label='vy', color='green')
    axs[1].plot(times, vz, label='vz', color='blue')
    axs[1].set_title('Velocity')
    axs[1].set_ylabel('Velocity (m/s)')
    axs[1].legend()
    axs[1].grid()

    axs[2].plot(times, ax_, label='ax', color='red')
    axs[2].plot(times, ay_, label='ay', color='green')
    axs[2].plot(times, az_, label='az', color='blue')
    axs[2].set_title('Acceleration')
    axs[2].set_ylabel('Acceleration (m/s²)')
    axs[2].legend()
    axs[2].grid()

    axs[3].plot(times, speed, label='|v|', color='purple')
    axs[3].set_title('Velocity Magnitude')
    axs[3].set_ylabel('Velocity (m/s)')
    axs[3].legend()
    axs[3].grid()

    axs[4].plot(times, acc_magnitude, label='|a|', color='orange')
    axs[4].set_title('Acceleration Magnitude')
    axs[4].set_xlabel('Time (s)')
    axs[4].set_ylabel('Acceleration (m/s²)')
    axs[4].legend()
    axs[4].grid()

    fig.tight_layout()

    waypoints_arr = np.asarray(waypoints, dtype=float)
    fig_3d = plt.figure(figsize=(8, 8))
    ax_3d = fig_3d.add_subplot(111, projection='3d')
    ax_3d.plot(x, y, z, label='Trajectory', color='purple')
    ax_3d.scatter(
        waypoints_arr[:, 0], waypoints_arr[:, 1], waypoints_arr[:, 2],
        color='red', marker='o', s=40, label='Waypoints',
    )
    ax_3d.scatter(
        initial_position[0], initial_position[1], initial_position[2],
        color='black', marker='^', s=60, label='Initial position',
    )
    ax_3d.set_title('3D Trajectory')
    ax_3d.set_xlabel('X (m)')
    ax_3d.set_ylabel('Y (m)')
    ax_3d.set_zlabel('Z (m)')
    ax_3d.legend()
    ax_3d.grid()


def main() -> None:
    """Generate the trajectory, sample it end-to-end and plot the result."""
    traj = DynamicTrajectory()
    traj.set_current_position(INITIAL_POSITION_M)
    traj.set_current_yaw(INITIAL_YAW_RAD)
    traj.set_max_velocity(MAX_VELOCITY_MPS)
    traj.set_path_facing(PATH_FACING)

    traj.generate_trajectory(WAYPOINTS_M)
    print(
        f'Trajectory generated: t in [{traj.min_time:.3f}, {traj.max_time:.3f}] s, '
        f'v_max = {MAX_VELOCITY_MPS:.2f} m/s, waypoints = {len(WAYPOINTS_M)}'
    )

    times, positions, velocities, accelerations, _yaws = sample_trajectory(traj, DT_EVAL_S)

    plot_trajectory(
        times, positions, velocities, accelerations,
        WAYPOINTS_M, INITIAL_POSITION_M,
    )
    plt.show()


if __name__ == '__main__':
    main()
