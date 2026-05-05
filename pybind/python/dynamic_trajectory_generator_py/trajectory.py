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

"""Python interface for the Dynamic Trajectory Generator binding."""

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

from dataclasses import dataclass
from typing import Sequence

import numpy as np

from ._dynamic_trajectory_generator_cpp import _DynamicTrajectory


@dataclass
class TrajectoryReference:
    """
    Reference sample evaluated at a given time along the trajectory.

    Units are SI (m, m/s, m/s^2, rad). The inertial frame is ENU.

    :ivar position: Position reference, shape (3,) in meters.
    :vartype position: numpy.ndarray
    :ivar velocity: Linear velocity reference, shape (3,) in m/s.
    :vartype velocity: numpy.ndarray
    :ivar acceleration: Linear acceleration reference, shape (3,) in m/s^2.
    :vartype acceleration: numpy.ndarray
    :ivar yaw: Yaw reference in rad.
    :vartype yaw: float
    """

    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    yaw: float


class DynamicTrajectory:
    """
    Pythonic stateful wrapper around the C++ dynamic trajectory generator.

    Usage pattern:

        traj = DynamicTrajectory()
        traj.set_current_position(np.array([0.0, 0.0, 0.0]))
        traj.set_current_yaw(0.0)
        traj.set_max_velocity(5.0)
        traj.generate_trajectory([np.array([5, 0, 2]), np.array([5, 5, 2])])
        for t in np.arange(traj.min_time, traj.max_time, 0.01):
            ref = traj.evaluate(t)

    The native binding lives in ``self._native`` for advanced usage.
    """

    def __init__(self) -> None:
        """Initialize the wrapper with an empty native trajectory and default state."""
        self._native = _DynamicTrajectory()
        self._current_position = np.zeros(3, dtype=float)
        self._current_yaw = 0.0
        self._max_velocity = 1.0

    def set_current_position(self, position: np.ndarray) -> None:
        """
        Set the current vehicle position (trajectory start).

        :param position: 3D position in meters (ENU).
        :type position: numpy.ndarray
        """
        self._current_position = np.asarray(position, dtype=float).reshape(3)

    def set_current_yaw(self, yaw: float) -> None:
        """
        Set the current yaw reference (trajectory start).

        :param yaw: Yaw angle in rad.
        :type yaw: float
        """
        self._current_yaw = float(yaw)

    def set_max_velocity(self, max_velocity: float) -> None:
        """
        Set the maximum linear velocity along the trajectory.

        :param max_velocity: Maximum linear speed in m/s. Must be strictly positive.
        :type max_velocity: float
        :raises ValueError: If ``max_velocity`` is not strictly positive.
        """
        if max_velocity <= 0.0:
            raise ValueError('max_velocity must be strictly positive')
        self._max_velocity = float(max_velocity)

    def set_path_facing(self, enable: bool) -> None:
        """
        Enable or disable automatic yaw alignment with the trajectory velocity.

        When enabled, the yaw reference returned by :meth:`evaluate` is derived
        from the horizontal velocity direction (with a low-speed fallback that
        holds the last known yaw).

        :param enable: True to enable path-facing yaw.
        :type enable: bool
        """
        self._native.set_path_facing(bool(enable))

    def generate_trajectory(self, waypoints: Sequence[np.ndarray]) -> None:
        """
        Generate a polynomial trajectory through ``waypoints``.

        Uses the stored current position, yaw and maximum velocity as initial
        conditions. Blocks until the optimizer returns a valid solution.

        :param waypoints: Ordered sequence of 3D waypoints in meters (ENU).
            At least two entries are required.
        :type waypoints: Sequence[numpy.ndarray]
        :raises ValueError: If fewer than two waypoints are provided.
        :raises RuntimeError: If the underlying optimizer rejects the inputs.
        """
        if len(waypoints) < 2:
            raise ValueError('At least two waypoints are required to generate a trajectory')
        native_waypoints = [np.asarray(w, dtype=float).reshape(3) for w in waypoints]
        self._native.generate_trajectory(
            self._current_position,
            self._current_yaw,
            native_waypoints,
            self._max_velocity,
        )

    def evaluate(self, time: float) -> TrajectoryReference:
        """
        Evaluate the trajectory at ``time``.

        :param time: Evaluation time in seconds. Must lie within
            ``[min_time, max_time]``.
        :type time: float
        :return: Reference sample at ``time``.
        :rtype: TrajectoryReference
        :raises RuntimeError: If ``time`` is outside the valid range.
        """
        position, velocity, acceleration, yaw = self._native.evaluate_trajectory(float(time))
        return TrajectoryReference(
            position=np.asarray(position, dtype=float),
            velocity=np.asarray(velocity, dtype=float),
            acceleration=np.asarray(acceleration, dtype=float),
            yaw=float(yaw),
        )

    @property
    def min_time(self) -> float:
        """
        Minimum valid evaluation time of the generated trajectory.

        :return: Lower bound of the trajectory time domain, in seconds.
        :rtype: float
        """
        return float(self._native.get_min_time())

    @property
    def max_time(self) -> float:
        """
        Maximum valid evaluation time of the generated trajectory.

        :return: Upper bound of the trajectory time domain, in seconds.
        :rtype: float
        """
        return float(self._native.get_max_time())

    @property
    def was_trajectory_regenerated(self) -> bool:
        """
        Whether the trajectory has been regenerated since the last query.

        :return: True if a new optimization finished since the previous call.
        :rtype: bool
        """
        return bool(self._native.get_was_trajectory_regenerated())
