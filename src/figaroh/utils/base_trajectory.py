# Copyright [2021-2025] Thanh Nguyen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Abstract base class for trajectory generation."""

from abc import ABC, abstractmethod
import numpy as np


class BaseTrajectory(ABC):
    """Abstract trajectory generator.

    Defines the interface for generating trajectory position, velocity, and
    acceleration from parameterized coefficients. Concrete implementations
    include CubicSplineTrajectory (ndcurves cubic splines) and
    FourierTrajectory (Fourier series).
    """

    @abstractmethod
    def get_trajectory(self, t: np.ndarray, coeffs: np.ndarray) -> np.ndarray:
        """Evaluate position q(t) at time points t.

        Args:
            t: Time points, shape (N,).
            coeffs: Trajectory coefficients, shape (n_param, ...).

        Returns:
            Position trajectory, shape (N, nq).
        """
        ...

    @abstractmethod
    def get_velocity(self, t: np.ndarray, coeffs: np.ndarray) -> np.ndarray:
        """Evaluate velocity v(t) = dq/dt at time points t.

        Args:
            t: Time points, shape (N,).
            coeffs: Trajectory coefficients, shape (n_param, ...).

        Returns:
            Velocity trajectory, shape (N, nv).
        """
        ...

    @abstractmethod
    def get_acceleration(self, t: np.ndarray, coeffs: np.ndarray) -> np.ndarray:
        """Evaluate acceleration a(t) = d2q/dt2 at time points t.

        Args:
            t: Time points, shape (N,).
            coeffs: Trajectory coefficients, shape (n_param, ...).

        Returns:
            Acceleration trajectory, shape (N, nv).
        """
        ...

    @abstractmethod
    def compute_torques(
        self, q: np.ndarray, v: np.ndarray, a: np.ndarray, robot
    ) -> np.ndarray:
        """Compute joint torques from trajectory.

        Args:
            q: Position trajectory, shape (N, nq).
            v: Velocity trajectory, shape (N, nv).
            a: Acceleration trajectory, shape (N, nv).
            robot: RobotWrapper instance.

        Returns:
            Joint torques, shape (N, nv).
        """
        ...

    @abstractmethod
    def check_constraints(
        self,
        q: np.ndarray,
        v: np.ndarray,
        tau: np.ndarray,
        robot,
    ) -> bool:
        """Check if trajectory violates joint constraints.

        Args:
            q: Position trajectory, shape (N, nq).
            v: Velocity trajectory, shape (N, nv).
            tau: Joint torques, shape (N, nv).
            robot: RobotWrapper instance.

        Returns:
            True if any constraint is violated.
        """
        ...
