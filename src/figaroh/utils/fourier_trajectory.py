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

"""Fourier series trajectory generation."""

from typing import Optional
import numpy as np

from .base_trajectory import BaseTrajectory


class FourierTrajectory(BaseTrajectory):
    """Fourier series parameterized trajectory.

    Generates smooth periodic trajectories as sums of harmonic sinusoids:

        q_j(t) = a0_j + Σ_{k=1}^{N} [ a_k_j * sin(k*ω*t) + b_k_j * cos(k*ω*t) ]

    where ω = 2π/T is the fundamental frequency.

    Args:
        n_harmonics: Number of harmonic terms (N).
        n_act: Number of active joints.
        omega: Fundamental frequency (rad/s). If None, computed as 2π/T.
        T: Period (s). Default 2π.
    """

    def __init__(
        self,
        n_harmonics: int = 5,
        n_act: int = 0,
        omega: Optional[float] = None,
        T: float = 2 * np.pi,
    ):
        self._n_harmonics = n_harmonics
        self._n_act = n_act
        self._T = T
        self._omega = omega if omega is not None else 2 * np.pi / T

    @property
    def n_harmonics(self) -> int:
        return self._n_harmonics

    @property
    def n_coeffs_per_joint(self) -> int:
        """Number of Fourier coefficients per joint: 1 (a0) + 2 * N."""
        return 1 + 2 * self._n_harmonics

    @property
    def omega(self) -> float:
        return self._omega

    # ── NumPy evaluation (for numerical backend) ───────────────────

    def _evaluate(self, t: np.ndarray, coeffs: np.ndarray) -> tuple:
        """Evaluate position, velocity, acceleration at time points.

        Args:
            t: Time points, shape (N,).
            coeffs: Fourier coefficients, shape (n_act, 2*n_harmonics + 1).

        Returns:
            Tuple (q, v, a) each shape (N, n_act).
        """
        N = len(t)
        n_act = coeffs.shape[0]
        n_h = self._n_harmonics

        q = np.zeros((N, n_act))
        v = np.zeros((N, n_act))
        a = np.zeros((N, n_act))

        for j in range(n_act):
            a0 = coeffs[j, 0]
            q[:, j] = a0
            v[:, j] = 0.0
            a[:, j] = 0.0

            for k in range(1, n_h + 1):
                ak = coeffs[j, 2 * k - 1]
                bk = coeffs[j, 2 * k]
                k_omega = k * self._omega

                sin_kwt = np.sin(k_omega * t)
                cos_kwt = np.cos(k_omega * t)

                # q = ak*sin(kωt) + bk*cos(kωt)
                q[:, j] += ak * sin_kwt + bk * cos_kwt
                # v = ak*kω*cos(kωt) - bk*kω*sin(kωt)
                v[:, j] += ak * k_omega * cos_kwt - bk * k_omega * sin_kwt
                # a = -ak*(kω)^2*sin(kωt) - bk*(kω)^2*cos(kωt)
                a[:, j] += -ak * k_omega**2 * sin_kwt - bk * k_omega**2 * cos_kwt

        return q, v, a

    def get_trajectory(self, t: np.ndarray, coeffs: np.ndarray) -> np.ndarray:
        q, _, _ = self._evaluate(t, coeffs)
        return q

    def get_velocity(self, t: np.ndarray, coeffs: np.ndarray) -> np.ndarray:
        _, v, _ = self._evaluate(t, coeffs)
        return v

    def get_acceleration(self, t: np.ndarray, coeffs: np.ndarray) -> np.ndarray:
        _, _, a = self._evaluate(t, coeffs)
        return a

    def compute_torques(
        self, q: np.ndarray, v: np.ndarray, a: np.ndarray, robot
    ) -> np.ndarray:
        """Compute joint torques via pinocchio rnea."""
        import pinocchio

        N = q.shape[0]
        nv = robot.model.nv
        tau = np.zeros((N, nv))
        for i in range(N):
            tau[i, :] = pinocchio.rnea(
                robot.model, robot.data, q[i, :], v[i, :], a[i, :]
            )
        return tau

    def check_constraints(
        self, q: np.ndarray, v: np.ndarray, tau: np.ndarray, robot
    ) -> bool:
        """Check if trajectory violates joint position/velocity/effort limits."""
        model = robot.model
        violated = False

        for i in range(q.shape[0]):
            for j in range(q.shape[1]):
                if q[i, j] > model.upperPositionLimit[j] or \
                   q[i, j] < model.lowerPositionLimit[j]:
                    violated = True

        for i in range(v.shape[0]):
            for j in range(v.shape[1]):
                if abs(v[i, j]) > model.velocityLimit[j]:
                    violated = True

        for i in range(tau.shape[0]):
            for j in range(tau.shape[1]):
                if abs(tau[i, j]) > model.effortLimit[j]:
                    violated = True

        return violated

    # ── CasADi SX expression (for symbolic NLP) ───────────────────

    def build_casadi_expression(self, t_sym, coeffs_sym, omega=None):
        """Build CasADi SX expression for q(t, coeffs).

        Args:
            t_sym: CasADi SX symbol for time (scalar).
            coeffs_sym: CasADi SX symbol for coefficients, shape (n_act, 2*n_harmonics+1).
            omega: Fundamental frequency (optional, uses self._omega if None).

        Returns:
            CasADi SX expression for q(t, coeffs), shape (n_act, 1).
        """
        import casadi as cs

        n_act = coeffs_sym.shape[0]
        n_h = self._n_harmonics
        omega_val = omega if omega is not None else self._omega

        q_expr = cs.SX.zeros(n_act, 1)
        for j in range(n_act):
            q_j = coeffs_sym[j, 0]  # a0
            for k in range(1, n_h + 1):
                ak = coeffs_sym[j, 2 * k - 1]
                bk = coeffs_sym[j, 2 * k]
                k_omega = k * omega_val
                q_j += ak * cs.sin(k_omega * t_sym) + bk * cs.cos(k_omega * t_sym)
            q_expr[j] = q_j
        return q_expr
