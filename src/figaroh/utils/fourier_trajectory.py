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

import numpy as np
from .base_trajectory import BaseTrajectory


class FourierTrajectory(BaseTrajectory):
    """Fourier series parameterized trajectory (velocity parameterization).

    Velocity is parameterized as a truncated Fourier series, and position is obtained by analytical integration.  
    This avoids the (kω)² amplification of acceleration that plagues direct position parameterization, yielding 
    better numerical conditioning for high-harmonic trajectories.

    v_j(t) = Σ_{k=1}^{N} [ a_k_j * sin(k*ω*t) + b_k_j * cos(k*ω*t) ]

    q_j(t) = a0_j + Σ_{k=1}^{N} [ -a_k_j/(kω) * cos(k*ω*t) + b_k_j/(kω) * sin(k*ω*t) ]

    a_j(t) = Σ_{k=1}^{N} [ a_k_j * kω * cos(k*ω*t) - b_k_j * kω * sin(k*ω*t) ]

    where ω = 2π/T is the fundamental frequency, a0_j is the mean position,
    and DC velocity is constrained to zero for periodicity.

    Args:
        n_harmonics: Number of harmonic terms (N).
        n_act: Number of active joints.
        omega: Fundamental frequency (rad/s). Default 1.0.
    """

    def __init__( self, n_harmonics: int = 5, n_act: int = 0, omega: float = 1.0):
        self._n_harmonics = n_harmonics
        self._n_act = n_act
        self._omega = omega

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

        Velocity parameterization: coeffs encode velocity harmonics. Position 
        is obtained by analytical integration; acceleration by differentiation.

        Args:
            t: Time points, shape (N,).
            coeffs: Fourier coefficients, shape (n_act, 2*n_harmonics + 1).
                    Column 0: a0 (mean position).
                    Columns 2k-1, 2k: ak, bk (velocity sin/cos amplitudes).

        Returns:
            Tuple (q, v, a) each shape (N, n_act).
        """
        N = len(t)
        n_act = coeffs.shape[0]
        n_h = self._n_harmonics
        omega = self._omega

        q = np.zeros((N, n_act))
        v = np.zeros((N, n_act))
        a = np.zeros((N, n_act))

        for j in range(n_act):
            mean_pos = coeffs[j, 0]
            q[:, j] = mean_pos

            for k in range(1, n_h + 1):
                ak = coeffs[j, 2 * k - 1]  # velocity sin amplitude
                bk = coeffs[j, 2 * k]      # velocity cos amplitude
                k_omega = k * omega

                sin_kwt = np.sin(k_omega * t)
                cos_kwt = np.cos(k_omega * t)

                # v(t) = ak*sin(kωt) + bk*cos(kωt)
                v[:, j] += ak * sin_kwt + bk * cos_kwt
                # q(t) = mean_pos + ∫v dt = mean_pos - ak/(kω)*cos + bk/(kω)*sin
                q[:, j] += -ak / k_omega * cos_kwt + bk / k_omega * sin_kwt
                # a(t) = dv/dt = ak*kω*cos - bk*kω*sin
                a[:, j] += ak * k_omega * cos_kwt - bk * k_omega * sin_kwt

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

    def compute_torques(self, q: np.ndarray, v: np.ndarray, a: np.ndarray, robot) -> np.ndarray:
        """Compute joint torques via pinocchio rnea."""
        import pinocchio

        N = q.shape[0]
        nv = robot.model.nv
        tau = np.zeros((N, nv))
        for i in range(N):
            tau[i, :] = pinocchio.rnea(robot.model, robot.data, q[i, :], v[i, :], a[i, :])
        return tau

    def check_constraints(
        self,
        q: np.ndarray,
        v: np.ndarray,
        tau: np.ndarray | None = None,
        robot=None,
        act_idxq: list | None = None,
        act_idxv: list | None = None,
    ) -> bool:
        """Check if trajectory violates joint position/velocity/effort limits.

        Args:
            q: Position trajectory, shape (N, n_act).
            v: Velocity trajectory, shape (N, n_act).
            tau: Joint torques, shape (N, n_act) or None to skip torque check.
            robot: RobotWrapper instance with ``model`` attribute.
            act_idxq: Active joint indices for position limits.  When None
                (default), uses ``range(q.shape[1])`` — correct only when
                active joints are [0, 1, …, n_act-1].
            act_idxv: Active joint indices for velocity/effort limits.  When
                None (default), uses ``range(v.shape[1])`` with the same
                contingency as ``act_idxq``.

        Returns:
            True if any constraint is violated.
        """
        model = robot.model
        idxq = act_idxq if act_idxq is not None else list(range(q.shape[1]))
        idxv = act_idxv if act_idxv is not None else list(range(v.shape[1]))

        q_upper = np.array([float(model.upperPositionLimit[j]) for j in idxq])
        q_lower = np.array([float(model.lowerPositionLimit[j]) for j in idxq])
        v_limit = np.array([float(model.velocityLimit[j]) for j in idxv])

        violated = False

        for i in range(q.shape[0]):
            for col, _ in enumerate(idxq):
                if q[i, col] > q_upper[col] or q[i, col] < q_lower[col]:
                    violated = True

        for i in range(v.shape[0]):
            for col, _ in enumerate(idxv):
                if abs(v[i, col]) > v_limit[col]:
                    violated = True

        if tau is not None:
            tau_limit = np.array([float(model.effortLimit[j]) for j in idxv])
            for i in range(tau.shape[0]):
                for col, _ in enumerate(idxv):
                    if abs(tau[i, col]) > tau_limit[col]:
                        violated = True

        return violated

    # ── CasADi MX expression (for symbolic NLP) ───────────────────

    def build_mx_trajectory(self, t_vec, coeffs_mat):
        """Build CasADi MX expressions for q(t), v(t), a(t) at all time points.

        Velocity-parameterized Fourier series evaluated symbolically for use
        in optimization NLP construction.

        Args:
            t_vec: CasADi MX expression for time, shape (1, Ns).
            coeffs_mat: CasADi MX symbol for coefficients,
                        shape (n_act, 2*n_harmonics+1).
                        Column 0: a0 (mean position).
                        Columns 2k-1, 2k: ak, bk (velocity sin/cos amplitudes).

        Returns:
            Tuple (Q, V, A) each shape (n_act, Ns) as CasADi MX expressions.
        """
        import casadi as cs

        n_act = coeffs_mat.shape[0]
        Ns = t_vec.shape[1]

        Q = cs.MX.zeros(n_act, Ns)
        V = cs.MX.zeros(n_act, Ns)
        A = cs.MX.zeros(n_act, Ns)

        for j in range(n_act):
            a0 = coeffs_mat[j, 0]  # mean position
            Q[j, :] = a0
            for k in range(1, self._n_harmonics + 1):
                ak = coeffs_mat[j, 2 * k - 1]  # velocity sin amplitude
                bk = coeffs_mat[j, 2 * k]      # velocity cos amplitude
                k_omega = k * self._omega

                sin_kwt = cs.sin(k_omega * t_vec)
                cos_kwt = cs.cos(k_omega * t_vec)

                # v(t) = ak*sin(kωt) + bk*cos(kωt)
                V[j, :] += ak * sin_kwt + bk * cos_kwt
                # q(t) = a0 - ak/(kω)*cos + bk/(kω)*sin
                Q[j, :] += -ak / k_omega * cos_kwt + bk / k_omega * sin_kwt
                # a(t) = ak*kω*cos - bk*kω*sin
                A[j, :] += ak * k_omega * cos_kwt - bk * k_omega * sin_kwt

        return Q, V, A
