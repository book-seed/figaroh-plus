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

"""Fourier series trajectory optimization strategy.

Implements a fully symbolic CasADi NLP pipeline for D-optimal excitation
trajectory optimization using Fourier series parameterization.
"""

import logging
from typing import Optional, Dict, Any

import numpy as np

from .base_strategy import TrajectoryOptimizationStrategy

_FOURIER_DEFAULTS = {
    "n_harmonics": 5,
    "fourier_frequency": None,
    "n_samples": 200,
    "reg_lambda": 1.0e-6,
    "tanh_alpha_opt": 10,
    "tanh_alpha_id": 100,
}


class FourierOptimizationStrategy(TrajectoryOptimizationStrategy):
    """Fourier series trajectory optimization via CasADi symbolic NLP.

    Builds a fully symbolic MX/SX hybrid NLP for D-optimal excitation
    trajectory optimization. The outer MX layer handles optimization
    variables (Fourier coefficients) and automatic differentiation,
    while the inner SX layer evaluates dynamics symbolically with
    ``map("openmp")`` parallelization.
    """

    def __init__(self, fourier_config: Optional[Dict] = None):
        self.logger = logging.getLogger(__name__)
        self.logger.addHandler(logging.NullHandler())

        cfg = dict(_FOURIER_DEFAULTS)
        if fourier_config:
            cfg.update(fourier_config)
        self._fourier_config = cfg

    def name(self) -> str:
        return "fourier"

    def solve(self, context) -> None:
        """Build and solve the fully symbolic Fourier NLP.

        Args:
            context: BaseOptimalTrajectory instance with robot model,
                backend, and configuration.
        """
        import casadi as cs

        cfg = self._fourier_config
        n_harmonics = cfg["n_harmonics"]
        n_samples = cfg["n_samples"]
        reg_lambda = cfg["reg_lambda"]
        tanh_alpha_opt = cfg["tanh_alpha_opt"]
        tanh_alpha_id = cfg["tanh_alpha_id"]
        freq = cfg.get("fourier_frequency")

        cas_be = context._backend
        cas_be._ensure_symbolic_model()
        cmodel = cas_be._cmodel
        nq = cmodel.nq
        nv = cmodel.nv

        # ── 1. Problem dimensions ──────────────────────────────────
        n_act = len(context.active_joints)
        act_idxq = context.identif_config.get(
            "act_idxq", list(range(n_act))
        )
        act_idxv = context.identif_config.get(
            "act_idxv", list(range(n_act))
        )
        n_coeffs_per_joint = 1 + 2 * n_harmonics
        n_vars = n_act * n_coeffs_per_joint

        # ── 2. MX optimization variables ───────────────────────────
        Z = cs.MX.sym("coeffs", n_vars)
        # Reshape to (n_act, n_coeffs_per_joint) -- column-major layout
        Z_mat = cs.reshape(Z, n_act, n_coeffs_per_joint)

        # ── 3. Time vector ─────────────────────────────────────────
        T = 2 * np.pi
        omega = freq if freq is not None else 1.0
        t_vec = cs.MX.linspace(0, T, n_samples)
        t_vec = t_vec.T  # (1, n_samples)

        # ── 4. Column-major trajectory construction ────────────────
        # Q_col[j, i] = q_j(t_i): shape (n_act, n_samples) -- NO TRANSPOSE
        Q_col = cs.MX.zeros(n_act, n_samples)
        V_col = cs.MX.zeros(n_act, n_samples)
        A_col = cs.MX.zeros(n_act, n_samples)

        for j in range(n_act):
            a0 = Z_mat[j, 0]
            Q_col[j, :] = a0
            for k in range(1, n_harmonics + 1):
                ak = Z_mat[j, 2 * k - 1]
                bk = Z_mat[j, 2 * k]
                k_omega = k * omega

                sin_kwt = cs.sin(k_omega * t_vec)
                cos_kwt = cs.cos(k_omega * t_vec)

                Q_col[j, :] += ak * sin_kwt + bk * cos_kwt
                V_col[j, :] += ak * k_omega * cos_kwt - bk * k_omega * sin_kwt
                A_col[j, :] += (
                    -ak * k_omega**2 * sin_kwt
                    - bk * k_omega**2 * cos_kwt
                )

        # ── 5. Build full joint-space matrices (nq/nv, n_samples) ─
        Q_full = cs.MX.zeros(nq, n_samples)
        V_full = cs.MX.zeros(nv, n_samples)
        A_full = cs.MX.zeros(nv, n_samples)

        for j in range(n_act):
            Q_full[act_idxq[j], :] = Q_col[j, :]
            V_full[act_idxv[j], :] = V_col[j, :]
            A_full[act_idxv[j], :] = A_col[j, :]

        # ── 6. Regressor via map("openmp") ─────────────────────────
        W_fun = cas_be.regressor_function
        n_param_total = W_fun.size_out(0)[1]

        W_map_fun = W_fun.map(n_samples, "openmp")
        W_raw = W_map_fun(Q_full, V_full, A_full)  # (nv, Ns * n_param)

        # Stack per-sample blocks vertically: (Ns * nv, n_param)
        W_blocks = [
            W_raw[:, i * n_param_total : (i + 1) * n_param_total]
            for i in range(n_samples)
        ]
        W_full = cs.vertcat(*W_blocks)

        # ── 7. D-optimal objective via Cholesky ────────────────────
        idx_b = context.idx_b
        if len(idx_b) > 0:
            W_b = W_full[:, idx_b]
        else:
            W_b = W_full

        N_s = n_samples
        J = cs.mtimes(W_b.T, W_b) / N_s

        n_base = W_b.shape[1]
        J_reg = J + reg_lambda * cs.MX.eye(n_base)

        # D-optimal: obj = -log(det(J_reg))
        # L = cholesky(J_reg), det(J_reg) = prod(L_ii)^2
        # log(det(J_reg)) = 2 * sum(log(L_ii))
        # obj = -log(det(J_reg)) = -2 * sum(log(L_ii))
        L = cs.cholesky(J_reg)
        obj = -2 * cs.sum1(cs.log(cs.diag(L)))

        # ── 8. Torque constraints via map("openmp") with friction ──
        rnea_fun = cas_be.rnea_function
        rnea_map_fun = rnea_fun.map(n_samples, "openmp")
        tau_raw = rnea_map_fun(Q_full, V_full, A_full)  # (nv, Ns)

        # Add friction model: tau += fv*v + fs*tanh(alpha*v)
        has_friction = context.identif_config.get("has_friction", False)
        if has_friction:
            for j in range(n_act):
                jid = act_idxv[j]
                # Viscous friction (velocity-proportional, implicit unit coeff)
                tau_raw[jid, :] += V_full[jid, :]
                # Coulomb friction (tanh approximation, implicit unit coeff)
                tau_raw[jid, :] += cs.tanh(tanh_alpha_opt * V_full[jid, :])

        # Flatten torque to (Ns * nv,) -- constraint order: per-sample, all joints
        tau_flat = tau_raw.T.reshape(-1)  # (Ns * nv,)

        # ── 9. Build constraint vector ─────────────────────────────
        cons_list = []

        # Position constraints: per-sample, per-joint
        for i in range(n_samples):
            for j in range(n_act):
                cons_list.append(Q_col[j, i])

        # Velocity constraints: per-sample, per-joint
        for i in range(n_samples):
            for j in range(n_act):
                cons_list.append(V_col[j, i])

        # Torque constraints: per-sample, per-active-joint
        for i in range(n_samples):
            for j in range(n_act):
                cons_list.append(tau_flat[i * nv + act_idxv[j]])

        cons = cs.vertcat(*cons_list) if cons_list else cs.MX(0)

        # ── 10. Build constraint bounds ────────────────────────────
        model = context.robot.model
        q_lower = model.lowerPositionLimit
        q_upper = model.upperPositionLimit
        v_limit = model.velocityLimit
        tau_limit = model.effortLimit

        cl_list = []
        cu_list = []

        # Position bounds
        for _ in range(n_samples):
            for j in range(n_act):
                cl_list.append(float(q_lower[act_idxq[j]]))
                cu_list.append(float(q_upper[act_idxq[j]]))

        # Velocity bounds
        for _ in range(n_samples):
            for j in range(n_act):
                cl_list.append(float(-v_limit[act_idxv[j]]))
                cu_list.append(float(v_limit[act_idxv[j]]))

        # Torque bounds
        for _ in range(n_samples):
            for j in range(n_act):
                cl_list.append(float(-tau_limit[act_idxv[j]]))
                cu_list.append(float(tau_limit[act_idxv[j]]))

        cl = np.array(cl_list, dtype=float)
        cu = np.array(cu_list, dtype=float)

        # ── 11. NLP definition ────────────────────────────────────
        nlp = {"x": Z, "f": obj, "g": cons}

        # ── 12. Solver options ────────────────────────────────────
        opts = {
            "ipopt.linear_solver": "ma57",
            "ipopt.tol": 1e-6,
            "ipopt.max_iter": 500,
            "ipopt.mu_strategy": "adaptive",
            "ipopt.print_level": 3,
            "print_time": False,
        }

        # Check HSL availability -- fallback to mumps
        try:
            solver = cs.nlpsol("fourier_opt", "ipopt", nlp, opts)
        except Exception:
            self.logger.warning(
                "HSL ma57 not available, falling back to mumps"
            )
            opts["ipopt.linear_solver"] = "mumps"
            solver = cs.nlpsol("fourier_opt", "ipopt", nlp, opts)

        # ── 13. Coefficient initialization ─────────────────────────
        x0 = self._initialize_coefficients(context, n_act, n_harmonics)
        self.logger.info(
            "Initial Fourier coefficients: min=%f, max=%f",
            float(np.min(x0)), float(np.max(x0)),
        )

        # ── 14. Solve ──────────────────────────────────────────────
        result = solver(x0=x0, lbg=cl, ubg=cu)

        # ── 15. Extract results ────────────────────────────────────
        x_opt = np.array(result["x"]).flatten()
        Z_opt = x_opt.reshape(n_act, n_coeffs_per_joint)

        # Build optimal trajectory via numpy evaluation
        from figaroh.utils.fourier_trajectory import FourierTrajectory
        ft = FourierTrajectory(
            n_harmonics=n_harmonics, n_act=n_act,
            omega=omega, T=T,
        )

        t_np = np.linspace(0, T, n_samples)
        q_opt = ft.get_trajectory(t_np, Z_opt)
        v_opt = ft.get_velocity(t_np, Z_opt)
        a_opt = ft.get_acceleration(t_np, Z_opt)

        # ── 16. Populate results ───────────────────────────────────
        context.results['T_F'].append(t_np.reshape(-1, 1))
        context.results['P_F'].append(q_opt)
        context.results['V_F'].append(v_opt)
        context.results['A_F'].append(a_opt)

        stats = solver.stats()
        context.results['iteration_data'].append({
            "iterations": list(range(stats.get("iter_count", 0))),
            "obj_values": [float(result["f"])],
            "solve_time": stats.get("t_proc_cpu", {}).get("TOTAL", 0.0),
            "status": stats["return_status"],
        })
        context.results["final_regressor_shape"] = (
            W_b.shape[0], W_b.shape[1]
        )

        self.logger.info(
            "Fourier optimization complete: %d vars, %d cons, status=%s",
            n_vars, cons.size1(), stats["return_status"],
        )

    def _initialize_coefficients(
        self, context, n_act: int, n_harmonics: int
    ) -> np.ndarray:
        """Initialize Fourier coefficients from joint limits.

        Strategy:
        - a0: midpoint of joint range
        - ak, bk: small random amplitude (5% of joint range)

        Args:
            context: BaseOptimalTrajectory instance.
            n_act: Number of active joints.
            n_harmonics: Number of harmonics.

        Returns:
            Initial coefficient vector, shape (n_vars,).
        """
        n_coeffs = 1 + 2 * n_harmonics
        model = context.robot.model
        act_idxq = context.identif_config.get(
            "act_idxq", list(range(n_act))
        )

        q_upper = np.array([float(model.upperPositionLimit[j])
                            for j in act_idxq])
        q_lower = np.array([float(model.lowerPositionLimit[j])
                            for j in act_idxq])

        rng = np.random.default_rng(42)
        x0 = np.zeros(n_act * n_coeffs)

        for j in range(n_act):
            mid = (q_upper[j] + q_lower[j]) / 2
            x0[j * n_coeffs] = mid
            amp = 0.05 * (q_upper[j] - q_lower[j])
            for k in range(1, n_harmonics + 1):
                x0[j * n_coeffs + 2 * k - 1] = rng.uniform(-amp, amp)
                x0[j * n_coeffs + 2 * k] = rng.uniform(-amp, amp)

        # Constraint validation: shrink if violated
        max_retries = 5
        from figaroh.utils.fourier_trajectory import FourierTrajectory
        ft = FourierTrajectory(
            n_harmonics=n_harmonics, n_act=n_act,
        )

        for retry in range(max_retries):
            Z_init = x0.reshape(n_act, n_coeffs)
            t_np = np.linspace(0, 2 * np.pi, 100)
            q_init = ft.get_trajectory(t_np, Z_init)
            v_init = ft.get_velocity(t_np, Z_init)

            violated = False
            for i in range(t_np.shape[0]):
                for j in range(n_act):
                    if q_init[i, j] > q_upper[j] or \
                            q_init[i, j] < q_lower[j]:
                        violated = True
                    if abs(v_init[i, j]) > model.velocityLimit[
                            act_idxq[j]]:
                        violated = True

            if not violated:
                break

            # Shrink amplitude by half and retry
            for j in range(n_act):
                for k in range(1, n_harmonics + 1):
                    idx_a = j * n_coeffs + 2 * k - 1
                    idx_b = j * n_coeffs + 2 * k
                    x0[idx_a] *= 0.5
                    x0[idx_b] *= 0.5

        return x0
