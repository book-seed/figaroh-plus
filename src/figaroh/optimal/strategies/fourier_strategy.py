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
trajectory optimization using velocity-parameterized Fourier series.

Velocity (not position) is parameterized to avoid (kω)² amplification of
acceleration at high harmonics, yielding better numerical conditioning.
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
            context: BaseOptimalTrajectory instance with robot model and configuration.
        """
        try:
            import casadi as cs
            from figaroh.backend.casadi import CasadiBackend
        except ImportError as e:
            raise ImportError(
                "FourierOptimizationStrategy requires CasADi. "
                "Install with: pixi add casadi"
            ) from e

        cfg = self._fourier_config
        n_harmonics = cfg["n_harmonics"]
        Ns = cfg["n_samples"]
        reg_lambda = cfg["reg_lambda"]
        omega = cfg.get("fourier_frequency")

        # CasadiBackend is owned by the Fourier strategy (the sole consumer
        # of the symbolic model). ensure_initialized() triggers the lazy
        # build once; subsequent property accesses hit the idempotent guard.
        cas_be = CasadiBackend(robot=context.robot)
        cas_be.ensure_initialized()
        nq = cas_be.nq
        nv = cas_be.nv

        # ── 1. Problem dimensions ──────────────────────────────────
        n_act = len(context.active_joints)
        act_idxq = context.identif_config.get("act_idxq", list(range(n_act)))
        act_idxv = context.identif_config.get("act_idxv", list(range(n_act)))
        n_coeffs_per_joint = 1 + 2 * n_harmonics
        n_vars = n_act * n_coeffs_per_joint

        # ── 2. MX optimization variables ───────────────────────────
        Z = cs.MX.sym("coeffs", n_vars)
        # Reshape to (n_act, n_coeffs_per_joint) -- column-major layout
        Z_mat = cs.reshape(Z, n_act, n_coeffs_per_joint)

        # ── 3. Time vector ─────────────────────────────────────────
        T = 2 * np.pi / omega
        t_np = np.linspace(0, T, Ns)
        t_vec = cs.MX(t_np).T  # (1, Ns)

        # ── 4. Velocity-parameterized trajectory construction ─────
        # Optimization variables per joint j:
        #   Z_mat[j, 0]          = a0 (mean position)
        #   Z_mat[j, 2k-1], Z_mat[j, 2k] = ak, bk (velocity amplitudes)
        from figaroh.utils.fourier_trajectory import FourierTrajectory

        ft = FourierTrajectory(n_harmonics=n_harmonics, n_act=n_act, omega=omega)
        Q_col, V_col, A_col = ft.build_mx_trajectory(t_vec, Z_mat)

        # ── 5. Build full joint-space matrices (nq/nv, Ns) ─
        Q_full = cs.MX.zeros(nq, Ns)
        V_full = cs.MX.zeros(nv, Ns)
        A_full = cs.MX.zeros(nv, Ns)

        for j in range(n_act):
            Q_full[act_idxq[j], :] = Q_col[j, :]
            V_full[act_idxv[j], :] = V_col[j, :]
            A_full[act_idxv[j], :] = A_col[j, :]

        # ── 6. Regressor via map("openmp") ─────────────────────────
        W_fun = cas_be.regressor_function

        # 从 CasADi 的 Function 对象中提取第 0 个输出矩阵的"列数"，
        # 即机器人动力学回归矩阵的总参数个数
        n_param_total = W_fun.size_out(0)[1]

        # 将"单样本函数" W_fun 映射为"多样本函数"，以便在 Ns 个时间点上并行计算回归矩阵
        # W_func 计算单个时间步 t_k 的回归矩阵
        W_map_fun = W_fun.map(Ns, "openmp")

        # 输入：形状为(nv, Ns)的关节位置、速度和加速度矩阵
        # 输出：形状为(nv, Ns * n_param_total)的回归矩阵，其中每个样本的回归矩阵按列堆叠
        # 第一列 q1 (对应t1时刻)，传给 W_func，计算出 W1
        # 第二列 q2 (对应t2时刻)，传给 W_func，计算出 W2
        # ...
        # 第 Ns 列 qn (对应tn时刻)，传给 W_func，计算出 Wn
        # W_raw = [W1, W2, ..., Wn]，形状为(nv, Ns * n_param_total)
        W_raw = W_map_fun(Q_full, V_full, A_full)

        # Stack per-sample blocks vertically: (Ns * nv, n_param)
        #                         [W1]
        #                         [W2]
        # 转化[W1, W2, ..., Wn]为  ...
        #                         [W(n-1)]
        #                         [Wn]
        W_blocks = cs.horzsplit(W_raw, n_param_total)
        W_full = cs.vertcat(*W_blocks)

        # ── 7. D-optimal objective via Cholesky ────────────────────
        idx_b = context.idx_b
        if idx_b is not None and len(idx_b) > 0:
            W_b = W_full[:, list(idx_b)]
        else:
            W_b = W_full

        # Q1：为什么要计算 J = W_b.T * W_b 呢？
        # 在动力学辨识中，我们通过最小二乘从实测关节力矩 tau 中求解基参数 \theta_b_hat:
        # tau = W_b * theta_b_hat + noise
        # 根据最小二乘的标准公式，基参数估计值 \theta_b_hat 的解为：
        # theta_b_hat = (W_b^T * W_b)^(-1) * W_b^T * tau
        # 其中 (W_b^T * W_b) 就是 Fisher 信息矩阵。他的数值特性直接决定了辨识结果的成败：
        # 1. 决定估计参数的方差（不确定度/噪声敏感度）。
        #    基参数估计值的协方差矩阵为：Cov(theta_b_hat) = sigma^2 * (W_b^T * W_b)^(-1) = sigma^2 / Ns * J^(-1)
        #    矩阵J越大， J^(-1)越小，辨识出来的动力学参数的方差就越小，越精确，对抗传感器噪声的能力就越强。
        # 2. 决定矩阵的满秩与可逆性。
        #    如果轨迹激励不充分（比如关节根本不动，或者运动过于单一），W_b的某些列会线性相关，导致 W_b^T * W_b 奇异不可逆，
        #    求解器报错或辨识出极度荒谬的物理参数。

        # Q2：为什么一定要除以Ns呢？
        # 1. 消除采样频率/点数对目标函数的影响：
        #    如果不除以Ns，采样点越多，W_b^T * W_b 里的每个元素数值就会自然翻倍。
        #    除以Ns后，J代表的是平均每个采样点所提供的平均信息量。
        # 2. 便于跨轨迹比较与优化收敛：
        #    无论采样周期设为100Hz还是1000Hz，J的数量级都保持在一个稳定的物理量级。
        #    这能防止优化求解器在改变采样密度时出现梯度数量级巨幅波动的问题。
        J = cs.mtimes(W_b.T, W_b) / Ns

        # Tikhonov正则化
        n_base = W_b.shape[1]
        J_reg = J + reg_lambda * cs.DM.eye(n_base)

        # D-optimal: obj = -log(det(J_reg)) = -2*sum(log(diag(chol(J_reg)))).
        # chol() is DM/SX-only (not MX); wrap the chol-based logdet in an SX
        # Function and call it from the MX graph. CasADi differentiates through
        # the SX Function automatically — same MX/SX hybrid pattern already
        # used for the regressor above (W_fun/rnea_fun are SX, called via .map).

        # D-最优轨迹是机器人动力学参数辨识中，用于最大化提取关节惯性参数信息，最小化辨识不确定性的理想运动轨迹。
        # D-最优准则的目标是最大化fisher信息矩阵J的行列式det(J)，等价于最小化-log(det(J))。
        # 恒等式推导:
        # J_reg = L·L^T (Cholesky分解，L为下三角矩阵)
        # det(J_reg) = det(L·L^T) = det(L)·det(L^T) = det(L)^2
        # 三角矩阵行列式 = 对角线元素乘积:
        # L = [[L11, 0, …, 0], [L21, L22, …, 0], …, [Ln1, Ln2, …, Lnn]]
        # 按第一列Laplace展开，仅L11非零 → 递归每次只剩对角线元素
        # ∴ det(L) = L11·L22·…·Lnn = Π diag(L)
        # ∴ log(det(L)) = log(Π Lii) = Σ log(Lii) = Σ log(diag(L))
        # ∴ log(det(J_reg)) = 2·log(det(L)) = 2·Σ log(diag(L))
        # ∴ -log(det(J_reg)) = -2·Σ log(diag(L))

        J_sx = cs.SX.sym("J_reg", n_base, n_base)
        L_sx = cs.chol(J_sx)
        logdet_sx = 2 * cs.sum1(cs.log(cs.diag(L_sx)))
        logdet_fn = cs.Function("logdet", [J_sx], [logdet_sx])

        # 强制保证数值对称性，防止 cs.chol 偶尔报 Not Symmetric 错误
        J_sym = 0.5 * (J_reg + J_reg.T)
        obj = -logdet_fn(J_sym)

        # ── 8. Torque constraints via map("openmp") ──
        rnea_fun = cas_be.rnea_function
        rnea_map_fun = rnea_fun.map(Ns, "openmp")
        tau_raw = rnea_map_fun(Q_full, V_full, A_full)  # (nv, Ns)

        # TODO: Friction model (fv*v + fs*tanh(alpha*v)) has been removed.
        # Friction identification is planned as a separate step before
        # full dynamics identification.

        # ── 9. Build constraint vector ─────────────────────────────
        cons_list = []

        # Position constraints: per-sample, per-joint
        for i in range(Ns):
            for j in range(n_act):
                cons_list.append(Q_col[j, i])

        # Velocity constraints: per-sample, per-joint
        for i in range(Ns):
            for j in range(n_act):
                cons_list.append(V_col[j, i])

        # Torque constraints: per-sample, per-active-joint
        for i in range(Ns):
            for j in range(n_act):
                cons_list.append(tau_raw[act_idxv[j], i])

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
        for _ in range(Ns):
            for j in range(n_act):
                cl_list.append(float(q_lower[act_idxq[j]]))
                cu_list.append(float(q_upper[act_idxq[j]]))

        # Velocity bounds
        for _ in range(Ns):
            for j in range(n_act):
                cl_list.append(float(-v_limit[act_idxv[j]]))
                cu_list.append(float(v_limit[act_idxv[j]]))

        # Torque bounds
        for _ in range(Ns):
            for j in range(n_act):
                cl_list.append(float(-tau_limit[act_idxv[j]]))
                cu_list.append(float(tau_limit[act_idxv[j]]))

        cl = np.array(cl_list, dtype=float)
        cu = np.array(cu_list, dtype=float)

        # ── 11. NLP definition ────────────────────────────────────
        nlp = {"x": Z, "f": obj, "g": cons}

        # ── 12. Solver options (MUMPS-optimized) ────────────────────
        opts = {
            "ipopt.linear_solver": "mumps",
            "ipopt.hessian_approximation": "limited-memory",
            "ipopt.tol": 1e-5,
            "ipopt.acceptable_tol": 1e-4,
            "ipopt.max_iter": 500,
            "ipopt.mu_strategy": "adaptive",
            "ipopt.print_level": 3,
            "print_time": False,
        }

        solver = cs.nlpsol("fourier_opt", "ipopt", nlp, opts)

        # ── 13. Coefficient initialization ─────────────────────────
        x0 = self._initialize_coefficients(context, n_act, n_harmonics, omega)
        self.logger.info("Initial Fourier coefficients: min=%f, max=%f", float(np.min(x0)), float(np.max(x0)))

        # ── 14. Solve ──────────────────────────────────────────────
        result = solver(x0=x0, lbg=cl, ubg=cu)

        # ── 15. Extract optimal coefficients ──────────────────────
        x_opt = np.array(result["x"]).flatten()
        Z_opt = x_opt.reshape(n_act, n_coeffs_per_joint)

        # ── 16. Evaluate optimal trajectory (NumPy) ────────────────
        t_np = np.linspace(0, T, Ns)
        q_opt = ft.get_trajectory(t_np, Z_opt)
        v_opt = ft.get_velocity(t_np, Z_opt)
        a_opt = ft.get_acceleration(t_np, Z_opt)

        # ── 17. Post-optimization diagnostics ────────────────────
        W_b_diag = context._stack_base_regressors(q_opt, v_opt, a_opt)
        diagnostics = context._compute_regressor_diagnostics(
            W_b_diag, n_samples=Ns
        )

        fim = diagnostics["fim_eigenvalues"]
        self.logger.info(
            "Fourier diagnostics: κ=%.2f, d_opt=%.2f, "
            "λ_min=%.2e, λ_max=%.2e, λ_ratio=%.2e, n_base=%d",
            diagnostics["condition_number"],
            diagnostics["d_optimal_objective"],
            fim["min"],
            fim["max"],
            fim["ratio"],
            W_b_diag.shape[1],
        )

        # ── 18. Populate results ───────────────────────────────────
        # Store Fourier coefficients as the primary result.  The
        # analytic trajectory can be reconstructed at arbitrary time
        # resolution via FourierTrajectory without storing sampled
        # P_F/V_F/A_F arrays.
        context.results['diagnostics'] = diagnostics
        context.results['fourier_coeffs'] = Z_opt
        context.results['omega'] = omega
        context.results['n_harmonics'] = n_harmonics

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

    def _initialize_coefficients(self, context, n_act: int, n_harmonics: int, omega: float) -> np.ndarray:
        """Initialize Fourier coefficients from joint/velocity limits.

        Velocity-parameterized initialization:
        - a0: midpoint of joint position range (mean position).
        - ak, bk: small random velocity amplitude (5% of velocity limit
          divided by n_harmonics to account for harmonic accumulation).

        Args:
            context: BaseOptimalTrajectory instance.
            n_act: Number of active joints.
            n_harmonics: Number of harmonics.
            omega: Fundamental frequency (rad/s).

        Returns:
            Initial coefficient vector, shape (n_vars,).
        """
        n_coeffs = 1 + 2 * n_harmonics
        model = context.robot.model
        act_idxq = context.identif_config.get("act_idxq", list(range(n_act)))
        act_idxv = context.identif_config.get("act_idxv", list(range(n_act)))

        q_upper = np.array([float(model.upperPositionLimit[j]) for j in act_idxq])
        q_lower = np.array([float(model.lowerPositionLimit[j]) for j in act_idxq])
        v_limit = np.array([float(model.velocityLimit[j]) for j in act_idxv])

        # Random initialization with fixed seed for reproducibility
        rng = np.random.default_rng(42)
        x0 = np.zeros(n_act * n_coeffs)

        for j in range(n_act):
            mid = (q_upper[j] + q_lower[j]) / 2
            x0[j * n_coeffs] = mid
            # Velocity amplitude: 5% of velocity limit, divided by
            # n_harmonics so the sum of all harmonics stays within a
            # reasonable fraction of the velocity limit.
            amp = 0.05 * v_limit[j] / n_harmonics
            for k in range(1, n_harmonics + 1):
                x0[j * n_coeffs + 2 * k - 1] = rng.uniform(-amp, amp)
                x0[j * n_coeffs + 2 * k] = rng.uniform(-amp, amp)

        # Constraint validation: shrink if violated
        max_retries = 5
        from figaroh.utils.fourier_trajectory import FourierTrajectory
        ft = FourierTrajectory(n_harmonics=n_harmonics, n_act=n_act, omega=omega)

        T_check = 2 * np.pi / omega
        for retry in range(max_retries):
            Z_init = x0.reshape(n_act, n_coeffs)
            t_np = np.linspace(0, T_check, 100)
            q_init = ft.get_trajectory(t_np, Z_init)
            v_init = ft.get_velocity(t_np, Z_init)
            a_init = ft.get_acceleration(t_np, Z_init)
            tau_init = ft.compute_torques(q_init, v_init, a_init, context.robot)

            violated = ft.check_constraints(
                q_init, v_init, tau=tau_init, robot=context.robot,
                act_idxq=act_idxq, act_idxv=act_idxv,
            )

            if not violated:
                break

            # Shrink velocity amplitude by half and retry
            for j in range(n_act):
                for k in range(1, n_harmonics + 1):
                    idx_a = j * n_coeffs + 2 * k - 1
                    idx_b = j * n_coeffs + 2 * k
                    x0[idx_a] *= 0.5
                    x0[idx_b] *= 0.5

        return x0
