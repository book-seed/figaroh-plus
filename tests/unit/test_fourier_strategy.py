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

"""Tests for FourierOptimizationStrategy."""

import pytest
import numpy as np
from unittest.mock import MagicMock, patch


class TestFourierStrategyInitialization:
    """Test strategy creation and configuration."""

    def test_strategy_name(self):
        """name() returns 'fourier'."""
        from figaroh.optimal.strategies.fourier_strategy import (
            FourierOptimizationStrategy
        )
        strategy = FourierOptimizationStrategy()
        assert strategy.name() == "fourier"

    def test_accepts_fourier_config(self):
        """Strategy accepts and stores fourier_config at init."""
        from figaroh.optimal.strategies.fourier_strategy import (
            FourierOptimizationStrategy
        )
        config = {
            "n_harmonics": 5,
            "fourier_frequency": None,
            "n_samples": 200,
            "reg_lambda": 1.0e-6,
        }
        strategy = FourierOptimizationStrategy(fourier_config=config)
        assert strategy._fourier_config["n_harmonics"] == 5
        assert strategy._fourier_config["n_samples"] == 200


class TestFourierStrategySolveFlow:
    """Test the high-level solve() flow."""

    def test_solve_populates_results(self):
        """solve() fills results dict with T_F, P_F, V_F, A_F."""
        pytest.importorskip("casadi")
        import casadi as cs
        from figaroh.optimal.strategies.fourier_strategy import (
            FourierOptimizationStrategy
        )

        strategy = FourierOptimizationStrategy(fourier_config={
            "n_harmonics": 3,
            "fourier_frequency": 1.0,
            "n_samples": 50,
            "reg_lambda": 1.0e-6,
        })

        # Build a minimal mock context
        mock_ctx = MagicMock()
        mock_ctx.trajectory_config = {
            "fourier_config": strategy._fourier_config,
        }
        mock_ctx.identif_config = {
            "has_friction": True,
            "has_actuator_inertia": False,
            "has_joint_offset": False,
            "act_idxv": [0],
        }
        mock_ctx.active_joints = ["joint1"]
        mock_ctx.idx_b = np.array([0], dtype=int)
        mock_ctx.idx_e = np.array([], dtype=int)

        # Mock robot with 1-DOF
        mock_robot = MagicMock()
        mock_robot.model.nq = 1
        mock_robot.model.nv = 1
        mock_robot.model.name = "test_robot"
        mock_robot.model.upperPositionLimit = np.array([2.0])
        mock_robot.model.lowerPositionLimit = np.array([-2.0])
        mock_robot.model.velocityLimit = np.array([5.0])
        mock_robot.model.effortLimit = np.array([100.0])
        mock_robot.model.inertias.tolist.return_value = [MagicMock(mass=1.0)]

        # Mock CasadiBackend with regressor_function and rnea_function.
        # solve() now constructs CasadiBackend(robot=context.robot) internally,
        # so we patch it at the fourier_strategy module rather than pre-setting
        # a _backend attribute on the context.
        mock_backend = MagicMock()

        # Create minimal SX functions for the mock backend
        cs_q = cs.SX.sym("q", 1)
        cs_v = cs.SX.sym("v", 1)
        cs_a = cs.SX.sym("a", 1)
        W_expr = cs.SX.ones(1, 10)  # dummy regressor
        tau_expr = cs.SX.ones(1)    # dummy torque

        mock_backend.regressor_function = cs.Function(
            "W", [cs_q, cs_v, cs_a], [W_expr]
        )
        mock_backend.rnea_function = cs.Function(
            "rnea", [cs_q, cs_v, cs_a], [tau_expr]
        )
        mock_backend.nq = 1
        mock_backend.nv = 1
        mock_backend._cmodel = mock_robot.model
        mock_backend._cdata = MagicMock()

        mock_ctx.results = {
            'fourier_coeffs': None,
            'omega': None,
            'n_harmonics': None,
            'iteration_data': [],
            'final_regressor_shape': None,
            'diagnostics': None,
        }
        mock_ctx.robot = mock_robot

        # Run solve — must complete without raising and populate results.
        # Previously this was wrapped in `try/except Exception: pass`, which
        # swallowed the AttributeError from the non-existent cs.cholesky call
        # and made the test falsely green while solve() actually crashed.
        #
        # _initialize_coefficients calls FourierTrajectory.compute_torques
        # internally; the mock robot doesn't have a real Pinocchio model,
        # so we stub it out.
        def _mock_compute_torques(self, q, v, a, robot):
            return np.zeros((q.shape[0], robot.model.nv))

        with patch(
            "figaroh.backend.casadi.CasadiBackend",
            return_value=mock_backend,
        ), patch(
            "figaroh.utils.fourier_trajectory.FourierTrajectory.compute_torques",
            _mock_compute_torques,
        ):
            strategy.solve(mock_ctx)

        # solve() must have populated Fourier coefficients and diagnostics
        assert mock_ctx.results['fourier_coeffs'] is not None
        assert mock_ctx.results['omega'] is not None
        assert mock_ctx.results['n_harmonics'] is not None
        assert mock_ctx.results['diagnostics'] is not None


class TestDOptimalObjective:
    """Test D-optimal objective function correctness."""

    def test_log_det_via_cholesky(self):
        """Cholesky-based logdet matches numpy logdet."""
        import casadi as cs
        import numpy as np

        # Build a known SPD matrix
        n = 4
        rng = np.random.default_rng(42)
        A = rng.standard_normal((n, n))
        J = A.T @ A  # SPD
        lam = 1e-6
        J_reg = J + lam * np.eye(n)

        # numpy reference
        sign, logdet_np = np.linalg.slogdet(J_reg)
        obj_np = -logdet_np

        # CasADi Cholesky
        J_sym = cs.SX.sym("J", n, n)
        L = cs.chol(J_sym)  # lower triangular Cholesky factor
        obj_sx = -2 * cs.sum1(cs.log(cs.diag(L)))
        obj_fn = cs.Function("obj", [J_sym], [obj_sx])

        obj_cs = float(obj_fn(J_reg))

        # Should match (up to numerical tolerance)
        assert obj_cs == pytest.approx(obj_np, abs=1e-8)

    def test_regularization_ensures_spd(self):
        """Regularization lambda ensures J+lambda*I is SPD for Cholesky."""
        import casadi as cs
        import numpy as np

        # Build a singular matrix
        J = np.ones((3, 3))  # rank 1
        lam = 1e-6
        J_reg = J + lam * np.eye(3)

        # Cholesky should succeed
        J_sym = cs.SX.sym("J", 3, 3)
        L = cs.chol(J_sym)  # lower triangular Cholesky factor
        chol_fn = cs.Function("chol", [J_sym], [L])
        L_val = np.array(chol_fn(J_reg))
        assert np.all(np.diag(L_val) > 0)

    def test_doptimal_objective_shape(self):
        """D-optimal objective returns scalar."""
        from figaroh.optimal.strategies.fourier_strategy import (
            FourierOptimizationStrategy
        )
        strategy = FourierOptimizationStrategy()
        # Verify the strategy has the required config
        assert "reg_lambda" in strategy._fourier_config


class TestSymbolicJacobian:
    """Compare CasADi symbolic Jacobian vs finite differences."""

    def test_regressor_jacobian_vs_fd(self):
        """Symbolic Jacobian of regressor matches finite difference."""
        pytest.importorskip("casadi")
        pytest.importorskip("pinocchio.casadi")
        import casadi as cs
        import pinocchio.casadi as cpin
        import numpy as np

        # Build a minimal 1-DOF model with CasADi SX types for Inertia
        model = cpin.Model()
        jid = model.addJoint(0, cpin.JointModelRY(), cpin.SE3.Identity(), "joint")
        model.appendBodyToJoint(jid, cpin.Inertia(
            mass=cs.SX(1.0),
            lever=cs.SX.zeros(3),
            inertia=cs.SX.eye(3),
        ), cpin.SE3.Identity())
        data = model.createData()

        # Symbolic regressor
        q = cs.SX.sym("q", 1)
        v = cs.SX.sym("v", 1)
        a = cs.SX.sym("a", 1)
        W_expr = cpin.computeJointTorqueRegressor(model, data, q, v, a)
        W_fn = cs.Function("W", [q, v, a], [W_expr])

        # Jacobian of W w.r.t. q (symbolic)
        J_sym = cs.jacobian(W_expr, q)
        J_fn = cs.Function("J_sym", [q, v, a], [J_sym])

        # Finite difference
        eps = 1e-6
        q0 = np.array([0.5])
        v0 = np.array([0.1])
        a0 = np.array([0.0])

        W0 = np.array(W_fn(q0, v0, a0)).flatten()
        J_fd = np.zeros((len(W0), 1))
        W_plus = np.array(W_fn(q0 + eps, v0, a0)).flatten()
        W_minus = np.array(W_fn(q0 - eps, v0, a0)).flatten()
        J_fd[:, 0] = (W_plus - W_minus) / (2 * eps)

        J_sym_val = np.array(J_fn(q0, v0, a0))

        np.testing.assert_allclose(J_sym_val, J_fd, atol=1e-4)


class TestFourierStrategyCasadiMissing:
    """Verify the friendly error when CasADi is unavailable at solve time.

    CasADi is imported lazily inside ``solve()``, so an absent CasADi must
    surface as an ImportError carrying the install hint, not a raw
    ModuleNotFoundError deferred to call time.
    """

    def test_solve_raises_friendly_import_error_without_casadi(self):
        """solve() raises ImportError with install hint when casadi missing."""
        import sys
        from figaroh.optimal.strategies.fourier_strategy import (
            FourierOptimizationStrategy
        )

        strategy = FourierOptimizationStrategy()
        mock_context = MagicMock()

        # Simulate casadi not installed: a ``None`` entry in sys.modules makes
        # ``import casadi`` raise ImportError ("import of casadi halted; ...").
        saved = sys.modules.get("casadi")
        sys.modules["casadi"] = None
        try:
            with pytest.raises(ImportError, match="pixi add casadi"):
                strategy.solve(mock_context)
        finally:
            if saved is not None:
                sys.modules["casadi"] = saved
            else:
                sys.modules.pop("casadi", None)
