"""Tests for FourierTrajectory."""

import pytest
import numpy as np
from unittest.mock import MagicMock


class TestFourierExpression:
    """Test Fourier series expression evaluation."""

    def test_basic_evaluation(self):
        """Evaluate Fourier series at time points returns correct shape."""
        from figaroh.utils.fourier_trajectory import FourierTrajectory

        n_harmonics = 3
        n_act = 2
        traj = FourierTrajectory(n_harmonics=n_harmonics, n_act=n_act)

        # Fourier coefficients: shape (n_act, 2*n_harmonics + 1)
        coeffs = np.zeros((n_act, 2 * n_harmonics + 1))
        coeffs[:, 0] = 0.5  # a0 offset
        coeffs[:, 1] = 0.1  # a1 sin
        coeffs[:, 2] = 0.1  # b1 cos

        t = np.linspace(0, 1, 50)
        q = traj.get_trajectory(t, coeffs)
        assert q.shape == (50, n_act)

    def test_velocity_is_derivative_of_position(self):
        """Numerical derivative of q should match analytical v."""
        from figaroh.utils.fourier_trajectory import FourierTrajectory

        n_harmonics = 5
        n_act = 3
        traj = FourierTrajectory(n_harmonics=n_harmonics, n_act=n_act)

        rng = np.random.default_rng(42)
        coeffs = rng.uniform(-0.1, 0.1, size=(n_act, 2 * n_harmonics + 1))
        coeffs[:, 0] = 0.5  # a0 offset

        t = np.linspace(0, 2 * np.pi, 200)
        dt = t[1] - t[0]

        q = traj.get_trajectory(t, coeffs)
        v = traj.get_velocity(t, coeffs)

        # Central difference for numerical velocity
        v_numerical = np.zeros_like(v)
        v_numerical[1:-1] = (q[2:] - q[:-2]) / (2 * dt)
        # First and last use forward/backward difference
        v_numerical[0] = (q[1] - q[0]) / dt
        v_numerical[-1] = (q[-1] - q[-2]) / dt

        # Tolerance accounts for truncation error of central difference
        # (O(dt^2) ~1e-3) and forward/backward difference at boundaries (O(dt) ~0.03)
        np.testing.assert_allclose(v, v_numerical, atol=5e-2)

    def test_acceleration_is_second_derivative(self):
        """Numerical second derivative of q should match analytical a."""
        from figaroh.utils.fourier_trajectory import FourierTrajectory

        n_harmonics = 5
        n_act = 2
        traj = FourierTrajectory(n_harmonics=n_harmonics, n_act=n_act)

        rng = np.random.default_rng(42)
        coeffs = rng.uniform(-0.1, 0.1, size=(n_act, 2 * n_harmonics + 1))
        coeffs[:, 0] = 0.5

        t = np.linspace(0, 2 * np.pi, 200)
        dt = t[1] - t[0]

        q = traj.get_trajectory(t, coeffs)
        a = traj.get_acceleration(t, coeffs)

        # Central difference for numerical acceleration
        a_numerical = np.zeros_like(a)
        a_numerical[1:-1] = (q[2:] - 2 * q[1:-1] + q[:-2]) / (dt ** 2)
        a_numerical[0] = a_numerical[1]
        a_numerical[-1] = a_numerical[-2]

        # Tolerance accounts for truncation error: second derivative amplifies
        # numerical error; boundaries are approximate from interior
        np.testing.assert_allclose(a, a_numerical, atol=5e-1)

    def test_torque_computation(self):
        """compute_torques delegates to pinocchio rnea."""
        from figaroh.utils.fourier_trajectory import FourierTrajectory

        traj = FourierTrajectory(n_harmonics=2, n_act=2)
        q = np.random.randn(10, 2)
        v = np.random.randn(10, 2)
        a = np.random.randn(10, 2)

        mock_robot = MagicMock()
        mock_robot.model.nv = 2
        mock_robot.model.nq = 2

        pinocchio = pytest.importorskip("pinocchio")

        # Create a minimal pinocchio model for testing
        model = pinocchio.Model()
        for _ in range(2):
            jid = model.addJoint(
                0, pinocchio.JointModelRY(),
                pinocchio.SE3.Identity(), f"joint_{_}"
            )
            model.appendBodyToJoint(
                jid, pinocchio.Inertia(1.0, np.zeros(3), np.eye(3)),
                pinocchio.SE3.Identity()
            )
        mock_robot.model = model
        mock_robot.data = model.createData()

        tau = traj.compute_torques(q, v, a, mock_robot)
        assert tau.shape == (10, 2)

    def test_casadi_expression_construction(self):
        """Build CasADi SX expression for Fourier series q(t, coeffs)."""
        pytest.importorskip("casadi")
        import casadi as cs
        from figaroh.utils.fourier_trajectory import FourierTrajectory

        n_harmonics = 3
        n_act = 2
        traj = FourierTrajectory(n_harmonics=n_harmonics, n_act=n_act)

        t = cs.SX.sym("t")
        coeffs = cs.SX.sym("coeffs", n_act, 2 * n_harmonics + 1)
        q_sx = traj.build_casadi_expression(t, coeffs)

        assert q_sx.shape == (n_act, 1)

        # Evaluate numerically
        coeffs_val = np.zeros((n_act, 2 * n_harmonics + 1))
        coeffs_val[0, 0] = 1.0  # a0 = 1 for joint 0
        q_fn = cs.Function("q", [t, coeffs], [q_sx])
        q_val = np.array(q_fn(0.5, coeffs_val)).flatten()
        assert q_val[0] == pytest.approx(1.0)  # a0 = 1

    def test_casadi_position_velocity_acceleration(self):
        """Verify CasADi SX expressions: v = dq/dt, a = d2q/dt2."""
        pytest.importorskip("casadi")
        import casadi as cs
        from figaroh.utils.fourier_trajectory import FourierTrajectory

        n_harmonics = 3
        n_act = 1
        omega = 2 * np.pi / 10.0
        traj = FourierTrajectory(n_harmonics=n_harmonics, n_act=n_act, omega=omega)

        t_sym = cs.SX.sym("t")
        coeffs_sym = cs.SX.sym("coeffs", n_act, 2 * n_harmonics + 1)

        q_sx = traj.build_casadi_expression(t_sym, coeffs_sym)
        v_sx = cs.jacobian(q_sx, t_sym)  # dq/dt
        a_sx = cs.jacobian(v_sx, t_sym)  # d2q/dt2

        # Evaluate with known coefficients: q = a0 + a1*sin(omega*t) + b1*cos(omega*t)
        coeffs_vec = np.zeros((n_act, 2 * n_harmonics + 1))
        coeffs_vec[0, 0] = 0.0   # a0
        coeffs_vec[0, 1] = 1.0   # a1 (sin)
        coeffs_vec[0, 2] = 0.0   # b1 (cos)

        # When a0=0, a1=1, b1=0, omega=2*pi/10:
        #   q = sin(omega*t)
        #   v = omega * cos(omega*t)
        #   a = -omega^2 * sin(omega*t)

        q_fn = cs.Function("q", [t_sym, coeffs_sym], [q_sx])
        v_fn = cs.Function("v", [t_sym, coeffs_sym], [v_sx])
        a_fn = cs.Function("a", [t_sym, coeffs_sym], [a_sx])

        t_val = 1.5
        q_val = float(q_fn(t_val, coeffs_vec))
        v_val = float(v_fn(t_val, coeffs_vec))
        a_val = float(a_fn(t_val, coeffs_vec))

        expected_q = np.sin(omega * t_val)
        expected_v = omega * np.cos(omega * t_val)
        expected_a = -omega**2 * np.sin(omega * t_val)

        assert q_val == pytest.approx(expected_q, abs=1e-10)
        assert v_val == pytest.approx(expected_v, abs=1e-10)
        assert a_val == pytest.approx(expected_a, abs=1e-10)
