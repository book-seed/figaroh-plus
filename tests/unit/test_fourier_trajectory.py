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

    def test_mx_trajectory_construction(self):
        """Build CasADi MX trajectory and verify shapes and basic evaluation."""
        pytest.importorskip("casadi")
        import casadi as cs
        from figaroh.utils.fourier_trajectory import FourierTrajectory

        n_harmonics = 3
        n_act = 2
        Ns = 5
        traj = FourierTrajectory(n_harmonics=n_harmonics, n_act=n_act)

        t_vec = cs.MX.sym("t", 1, Ns)
        coeffs = cs.MX.sym("coeffs", n_act, 2 * n_harmonics + 1)
        Q, V, A = traj.build_mx_trajectory(t_vec, coeffs)

        assert Q.shape == (n_act, Ns)
        assert V.shape == (n_act, Ns)
        assert A.shape == (n_act, Ns)

        # Evaluate numerically: a0=1, all other coeffs=0 → q=1, v=0, a=0
        coeffs_val = np.zeros((n_act, 2 * n_harmonics + 1))
        coeffs_val[0, 0] = 1.0
        t_val = np.linspace(0, 1, Ns).reshape(1, Ns)

        q_fn = cs.Function("q", [t_vec, coeffs], [Q])
        q_val = np.array(q_fn(t_val, coeffs_val))
        assert q_val[0, 0] == pytest.approx(1.0)

        v_fn = cs.Function("v", [t_vec, coeffs], [V])
        v_val = np.array(v_fn(t_val, coeffs_val))
        assert np.allclose(v_val, 0.0, atol=1e-10)

    def test_mx_position_velocity_acceleration(self):
        """Verify MX trajectory returns correct q, v, a for known coefficients."""
        pytest.importorskip("casadi")
        import casadi as cs
        from figaroh.utils.fourier_trajectory import FourierTrajectory

        n_harmonics = 3
        n_act = 1
        omega = 2 * np.pi / 10.0
        traj = FourierTrajectory(n_harmonics=n_harmonics, n_act=n_act, omega=omega)

        # Single time point as (1, 1) row vector
        t_vec = cs.MX.sym("t", 1, 1)
        coeffs_sym = cs.MX.sym("coeffs", n_act, 2 * n_harmonics + 1)
        Q, V, A = traj.build_mx_trajectory(t_vec, coeffs_sym)

        # Coefficients: a0=0, a1=1, b1=0, all others 0
        # -> v = sin(omega*t), q = -cos(omega*t)/omega, a = omega*cos(omega*t)
        coeffs_vec = np.zeros((n_act, 2 * n_harmonics + 1))
        coeffs_vec[0, 1] = 1.0  # a1 (velocity sin amplitude)

        q_fn = cs.Function("q", [t_vec, coeffs_sym], [Q])
        v_fn = cs.Function("v", [t_vec, coeffs_sym], [V])
        a_fn = cs.Function("a", [t_vec, coeffs_sym], [A])

        t_val = np.array([[1.5]])
        q_val = float(q_fn(t_val, coeffs_vec))
        v_val = float(v_fn(t_val, coeffs_vec))
        a_val = float(a_fn(t_val, coeffs_vec))

        expected_q = -np.cos(omega * 1.5) / omega
        expected_v = np.sin(omega * 1.5)
        expected_a = omega * np.cos(omega * 1.5)

        assert q_val == pytest.approx(expected_q, abs=1e-10)
        assert v_val == pytest.approx(expected_v, abs=1e-10)
        assert a_val == pytest.approx(expected_a, abs=1e-10)


class TestFiniteDifference:
    """Compare CasADi symbolic expressions against finite differences."""

    def test_mx_trajectory_vs_fd_velocity(self):
        """MX trajectory velocity matches finite difference of numpy eval."""
        pytest.importorskip("casadi")
        import casadi as cs
        import numpy as np
        from figaroh.utils.fourier_trajectory import FourierTrajectory

        n_harmonics = 4
        n_act = 2
        omega = 2.0
        traj = FourierTrajectory(n_harmonics=n_harmonics, n_act=n_act, omega=omega)

        rng = np.random.default_rng(42)
        coeffs = rng.uniform(-0.1, 0.1, size=(n_act, 2 * n_harmonics + 1))
        coeffs[:, 0] = 0.5

        # -- CasADi MX trajectory (single time point) --
        t_vec = cs.MX.sym("t", 1, 1)
        coeffs_sym = cs.MX.sym("coeffs", n_act, 2 * n_harmonics + 1)
        Q, V, _A = traj.build_mx_trajectory(t_vec, coeffs_sym)
        v_fn = cs.Function("v_sym", [t_vec, coeffs_sym], [V])

        # -- Finite difference of numpy evaluation --
        t_vals = np.array([0.2, 1.3, 2.7])
        eps = 1e-6

        for t0 in t_vals:
            v_sym_val = np.array(v_fn(np.array([[t0]]), coeffs)).flatten()
            q_plus = traj.get_trajectory(np.array([t0 + eps]), coeffs).flatten()
            q_minus = traj.get_trajectory(np.array([t0 - eps]), coeffs).flatten()
            v_fd = (q_plus - q_minus) / (2 * eps)

            np.testing.assert_allclose(v_sym_val, v_fd, atol=1e-4)

    def test_mx_trajectory_vs_fd_acceleration(self):
        """MX trajectory acceleration matches finite difference of numpy eval."""
        pytest.importorskip("casadi")
        import casadi as cs
        import numpy as np
        from figaroh.utils.fourier_trajectory import FourierTrajectory

        n_harmonics = 4
        n_act = 2
        omega = 2.0
        traj = FourierTrajectory(n_harmonics=n_harmonics, n_act=n_act, omega=omega)

        rng = np.random.default_rng(42)
        coeffs = rng.uniform(-0.1, 0.1, size=(n_act, 2 * n_harmonics + 1))
        coeffs[:, 0] = 0.5

        # -- CasADi MX trajectory (single time point) --
        t_vec = cs.MX.sym("t", 1, 1)
        coeffs_sym = cs.MX.sym("coeffs", n_act, 2 * n_harmonics + 1)
        _Q, _V, A = traj.build_mx_trajectory(t_vec, coeffs_sym)
        a_fn = cs.Function("a_sym", [t_vec, coeffs_sym], [A])

        # -- Finite difference of numpy evaluation (second order) --
        t_vals = np.array([0.2, 1.3, 2.7])
        eps = 1e-4  # larger eps for second derivative FD

        for t0 in t_vals:
            a_sym_val = np.array(a_fn(np.array([[t0]]), coeffs)).flatten()
            q_plus = traj.get_trajectory(np.array([t0 + eps]), coeffs).flatten()
            q_center = traj.get_trajectory(np.array([t0]), coeffs).flatten()
            q_minus = traj.get_trajectory(np.array([t0 - eps]), coeffs).flatten()
            a_fd = (q_plus - 2 * q_center + q_minus) / (eps ** 2)

            np.testing.assert_allclose(a_sym_val, a_fd, atol=1e-2)

    def test_mx_get_velocity_matches_direct(self):
        """MX trajectory velocity matches FourierTrajectory.get_velocity."""
        pytest.importorskip("casadi")
        import casadi as cs
        import numpy as np
        from figaroh.utils.fourier_trajectory import FourierTrajectory

        n_harmonics = 5
        n_act = 3
        omega = 1.5
        traj = FourierTrajectory(n_harmonics=n_harmonics, n_act=n_act, omega=omega)

        rng = np.random.default_rng(7)
        coeffs = rng.uniform(-0.2, 0.2, size=(n_act, 2 * n_harmonics + 1))
        coeffs[:, 0] = 0.3

        # CasADi MX symbolic velocity (single time point)
        t_vec = cs.MX.sym("t", 1, 1)
        coeffs_sym = cs.MX.sym("coeffs", n_act, 2 * n_harmonics + 1)
        _Q, V, _A = traj.build_mx_trajectory(t_vec, coeffs_sym)
        v_cs_fn = cs.Function("v_cs", [t_vec, coeffs_sym], [V])

        # Compare at multiple time points
        t_vals = np.linspace(0, 2 * np.pi, 50)
        v_np = traj.get_velocity(t_vals, coeffs)
        v_cs = np.array([v_cs_fn(np.array([[t]]), coeffs).toarray().flatten() for t in t_vals])

        np.testing.assert_allclose(v_cs, v_np, atol=1e-10)
