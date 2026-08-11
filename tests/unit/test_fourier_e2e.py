"""End-to-end integration test for Fourier excitation trajectory optimization.

Requires: CasADi, pinocchio.casadi, IPOPT.
Skip gracefully when dependencies are missing.
"""

import numpy as np
import pytest

pytest.importorskip("casadi")
pytest.importorskip("pinocchio.casadi")


@pytest.fixture
def simple_robot():
    """Build a minimal 1-DOF pendulum robot."""
    import pinocchio as pin
    from pinocchio.robot_wrapper import RobotWrapper

    model = pin.Model()
    model.name = "pendulum"
    joint_id = model.addJoint(
        0, pin.JointModelRY(), pin.SE3.Identity(), "joint_1"
    )
    inertia = pin.Inertia.FromCylinder(1.0, 0.1, 1.0)
    body_placement = pin.SE3(np.eye(3), np.array([0.0, 0.0, 0.5]))
    model.appendBodyToJoint(joint_id, inertia, body_placement)
    model.upperPositionLimit = np.array([np.pi])
    model.lowerPositionLimit = np.array([-np.pi])
    model.velocityLimit = np.array([10.0])
    model.effortLimit = np.array([100.0])
    data = model.createData()
    robot = RobotWrapper(model)
    robot.data = data
    robot.q0 = np.zeros(model.nq)
    robot.v0 = np.zeros(model.nv)
    return robot


def _make_config():
    return {
        "n_wps": 3, "freq": 100, "t_s": 0.1,
        "soft_lim": 0.05, "max_attempts": 10,
        "trajectory_type": "fourier",
        "fourier_config": {
            "n_harmonics": 1,
            "n_samples": 10,
            "fourier_frequency": 1.0,
            "reg_lambda": 1e-3,
        },
    }


def _make_id_config():
    return {
        "active_joints": ["joint_1"],
        "act_Jid": [1],
        "act_idxq": [0],
        "act_idxv": [0],
        "has_friction": False,
        "has_actuator_inertia": False,
        "has_joint_offset": False,
    }


class TestFourierPipeline:
    """Smoke tests that verify the Fourier pipeline without full IPOPT solve."""

    def test_strategy_creation_and_initialization(self, simple_robot):
        """Strategy factory creates Fourier strategy and initialize works."""
        from figaroh.optimal.base_optimal_trajectory import BaseOptimalTrajectory
        from unittest.mock import patch

        with patch(
            "figaroh.optimal.base_optimal_trajectory.load_param",
            return_value=(_make_config(), _make_id_config()),
        ):
            traj = BaseOptimalTrajectory(
                simple_robot,
                config_file="dummy.yaml",
            )

        assert traj.strategy.name() == "fourier"
        traj.initialize()
        assert len(traj.idx_b) > 0, "Should find base parameters"

    def test_solve_produces_results(self, simple_robot):
        """Full Fourier solve produces valid trajectory results."""
        from figaroh.optimal.base_optimal_trajectory import BaseOptimalTrajectory
        from unittest.mock import patch

        # Override IPOPT max_iter to keep this test fast
        config = _make_config()
        config["fourier_config"] = dict(config["fourier_config"])
        config["fourier_config"]["n_harmonics"] = 1
        config["fourier_config"]["n_samples"] = 5  # minimal

        with patch(
            "figaroh.optimal.base_optimal_trajectory.load_param",
            return_value=(config, _make_id_config()),
        ):
            traj = BaseOptimalTrajectory(
                simple_robot,
                config_file="dummy.yaml",
            )
        traj.initialize()
        traj.solve()

        assert traj.results["fourier_coeffs"] is not None
        assert traj.results["omega"] is not None
        assert traj.results["n_harmonics"] is not None

        # Verify diagnostics are populated
        diag = traj.results["diagnostics"]
        assert diag is not None
        assert diag["condition_number"] > 0
        assert np.isfinite(diag["condition_number"])
        assert len(diag["fim_eigenvalues"]["spectrum"]) > 0

        # Synthesize trajectory samples from coefficients for regressor build
        coeffs = traj.results["fourier_coeffs"]
        omega = traj.results["omega"]
        n_harmonics = traj.results["n_harmonics"]
        t_list, q_list, v_list, a_list = traj._synthesize_fourier_samples(
            coeffs, omega, n_harmonics, n_plot=200
        )
        W_b = traj._stack_base_regressors(
            q_list[0], v_list[0], a_list[0],
        )
        cond = float(np.linalg.cond(W_b))
        assert cond > 0 and np.isfinite(cond)
