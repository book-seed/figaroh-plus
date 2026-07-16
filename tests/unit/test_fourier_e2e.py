"""End-to-end integration test for Fourier excitation trajectory optimization.

Requires: CasADi, pinocchio.casadi, IPOPT.
Skip gracefully when dependencies are missing.
Marked as slow — run with: pytest -m slow
"""

import numpy as np
import pytest

pytest.importorskip("casadi")
pytest.importorskip("pinocchio.casadi")

pytestmark = pytest.mark.slow


class TestFourierEndToEnd:
    """Full pipeline integration tests for Fourier trajectory optimization."""

    @pytest.fixture
    def simple_robot(self):
        """Build a minimal 1-DOF pendulum robot for fast optimization."""
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

    @pytest.mark.slow
    def test_fourier_nlp_smoke(self, simple_robot):
        """Smoke test: Fourier NLP builds, initializes, and starts solving.

        Uses minimal parameters (n_harmonics=1) for fast convergence.
        """
        from figaroh.optimal.base_optimal_trajectory import BaseOptimalTrajectory
        from unittest.mock import patch

        config = {
            "n_wps": 3, "freq": 100, "t_s": 0.1,
            "soft_lim": 0.05, "max_attempts": 10,
            "trajectory_type": "fourier",
            "fourier_config": {
                "n_harmonics": 1,   # 1 harmonic = 3 coeffs (fastest)
                "n_samples": 10,     # minimal samples
                "fourier_frequency": None,
                "reg_lambda": 1e-3,
                "tanh_alpha_opt": 10,
                "tanh_alpha_id": 100,
            },
        }
        id_config = {
            "active_joints": ["joint_1"],
            "act_Jid": [1],
            "act_J": [simple_robot.model.joints[1]],
            "act_idxq": [0],
            "act_idxv": [0],
            "has_friction": False,
            "has_actuator_inertia": False,
            "has_joint_offset": False,
        }

        with patch(
            "figaroh.optimal.base_optimal_trajectory.load_param",
            return_value=(config, id_config),
        ):
            traj = BaseOptimalTrajectory(
                simple_robot, ["joint_1"],
                config_file="dummy.yaml",
                backend="casadi",
            )

        assert traj.strategy.name() == "fourier"
        traj.initialize()
        assert len(traj.idx_b) > 0, "Should find base parameters"

        # Solve — on 1-DOF with 1 harmonic this should converge quickly
        traj.solve()

        assert len(traj.results["T_F"]) > 0
        assert len(traj.results["P_F"]) > 0

        # Condition number should be finite and positive
        W_b = traj._stack_base_regressors(
            traj.results["P_F"][0],
            traj.results["V_F"][0],
            traj.results["A_F"][0],
        )
        cond = float(np.linalg.cond(W_b))
        assert cond > 0
        assert np.isfinite(cond), f"Condition number {cond} should be finite"
