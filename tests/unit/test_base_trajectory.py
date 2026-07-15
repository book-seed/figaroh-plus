"""Tests for BaseTrajectory abstract base class."""

import pytest
import numpy as np


class TestBaseTrajectoryABC:
    """Test BaseTrajectory ABC contract."""

    def test_abc_cannot_be_instantiated(self):
        """BaseTrajectory ABC raises TypeError when instantiated directly."""
        from figaroh.utils.base_trajectory import BaseTrajectory
        with pytest.raises(TypeError):
            BaseTrajectory()  # type: ignore

    def test_concrete_trajectory_must_implement_abstract_methods(self):
        """Subclass missing abstract methods raises TypeError."""
        from figaroh.utils.base_trajectory import BaseTrajectory

        class IncompleteTraj(BaseTrajectory):
            pass

        with pytest.raises(TypeError):
            IncompleteTraj()  # type: ignore

    def test_concrete_trajectory_with_all_methods(self):
        """Subclass implementing all abstract methods can be instantiated."""
        from figaroh.utils.base_trajectory import BaseTrajectory

        class ConcreteTraj(BaseTrajectory):
            def get_trajectory(self, t, coeffs):
                return np.zeros((len(t), 3))

            def get_velocity(self, t, coeffs):
                return np.zeros((len(t), 3))

            def get_acceleration(self, t, coeffs):
                return np.zeros((len(t), 3))

            def compute_torques(self, q, v, a, robot):
                return np.zeros((q.shape[0], 3))

            def check_constraints(self, q, v, tau, robot):
                return False

        traj = ConcreteTraj()
        t = np.linspace(0, 1, 10)
        coeffs = np.random.randn(3, 3)
        assert traj.get_trajectory(t, coeffs).shape == (10, 3)
        assert traj.get_velocity(t, coeffs).shape == (10, 3)
        assert traj.get_acceleration(t, coeffs).shape == (10, 3)
        assert not traj.check_constraints(
            np.zeros((10, 3)), np.zeros((10, 3)), np.zeros((10, 3)), None
        )
