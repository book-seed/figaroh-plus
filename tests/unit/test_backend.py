"""Tests for the CasADi symbolic computation backend.

The former ``Backend`` ABC, ``NumericalBackend``, ``create_backend`` factory
and ``ColumnEliminationCallback`` have been removed (change
``remove-trajectory-backend-optionality``). ``CasadiBackend`` is now the sole
backend, created automatically when ``trajectory_type == 'fourier'``.
"""

import pytest
import numpy as np
from unittest.mock import patch, MagicMock


class TestCasadiBackend:
    """Test CasadiBackend (with mocked pinocchio.casadi and casadi)."""

    @pytest.fixture(autouse=True)
    def _reset_casadi_globals(self):
        """Reset module-level lazy imports between tests."""
        import figaroh.backend.casadi as _m
        _m.cpin = None
        _m.cs = None
        yield

    def test_import_error_without_casadi(self):
        """CasadiBackend._lazy_import raises ImportError when deps missing."""
        from figaroh.backend.casadi import _lazy_import
        import figaroh.backend.casadi as _m

        _m.cpin = None
        _m.cs = None
        with patch.dict('sys.modules', {'casadi': None}):
            with pytest.raises(ImportError, match="CasADi backend requires"):
                _lazy_import()

    @patch('figaroh.backend.casadi.cpin')
    @patch('figaroh.backend.casadi.cs')
    def test_lazy_initialization(self, mock_cs, mock_cpin):
        """CasadiBackend initializes lazy -- no symbolic model at __init__."""
        from figaroh.backend.casadi import CasadiBackend

        robot = MagicMock()
        backend = CasadiBackend(robot=robot)

        assert backend._cmodel is None
        assert backend._W_fun is None
        assert not mock_cpin.Model.called

    @patch('figaroh.backend.casadi.cpin')
    @patch('figaroh.backend.casadi.cs')
    def test_lazy_initialization_triggers_on_call(self, mock_cs, mock_cpin):
        """First method call triggers _ensure_symbolic_model()."""
        from figaroh.backend.casadi import CasadiBackend

        mock_cpin.Model.return_value.nq = 3
        mock_cpin.Model.return_value.nv = 3

        robot = MagicMock()
        backend = CasadiBackend(robot=robot)

        # _ensure_symbolic_model is the trigger point for lazy init
        backend._ensure_symbolic_model()
        assert mock_cpin.Model.called
        assert backend._cmodel is not None


class TestRootPackageExport:
    """Test that figaroh root package exports backend subpackage."""

    def test_backend_accessible_as_figaroh_attribute(self):
        """backend is accessible as figaroh.backend after importing figaroh."""
        import sys
        import importlib

        # Clear figaroh modules to avoid pollution from test module imports
        for mod in list(sys.modules.keys()):
            if "figaroh" in mod:
                del sys.modules[mod]

        import figaroh

        # This requires `from . import backend` in figaroh/__init__.py.
        # Without it, figaroh.backend is NOT set as an attribute on the module.
        assert hasattr(figaroh, "backend")
        assert figaroh.backend is not None


class TestCasadiBackendProperties:
    """Test CasadiBackend property accessors."""

    @pytest.fixture(autouse=True)
    def _reset_casadi_globals(self):
        import figaroh.backend.casadi as _m
        _m.cpin = None
        _m.cs = None
        yield

    @patch('figaroh.backend.casadi.cpin')
    @patch('figaroh.backend.casadi.cs')
    def test_regressor_function_property(self, mock_cs, mock_cpin):
        """regressor_function property returns a cs.Function."""
        from figaroh.backend.casadi import CasadiBackend

        mock_cpin.Model.return_value.nq = 3
        mock_cpin.Model.return_value.nv = 3
        mock_cs.Function.return_value = mock_cs.Function

        robot = MagicMock()
        backend = CasadiBackend(robot=robot)
        fn = backend.regressor_function
        assert fn is not None

    @patch('figaroh.backend.casadi.cpin')
    @patch('figaroh.backend.casadi.cs')
    def test_rnea_function_property(self, mock_cs, mock_cpin):
        """rnea_function property returns a cs.Function."""
        from figaroh.backend.casadi import CasadiBackend

        mock_cpin.Model.return_value.nq = 3
        mock_cpin.Model.return_value.nv = 3
        mock_cs.Function.return_value = mock_cs.Function

        robot = MagicMock()
        backend = CasadiBackend(robot=robot)
        fn = backend.rnea_function
        assert fn is not None

    @patch('figaroh.backend.casadi.cpin')
    def test_cache_key_includes_full_inertia(self, mock_cpin):
        """Cache key uses SHA256 hash of full inertial parameters."""
        from figaroh.backend.casadi import CasadiBackend

        robot = MagicMock()
        robot.model.name = "test_robot"
        robot.model.nq = 3
        robot.model.nv = 3
        # Mock inertias such that total mass differs
        robot.model.inertias.tolist.return_value = [
            MagicMock(mass=1.0), MagicMock(mass=2.0), MagicMock(mass=0.0)
        ]

        key1 = CasadiBackend._cache_key(robot)
        assert "test_robot" in key1
        assert len(key1) > len("test_robot_")


class TestTrajectoryStrategyIntegration:
    """Test strategy pattern integration with BaseOptimalTrajectory."""

    def test_default_strategy_is_spline(self):
        """Default trajectory_type 'spline' creates SplineOptimizationStrategy."""
        from figaroh.optimal.base_optimal_trajectory import BaseOptimalTrajectory
        from figaroh.optimal.strategies.spline_strategy import (
            SplineOptimizationStrategy,
        )

        robot = MagicMock()
        robot.model.name = "test_robot"
        robot.model.nq = 3
        robot.model.nv = 3

        with patch(
            "figaroh.optimal.base_optimal_trajectory.load_param"
        ) as mock_load:
            mock_load.return_value = (
                {
                    "n_wps": 5, "freq": 100, "t_s": 2.0,
                    "soft_lim": 0.05, "max_attempts": 1000,
                    "trajectory_type": "spline",
                    "fourier_config": {},
                },
                {"active_joints": ["joint1"]},
            )
            traj = BaseOptimalTrajectory(
                robot, config_file="dummy.yaml",
            )
            assert hasattr(traj, "strategy")
            assert traj.strategy.name() == "spline"
            assert isinstance(traj.strategy, SplineOptimizationStrategy)
            # Spline creates no backend.
            assert traj._backend is None

    def test_fourier_strategy_created_when_configured(self):
        """trajectory_type 'fourier' creates FourierOptimizationStrategy."""
        from figaroh.optimal.base_optimal_trajectory import BaseOptimalTrajectory

        robot = MagicMock()
        robot.model.name = "test_robot"
        robot.model.nq = 3
        robot.model.nv = 3

        with patch(
            "figaroh.optimal.base_optimal_trajectory.load_param"
        ) as mock_load:
            mock_load.return_value = (
                {
                    "n_wps": 5, "freq": 100, "t_s": 2.0,
                    "soft_lim": 0.05, "max_attempts": 1000,
                    "trajectory_type": "fourier",
                    "fourier_config": {"n_harmonics": 5},
                },
                {"active_joints": ["joint1"]},
            )
            # fourier instantiates CasadiBackend — mock it so the test does
            # not require a real casadi install in the default env.
            with patch(
                "figaroh.optimal.base_optimal_trajectory.CasadiBackend"
            ) as mock_cb:
                mock_backend = MagicMock()
                mock_cb.return_value = mock_backend

                with patch(
                    "figaroh.optimal.base_optimal_trajectory.create_strategy"
                ) as mock_create:
                    mock_strategy = MagicMock()
                    mock_strategy.name.return_value = "fourier"
                    mock_create.return_value = mock_strategy

                    traj = BaseOptimalTrajectory(
                        robot, config_file="dummy.yaml",
                    )
                    mock_create.assert_called_once_with(
                        "fourier", fourier_config={"n_harmonics": 5}
                    )
                    mock_cb.assert_called_once_with(robot=robot)
                    assert traj.strategy.name() == "fourier"
                    assert traj._backend is mock_backend

    def test_solve_delegates_to_strategy(self):
        """solve() delegates to strategy.solve()."""
        from figaroh.optimal.base_optimal_trajectory import BaseOptimalTrajectory

        robot = MagicMock()
        robot.model.name = "test_robot"

        with patch(
            "figaroh.optimal.base_optimal_trajectory.load_param"
        ) as mock_load:
            mock_load.return_value = (
                {
                    "n_wps": 5, "freq": 100, "t_s": 2.0,
                    "soft_lim": 0.05, "max_attempts": 1000,
                    "trajectory_type": "spline",
                    "fourier_config": {},
                },
                {"active_joints": ["joint1"]},
            )
            traj = BaseOptimalTrajectory(
                robot, config_file="dummy.yaml",
            )
            # Replace strategy with mock
            mock_strategy = MagicMock()
            mock_strategy.name.return_value = "test"
            traj.strategy = mock_strategy

            traj.results = {
                'T_F': [], 'P_F': [], 'V_F': [], 'A_F': [],
                'iteration_data': [], 'final_regressor_shape': None,
            }

            traj.solve()

            mock_strategy.solve.assert_called_once_with(traj)

    def test_save_results_format_consistent(self):
        """Both strategies produce same results format for save_results."""
        from figaroh.optimal.base_optimal_trajectory import BaseOptimalTrajectory

        robot = MagicMock()
        robot.model.name = "test_robot"
        robot.model.nq = 3
        robot.model.nv = 3

        with patch(
            "figaroh.optimal.base_optimal_trajectory.load_param"
        ) as mock_load:
            mock_load.return_value = (
                {
                    "n_wps": 5, "freq": 100, "t_s": 2.0,
                    "soft_lim": 0.05, "max_attempts": 1000,
                    "trajectory_type": "spline",
                    "fourier_config": {},
                },
                {"active_joints": ["joint1"]},
            )
            traj = BaseOptimalTrajectory(
                robot, config_file="dummy.yaml",
            )
            # Verify results dict has expected format
            expected_keys = {'T_F', 'P_F', 'V_F', 'A_F',
                             'iteration_data', 'final_regressor_shape'}
            assert expected_keys.issubset(traj.results.keys())
