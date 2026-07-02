"""Tests for configuration parsing and backend selection."""

import pytest
from unittest.mock import MagicMock, patch, mock_open


class TestCreateConfigBackend:
    """Test that create_config extracts backend from problem params."""

    def test_create_config_with_backend(self):
        """create_config includes 'backend' key when present in problem params."""
        from figaroh.optimal.config import create_config

        unified_cfg = {
            "problem": {"backend": "casadi"},
            "trajectory": {},
            "constraints": {},
            "output": {},
        }
        result = create_config(unified_cfg)
        assert "backend" in result
        assert result["backend"] == "casadi"

    def test_create_config_default_backend(self):
        """create_config defaults backend to 'numerical' when absent."""
        from figaroh.optimal.config import create_config

        unified_cfg = {
            "problem": {},
            "trajectory": {},
            "constraints": {},
            "output": {},
        }
        result = create_config(unified_cfg)
        assert "backend" in result
        assert result["backend"] == "numerical"

    def test_create_config_backend_overrides_default(self):
        """create_config uses explicit backend from problem params over default."""
        from figaroh.optimal.config import create_config

        unified_cfg = {
            "problem": {"backend": "casadi"},
            "trajectory": {},
            "constraints": {},
            "output": {},
        }
        result = create_config(unified_cfg)
        assert result["backend"] == "casadi"
        # Other default keys should still be present
        assert result["n_wps"] == 5
        assert result["freq"] == 100
        assert result["t_s"] == 2.0
        assert result["soft_lim"] == 0.05
        assert result["max_attempts"] == 1000


class TestLoadParamBackend:
    """Test that load_param extracts backend from legacy YAML format."""

    def test_load_param_legacy_with_backend(self):
        """load_param includes 'backend' from identif_data in legacy format."""
        from figaroh.optimal.config import load_param

        robot = MagicMock()
        robot.model.name = "test_robot"

        # Mock is_unified_config to return False (legacy path)
        yaml_content = {
            "identification": {
                "robot_params": [{"q_lim_def": [], "dq_lim_def": []}],
                "problem_params": [{
                    "is_external_wrench": False,
                    "is_joint_torques": True,
                    "force_torque": None,
                    "external_wrench_offsets": True,
                    "has_friction": True,
                    "fv": [],
                    "fs": [],
                    "has_actuator_inertia": True,
                    "Ia": [],
                    "has_joint_offset": False,
                    "off": [],
                    "has_coupled_wrist": False,
                    "Iam6": 0,
                    "fvm6": 0,
                    "fsm6": 0,
                }],
                "processing_params": [{"ts": 0.001, "cut_off_frequency_butterworth": 100.0}],
                "tls_params": [{"mass_load": 0.0, "which_body_loaded": 0}],
                "reduction_ratio": [],
                "ratio_essential": 30.0,
                "trajectory_params": [{
                    "n_wps": 8,
                    "freq": 50,
                    "t_s": 1.0,
                    "soft_lim": 0.1,
                    "max_attempts": 500,
                }],
                "backend": "casadi",  # NEW: backend key
            }
        }

        with patch("figaroh.optimal.config.is_unified_config", return_value=False):
            with patch(
                "figaroh.optimal.config.open",
                mock_open(read_data=""),
            ):
                with patch(
                    "figaroh.optimal.config.yaml.load", return_value=yaml_content
                ):
                    traj_config, identif_config = load_param(robot, "dummy.yaml")

        assert "backend" in traj_config
        assert traj_config["backend"] == "casadi"
        # Other trajectory params preserved
        assert traj_config["n_wps"] == 8

    def test_load_param_legacy_backend_default(self):
        """load_param defaults backend to 'numerical' when absent in legacy."""
        from figaroh.optimal.config import load_param

        robot = MagicMock()
        robot.model.name = "test_robot"

        yaml_content = {
            "identification": {
                "robot_params": [{"q_lim_def": [], "dq_lim_def": []}],
                "problem_params": [{
                    "is_external_wrench": False,
                    "is_joint_torques": True,
                    "force_torque": None,
                    "external_wrench_offsets": True,
                    "has_friction": True,
                    "fv": [],
                    "fs": [],
                    "has_actuator_inertia": True,
                    "Ia": [],
                    "has_joint_offset": False,
                    "off": [],
                    "has_coupled_wrist": False,
                    "Iam6": 0,
                    "fvm6": 0,
                    "fsm6": 0,
                }],
                "processing_params": [{"ts": 0.001, "cut_off_frequency_butterworth": 100.0}],
                "tls_params": [{"mass_load": 0.0, "which_body_loaded": 0}],
                "reduction_ratio": [],
                "ratio_essential": 30.0,
                "trajectory_params": [{
                    "n_wps": 5,
                    "freq": 100,
                }],
                # No "backend" key
            }
        }

        with patch("figaroh.optimal.config.is_unified_config", return_value=False):
            with patch(
                "figaroh.optimal.config.open",
                mock_open(read_data=""),
            ):
                with patch(
                    "figaroh.optimal.config.yaml.load", return_value=yaml_content
                ):
                    traj_config, identif_config = load_param(robot, "dummy.yaml")

        assert "backend" in traj_config
        assert traj_config["backend"] == "numerical"


class TestBaseOptimalTrajectoryBackendPrecedence:
    """Test backend selection precedence in BaseOptimalTrajectory.

    Precedence:
    1. Explicit programmatic argument (highest)
    2. Config file 'backend' key
    3. Default 'numerical'
    """

    def test_config_backend_used_when_default_arg(self):
        """Config backend is used when programmatic arg is default 'numerical'."""
        from figaroh.optimal.base_optimal_trajectory import BaseOptimalTrajectory

        robot = MagicMock()
        robot.model.name = "test_robot"

        traj_config_with_backend = {
            "n_wps": 5, "freq": 100, "t_s": 2.0,
            "soft_lim": 0.05, "max_attempts": 1000,
            "backend": "casadi",
        }

        with patch("figaroh.optimal.base_optimal_trajectory.load_param") as mock_load:
            mock_load.return_value = (traj_config_with_backend, {})
            with patch(
                "figaroh.optimal.base_optimal_trajectory.create_backend"
            ) as mock_create:
                traj = BaseOptimalTrajectory(
                    robot, ["joint1"],
                    config_file="dummy.yaml",
                    backend="numerical",  # default
                )

        # create_backend should be called with "casadi" from config
        mock_create.assert_called_once()
        args, kwargs = mock_create.call_args
        assert args[0] == "casadi" or kwargs.get("backend") == "casadi" or \
            (isinstance(args[0], str) and args[0] == "casadi")

    def test_explicit_backend_wins_over_config(self):
        """Explicit programmatic backend overrides config backend."""
        from figaroh.optimal.base_optimal_trajectory import BaseOptimalTrajectory

        robot = MagicMock()

        traj_config_with_backend = {
            "n_wps": 5, "freq": 100, "t_s": 2.0,
            "soft_lim": 0.05, "max_attempts": 1000,
            "backend": "numerical",
        }

        with patch("figaroh.optimal.base_optimal_trajectory.load_param") as mock_load:
            mock_load.return_value = (traj_config_with_backend, {})
            with patch(
                "figaroh.optimal.base_optimal_trajectory.create_backend"
            ) as mock_create:
                traj = BaseOptimalTrajectory(
                    robot, ["joint1"],
                    config_file="dummy.yaml",
                    backend="casadi",  # explicit casadi
                )

        # create_backend should be called with "casadi" (explicit wins)
        mock_create.assert_called_once()
        args, kwargs = mock_create.call_args
        assert args[0] == "casadi"

    def test_default_backend_when_no_config_backend(self):
        """Default 'numerical' is used when config has no backend key."""
        from figaroh.optimal.base_optimal_trajectory import BaseOptimalTrajectory

        robot = MagicMock()

        traj_config_no_backend = {
            "n_wps": 5, "freq": 100, "t_s": 2.0,
            "soft_lim": 0.05, "max_attempts": 1000,
            # No backend key
        }

        with patch("figaroh.optimal.base_optimal_trajectory.load_param") as mock_load:
            mock_load.return_value = (traj_config_no_backend, {})
            with patch(
                "figaroh.optimal.base_optimal_trajectory.create_backend"
            ) as mock_create:
                traj = BaseOptimalTrajectory(
                    robot, ["joint1"],
                    config_file="dummy.yaml",
                    backend="numerical",
                )

        mock_create.assert_called_once()
        args, kwargs = mock_create.call_args
        assert args[0] == "numerical"
