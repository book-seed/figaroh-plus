"""Tests for Fourier trajectory configuration parsing."""

import pytest
from figaroh.optimal.config import create_config


class TestFourierConfigParsing:
    """Test that Fourier-specific config fields are parsed correctly."""

    def test_default_fourier_fields(self):
        """When fourier_config is absent, defaults are used."""
        unified_cfg = {
            "problem": {},
            "trajectory": {},
            "constraints": {},
            "output": {},
        }
        result = create_config(unified_cfg)
        # trajectory_type should exist
        assert "trajectory_type" in result
        assert result["trajectory_type"] == "spline"
        # fourier sub-config should exist
        fourier = result.get("fourier_config", {})
        assert fourier["n_harmonics"] == 5
        assert fourier["fourier_frequency"] is None
        assert fourier["n_samples"] == 200
        assert fourier["reg_lambda"] == 1.0e-6
        assert fourier["tanh_alpha_opt"] == 10
        assert fourier["tanh_alpha_id"] == 100

    def test_explicit_fourier_config(self):
        """Fourier fields override defaults when provided."""
        unified_cfg = {
            "problem": {},
            "trajectory": {
                "type": "fourier",
                "fourier": {
                    "n_harmonics": 7,
                    "fourier_frequency": 1.5,
                    "n_samples": 500,
                    "reg_lambda": 1.0e-8,
                    "tanh_alpha_opt": 20,
                    "tanh_alpha_id": 200,
                },
            },
            "constraints": {},
            "output": {},
        }
        result = create_config(unified_cfg)
        assert result["trajectory_type"] == "fourier"
        fourier = result["fourier_config"]
        assert fourier["n_harmonics"] == 7
        assert fourier["fourier_frequency"] == 1.5
        assert fourier["n_samples"] == 500
        assert fourier["reg_lambda"] == 1.0e-8
        assert fourier["tanh_alpha_opt"] == 20
        assert fourier["tanh_alpha_id"] == 200

    def test_invalid_trajectory_type(self):
        """Invalid trajectory_type raises ValueError."""
        unified_cfg = {
            "problem": {},
            "trajectory": {"type": "polynomial"},
            "constraints": {},
            "output": {},
        }
        with pytest.raises(ValueError, match="trajectory_type.*polynomial"):
            create_config(unified_cfg)

    def test_partial_fourier_config(self):
        """Partial fourier config uses defaults for missing fields."""
        unified_cfg = {
            "problem": {},
            "trajectory": {
                "type": "fourier",
                "fourier": {"n_harmonics": 3},
            },
            "constraints": {},
            "output": {},
        }
        result = create_config(unified_cfg)
        assert result["trajectory_type"] == "fourier"
        fourier = result["fourier_config"]
        assert fourier["n_harmonics"] == 3  # explicit
        assert fourier["n_samples"] == 200  # default
        assert fourier["reg_lambda"] == 1.0e-6  # default

    def test_legacy_config_preserves_behavior(self):
        """Legacy format fields are unchanged."""
        from figaroh.optimal.config import load_param
        from unittest.mock import patch, MagicMock

        robot = MagicMock()
        robot.model.name = "test_robot"

        yaml_content = """
identification:
  trajectory_params:
    - n_wps: 8
      freq: 50
      t_s: 1.0
      soft_lim: 0.1
      max_attempts: 200
  backend: numerical
"""
        with patch("builtins.open") as mock_open:
            mock_open.return_value.__enter__.return_value.read.return_value = yaml_content
            with patch("yaml.load") as mock_yaml:
                mock_yaml.return_value = {
                    "identification": {
                        "trajectory_params": [{
                            "n_wps": 8, "freq": 50, "t_s": 1.0,
                            "soft_lim": 0.1, "max_attempts": 200,
                        }],
                        "backend": "numerical",
                    }
                }
                with patch(
                    "figaroh.optimal.config.get_identification_param_from_yaml",
                    return_value={},
                ):
                    traj_config, _ = load_param(robot, "dummy.yaml")
                    assert traj_config["n_wps"] == 8
                    assert traj_config["freq"] == 50
                    assert traj_config["trajectory_type"] == "spline"  # default


class TestConfigStrategyIntegration:
    """Test config-to-strategy integration."""

    def test_create_strategy_from_default_config(self):
        """Default output from create_config can create a fourier strategy."""
        pytest.importorskip("casadi")
        from figaroh.optimal.config import create_config
        from figaroh.optimal.strategies import create_strategy

        unified_cfg = {
            "problem": {},
            "trajectory": {"type": "fourier"},
            "constraints": {},
            "output": {},
        }
        cfg = create_config(unified_cfg)
        assert cfg["trajectory_type"] == "fourier"

        strategy = create_strategy(
            cfg["trajectory_type"],
            fourier_config=cfg.get("fourier_config"),
        )
        assert strategy.name() == "fourier"
        assert strategy._fourier_config["n_harmonics"] == 5
        assert strategy._fourier_config["n_samples"] == 200
        assert strategy._fourier_config["reg_lambda"] == 1.0e-6
        assert strategy._fourier_config["tanh_alpha_opt"] == 10
        assert strategy._fourier_config["tanh_alpha_id"] == 100

    def test_create_strategy_with_custom_config(self):
        """Custom fourier config propagates through create_strategy."""
        pytest.importorskip("casadi")
        from figaroh.optimal.config import create_config
        from figaroh.optimal.strategies import create_strategy

        unified_cfg = {
            "problem": {},
            "trajectory": {
                "type": "fourier",
                "fourier": {
                    "n_harmonics": 8,
                    "n_samples": 100,
                    "reg_lambda": 1.0e-8,
                    "tanh_alpha_opt": 20,
                    "tanh_alpha_id": 200,
                },
            },
            "constraints": {},
            "output": {},
        }
        cfg = create_config(unified_cfg)

        strategy = create_strategy(
            cfg["trajectory_type"],
            fourier_config=cfg.get("fourier_config"),
        )
        assert strategy.name() == "fourier"
        assert strategy._fourier_config["n_harmonics"] == 8
        assert strategy._fourier_config["n_samples"] == 100
        assert strategy._fourier_config["reg_lambda"] == 1.0e-8
        assert strategy._fourier_config["tanh_alpha_opt"] == 20
        assert strategy._fourier_config["tanh_alpha_id"] == 200

    def test_create_strategy_spline_default_config(self):
        """Default spline config creates SplineOptimizationStrategy."""
        from figaroh.optimal.config import create_config
        from figaroh.optimal.strategies import create_strategy

        unified_cfg = {
            "problem": {},
            "trajectory": {},
            "constraints": {},
            "output": {},
        }
        cfg = create_config(unified_cfg)
        assert cfg["trajectory_type"] == "spline"

        strategy = create_strategy(cfg["trajectory_type"])
        assert strategy.name() == "spline"

    def test_create_strategy_fourier_default_with_frequency(self):
        """Fourier frequency config propagates to strategy."""
        pytest.importorskip("casadi")
        from figaroh.optimal.config import create_config
        from figaroh.optimal.strategies import create_strategy

        unified_cfg = {
            "problem": {},
            "trajectory": {
                "type": "fourier",
                "fourier": {"fourier_frequency": 2.5},
            },
            "constraints": {},
            "output": {},
        }
        cfg = create_config(unified_cfg)
        strategy = create_strategy(
            cfg["trajectory_type"],
            fourier_config=cfg.get("fourier_config"),
        )
        assert strategy._fourier_config["fourier_frequency"] == 2.5
