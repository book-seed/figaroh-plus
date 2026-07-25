"""Tests for configuration parsing.

The ``backend`` key is no longer parsed from YAML (change
``remove-trajectory-backend-optionality``): the backend is determined
solely by ``trajectory_type``.
"""

import pytest
from unittest.mock import MagicMock, patch


class TestCreateConfig:
    """Test that create_config builds trajectory_config without a backend key."""

    def test_create_config_has_no_backend_key(self):
        """create_config result does not contain a 'backend' key."""
        from figaroh.optimal.config import create_config

        unified_cfg = {
            "problem": {"backend": "casadi"},  # legacy key — must be ignored
            "trajectory": {},
            "constraints": {},
            "output": {},
        }
        result = create_config(unified_cfg)
        assert "backend" not in result
        # Default trajectory keys are still present.
        assert result["n_wps"] == 5
        assert result["freq"] == 100
        assert result["t_s"] == 2.0
        assert result["soft_lim"] == 0.05
        assert result["max_attempts"] == 1000

    def test_create_config_includes_trajectory_type(self):
        """create_config propagates trajectory_type from trajectory.type."""
        from figaroh.optimal.config import create_config

        unified_cfg = {
            "problem": {},
            "trajectory": {"type": "fourier"},
            "constraints": {},
            "output": {},
        }
        result = create_config(unified_cfg)
        assert result["trajectory_type"] == "fourier"

    def test_create_config_defaults_trajectory_type_to_spline(self):
        """create_config defaults trajectory_type to 'spline' when absent."""
        from figaroh.optimal.config import create_config

        unified_cfg = {
            "problem": {},
            "trajectory": {},
            "constraints": {},
            "output": {},
        }
        result = create_config(unified_cfg)
        assert result["trajectory_type"] == "spline"
