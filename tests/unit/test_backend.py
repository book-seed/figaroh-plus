"""Tests for the computation backend abstraction layer."""

import pytest
import numpy as np
from typing import Callable
from unittest.mock import Mock, patch, MagicMock

from figaroh.backend.base import Backend, BackendType, create_backend
from figaroh.backend.numerical import NumericalBackend


class TestBackendABC:
    """Test Backend abstract base class contract."""

    def test_backend_abc_cannot_be_instantiated(self):
        """Backend ABC raises TypeError when instantiated directly."""
        with pytest.raises(TypeError):
            Backend()  # type: ignore

    def test_concrete_backend_must_implement_abstract_methods(self):
        """Subclass missing abstract methods raises TypeError."""
        class IncompleteBackend(Backend):
            pass

        with pytest.raises(TypeError):
            IncompleteBackend()  # type: ignore

    def test_concrete_backend_with_all_methods(self):
        """Subclass implementing all abstract methods can be instantiated."""
        class ConcreteBackend(Backend):
            def build_regressor(self, q, v, a, identif_config):
                return np.zeros((10, 5))
            def gradient(self, objective_fn, x):
                return np.zeros_like(x)
            def jacobian(self, constraints_fn, x):
                return np.zeros((1, len(x)))
            def create_solver(self, nlp_def, opts):
                return lambda x0: {'x': x0, 'f': 0.0, 'g': np.array([])}
            @property
            def name(self):
                return "concrete"

        backend = ConcreteBackend()
        assert backend.name == "concrete"
        assert callable(backend.build_regressor)
        assert callable(backend.gradient)
        assert callable(backend.jacobian)
        assert callable(backend.create_solver)


class TestCreateBackendFactory:
    """Test create_backend factory function."""

    def test_create_backend_numerical(self):
        """create_backend('numerical') returns NumericalBackend."""
        backend = create_backend("numerical")
        assert isinstance(backend, NumericalBackend)

    def test_create_backend_default_is_numerical(self):
        """create_backend() with no args returns NumericalBackend."""
        backend = create_backend()
        assert isinstance(backend, NumericalBackend)

    def test_create_backend_passthrough(self):
        """create_backend(backend_instance) returns the instance unchanged."""
        numerical = NumericalBackend()
        result = create_backend(numerical)
        assert result is numerical

    def test_create_backend_invalid_string(self):
        """create_backend('invalid') raises ValueError."""
        with pytest.raises(ValueError, match="Unknown backend"):
            create_backend("invalid")

    def test_create_backend_casadi_without_deps(self):
        """create_backend('casadi') without casadi installed raises ImportError."""
        with patch.dict('sys.modules', {'pinocchio.casadi': None}):
            with pytest.raises(ImportError):
                create_backend("casadi")
