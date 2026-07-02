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


class TestNumericalBackend:
    """Test NumericalBackend concrete implementation."""

    def test_name(self):
        """NumericalBackend.name returns 'numerical'."""
        backend = NumericalBackend()
        assert backend.name == "numerical"

    def test_gradient_of_quadratic(self):
        """gradient computes gradient of f(x) = x0^2 + x1^2 at (1, 2)."""
        backend = NumericalBackend()
        f = lambda x: float(x[0]**2 + x[1]**2)
        grad = backend.gradient(f, np.array([1.0, 2.0]))
        # Analytical: [2*x0, 2*x1] = [2, 4]
        np.testing.assert_allclose(grad, [2.0, 4.0], rtol=1e-5)

    def test_jacobian_of_linear_system(self):
        """jacobian computes Jacobian of f(x) = [x0+2*x1, 3*x0+4*x1]."""
        backend = NumericalBackend()
        f = lambda x: np.array([x[0] + 2*x[1], 3*x[0] + 4*x[1]])
        jac = backend.jacobian(f, np.array([1.0, 1.0]))
        expected = np.array([[1.0, 2.0], [3.0, 4.0]])
        np.testing.assert_allclose(jac, expected, rtol=1e-5)

    def test_build_regressor_delegates_to_build_regressor_basic(self):
        """build_regressor delegates to build_regressor_basic with correct args."""
        mock_robot = MagicMock()
        backend = NumericalBackend(robot=mock_robot)
        q = np.array([[1.0], [2.0], [3.0]])
        v = np.zeros((3, 1))
        a = np.zeros((3, 1))
        identif_config = {"has_friction": True}

        with patch('figaroh.backend.numerical.build_regressor_basic') as mock_build:
            mock_build.return_value = np.eye(5)
            result = backend.build_regressor(q, v, a, identif_config)

        mock_build.assert_called_once_with(mock_robot, q, v, a, identif_config)
        np.testing.assert_array_equal(result, np.eye(5))

    def test_create_solver_returns_callable(self):
        """create_solver returns a callable that returns a dict on invocation."""
        mock_problem = MagicMock()
        mock_problem.get_initial_guess.return_value = [0.5, 1.0]
        mock_problem.get_variable_bounds.return_value = ([0.0, 0.0], [2.0, 2.0])
        mock_problem.get_constraint_bounds.return_value = ([-1.0], [1.0])
        nlp_def = {"problem": mock_problem}
        opts = {b"tol": 1e-6}

        with patch('figaroh.backend.numerical.cyipopt') as mock_cyipopt:
            mock_nlp = MagicMock()
            mock_nlp.solve.return_value = (
                np.array([0.5, 1.0]),
                {"status": 0, "obj_val": 1.0},
            )
            mock_cyipopt.Problem.return_value = mock_nlp

            backend = NumericalBackend()
            solver = backend.create_solver(nlp_def, opts)

            assert callable(solver)

            result = solver()

            assert isinstance(result, dict)
            assert "x" in result
            np.testing.assert_array_equal(result["x"], [0.5, 1.0])
            mock_cyipopt.Problem.assert_called_once()

    def test_create_solver_passes_custom_bounds(self):
        """create_solver passes custom lbg/ubg to cyipopt.Problem."""
        mock_problem = MagicMock()
        mock_problem.get_initial_guess.return_value = [0.0, 0.0]
        mock_problem.get_variable_bounds.return_value = ([-1.0, -1.0], [1.0, 1.0])
        mock_problem.get_constraint_bounds.return_value = ([-5.0], [5.0])
        nlp_def = {"problem": mock_problem}
        opts = {b"tol": 1e-8}

        with patch('figaroh.backend.numerical.cyipopt') as mock_cyipopt:
            mock_nlp = MagicMock()
            mock_nlp.solve.return_value = (
                np.array([0.0, 0.0]),
                {"status": 0},
            )
            mock_cyipopt.Problem.return_value = mock_nlp

            backend = NumericalBackend()
            solver = backend.create_solver(nlp_def, opts)

            # Call with custom constraint bounds
            solver(lbg=[0.0], ubg=[10.0])

            mock_cyipopt.Problem.assert_called_once_with(
                n=2, m=1,
                problem_obj=mock_problem,
                lb=[-1.0, -1.0], ub=[1.0, 1.0],
                cl=[0.0], cu=[10.0],
            )
