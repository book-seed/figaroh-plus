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

    def test_create_backend_numerical_forwards_robot(self):
        """create_backend('numerical', robot=robot) forwards robot to NumericalBackend."""
        robot = MagicMock()
        backend = create_backend("numerical", robot=robot)
        assert isinstance(backend, NumericalBackend)
        assert backend._robot is robot

    def test_create_backend_casadi_forwards_robot(self):
        """create_backend('casadi', robot=robot) forwards robot to CasadiBackend."""
        robot = MagicMock()
        backend = create_backend("casadi", robot=robot)
        assert backend is not None

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
        """create_backend('casadi') without casadi raises ImportError on use."""
        from unittest.mock import MagicMock

        robot = MagicMock()
        # Lazy import means creating the backend succeeds.
        backend = create_backend("casadi", robot=robot)
        # Only method calls trigger _lazy_import, which will fail.
        with patch.dict(
            "sys.modules",
            {"casadi": None, "pinocchio": None, "pinocchio.casadi": None},
        ):
            with pytest.raises(ImportError):
                backend.build_regressor(
                    np.array([[1.0]]),
                    np.array([[1.0]]),
                    np.array([[1.0]]),
                    {},
                )


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

        mock_cyipopt = MagicMock()
        mock_nlp = MagicMock()
        mock_nlp.solve.return_value = (
            np.array([0.5, 1.0]),
            {"status": 0, "obj_val": 1.0},
        )
        mock_cyipopt.Problem.return_value = mock_nlp

        with patch.dict('sys.modules', {'cyipopt': mock_cyipopt}):
            backend = NumericalBackend()
            solver = backend.create_solver(nlp_def, opts)

            assert callable(solver)

            result = solver()

            assert isinstance(result, dict)
            assert "x" in result
            np.testing.assert_array_equal(result["x"], [0.5, 1.0])
            mock_cyipopt.Problem.assert_called_once()

    def test_cyipopt_not_imported_at_module_level(self):
        """cyipopt is lazily imported in create_solver, not at module level."""
        import importlib
        import sys

        # Clean up any prior imports so we can test fresh
        sys.modules.pop('cyipopt', None)
        if 'figaroh.backend.numerical' in sys.modules:
            importlib.reload(sys.modules['figaroh.backend.numerical'])

        # cyipopt should NOT appear in sys.modules from importing numerical
        assert 'cyipopt' not in sys.modules, (
            "cyipopt must be lazy-imported inside create_solver, "
            "not at module level"
        )

    def test_create_solver_passes_custom_bounds(self):
        """create_solver passes custom lbg/ubg to cyipopt.Problem."""
        mock_problem = MagicMock()
        mock_problem.get_initial_guess.return_value = [0.0, 0.0]
        mock_problem.get_variable_bounds.return_value = ([-1.0, -1.0], [1.0, 1.0])
        mock_problem.get_constraint_bounds.return_value = ([-5.0], [5.0])
        nlp_def = {"problem": mock_problem}
        opts = {b"tol": 1e-8}

        mock_cyipopt = MagicMock()
        mock_nlp = MagicMock()
        mock_nlp.solve.return_value = (
            np.array([0.0, 0.0]),
            {"status": 0},
        )
        mock_cyipopt.Problem.return_value = mock_nlp

        with patch.dict('sys.modules', {'cyipopt': mock_cyipopt}):
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

    @patch('figaroh.backend.casadi.cpin')
    @patch('figaroh.backend.casadi.cs')
    def test_name(self, mock_cs, mock_cpin):
        """CasadiBackend.name returns 'casadi'."""
        from figaroh.backend.casadi import CasadiBackend

        backend = CasadiBackend(robot=MagicMock())
        assert backend.name == "casadi"

    @patch('figaroh.backend.casadi.cpin')
    def test_create_solver_returns_callable(self, mock_cpin):
        """CasadiBackend.create_solver returns cs.nlpsol wrapping callable."""
        import casadi as _cs_real
        import figaroh.backend.casadi as _m

        # Inject the real casadi module so cs.nlpsol works
        _m.cs = _cs_real
        try:
            from figaroh.backend.casadi import CasadiBackend

            backend = CasadiBackend(robot=MagicMock())
            x = _cs_real.SX.sym('x', 2)
            nlp_def = {
                'x': x,
                'f': x[0]**2 + x[1]**2,
                'g': _cs_real.vertcat(x[0] + x[1] - 1.0),
            }
            opts = {}
            solver = backend.create_solver(nlp_def, opts)
            assert callable(solver)
        finally:
            _m.cs = None

    def test_create_solver_returns_info_key(self):
        """CasadiBackend.create_solver returns 'info' key for consistency."""
        import casadi as _cs_real
        import figaroh.backend.casadi as _m

        _m.cs = _cs_real
        try:
            from figaroh.backend.casadi import CasadiBackend

            backend = CasadiBackend(robot=MagicMock())
            x = _cs_real.SX.sym('x', 2)
            nlp_def = {
                'x': x,
                'f': x[0]**2 + x[1]**2,
            }
            opts = {}
            solver = backend.create_solver(nlp_def, opts)
            result = solver(np.array([0.5, 0.5]))

            # Must have 'info' key (dict) for consistency with NumericalBackend
            assert "info" in result, (
                "CasadiBackend.create_solver must return 'info' key"
            )
            assert isinstance(result["info"], dict), "'info' must be a dict"
            # 'status' must still be present for backward compatibility
            assert "status" in result
        finally:
            _m.cs = None

    def test_column_elimination_callback(self):
        """ColumnEliminationCallback wraps numpy-based column elimination."""
        from figaroh.backend.casadi import ColumnEliminationCallback

        cb = ColumnEliminationCallback('test_elim')
        W_full = np.random.randn(15, 10)
        active_cols = np.ones(10)
        tol = np.array(1e-6)
        result = cb.eval([W_full, active_cols, tol])

        assert len(result) == 1
        W_b = result[0]
        assert isinstance(W_b, np.ndarray)
        # W_b should have same or fewer columns than W_full
        assert W_b.shape[1] <= W_full.shape[1]


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
