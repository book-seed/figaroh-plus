# Copyright [2021-2025] Thanh Nguyen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Abstract computation backend and factory function."""

from __future__ import annotations
from abc import ABC, abstractmethod
from typing import Callable, Literal, Union, Any
import numpy as np

BackendType = Union[Literal["numerical", "casadi"], "Backend"]


class Backend(ABC):
    """Abstract computation backend.

    Defines the interface for regressor construction and IPOPT solver
    creation. Two concrete implementations are provided:
    - NumericalBackend: wraps existing RegressorBuilder + cyipopt + numdifftools
    - CasadiBackend: uses pinocchio.casadi + cs.nlpsol for analytical derivatives
    """

    @abstractmethod
    def build_regressor(
        self,
        q: np.ndarray,
        v: np.ndarray,
        a: np.ndarray,
        identif_config: Any,
    ) -> np.ndarray:
        """Build the stacked base regressor matrix W_b.

        Args:
            q: Joint positions, shape (nq, N) or (N, nq).
            v: Joint velocities, shape (nv, N) or (N, nv).
            a: Joint accelerations, shape (nv, N) or (N, nv).
            identif_config: Identification configuration dictionary.

        Returns:
            Stacked base regressor matrix, shape (N*nv, n_param).
        """
        ...

    @abstractmethod
    def gradient(
        self,
        objective_fn: Callable[[np.ndarray], float],
        x: np.ndarray,
    ) -> np.ndarray:
        """Compute gradient of objective function at point x.

        Args:
            objective_fn: Objective function f(x) -> float.
            x: Point at which to evaluate gradient.

        Returns:
            Gradient vector, same shape as x.
        """
        ...

    @abstractmethod
    def jacobian(
        self,
        constraints_fn: Callable[[np.ndarray], np.ndarray],
        x: np.ndarray,
    ) -> np.ndarray:
        """Compute Jacobian of constraint functions at point x.

        Args:
            constraints_fn: Constraint function c(x) -> ndarray.
            x: Point at which to evaluate Jacobian.

        Returns:
            Jacobian matrix, shape (n_constraints, n_vars).
        """
        ...

    @abstractmethod
    def create_solver(
        self,
        nlp_def: dict,
        opts: dict,
    ) -> Callable:
        """Create and return a callable IPOPT solver.

        Args:
            nlp_def: NLP definition.
                For numerical backends: contains 'problem' key with a
                BaseOptimizationProblem instance, plus bound info.
                For CasADi backends: contains 'x', 'f', 'g' CasADi SX symbols.
            opts: Solver options dict. For numerical backends, this is the
                IPOPTConfig.to_ipopt_options() dict. For CasADi backends, the
                options are mapped to CasADi nlpsol format.

        Returns:
            A callable solver with signature solver(x0, lbg, ubg) -> dict.
        """
        ...

    @property
    @abstractmethod
    def name(self) -> str:
        """Human-readable backend identifier."""
        ...


def create_backend(
    backend: BackendType = "numerical",
    robot=None,
    **kwargs,
) -> Backend:
    """Resolve a backend specifier to a Backend instance.

    Args:
        backend: Backend specifier — "numerical", "casadi", or a Backend
            instance (returned as-is).
        robot: RobotWrapper instance (required for "casadi", optional for
            "numerical").
        **kwargs: Backend-specific keyword arguments.

    Returns:
        A Backend instance.

    Raises:
        ImportError: If "casadi" is chosen but pinocchio.casadi is missing.
        ValueError: If the backend string is not recognized.
    """
    if isinstance(backend, Backend):
        return backend

    if backend == "numerical":
        return NumericalBackend(**kwargs)

    if backend == "casadi":
        return _create_casadi_backend(robot=robot, **kwargs)

    raise ValueError(
        f"Unknown backend: '{backend}'. Valid options: 'numerical', 'casadi'."
    )


def _create_casadi_backend(robot=None, **kwargs):
    """Lazy-import CasadiBackend to avoid import error at module level."""
    try:
        from .casadi import CasadiBackend
    except ImportError as e:
        raise ImportError(
            "CasADi backend requires conda-forge pinocchio with CasADi "
            "bindings.\nInstall with:\n"
            "  pixi add --feature casadi casadi pinocchio\n"
            "or:\n"
            "  conda install -c conda-forge casadi pinocchio\n\n"
            "Alternative: use backend='numerical' (default) which does not "
            "require CasADi."
        ) from e
    return CasadiBackend(robot=robot, **kwargs)


# Avoid circular import — NumericalBackend is imported here because
# both base.py and numerical.py need access to each other's symbols
# during create_backend resolution. This deferred import is safe because
# create_backend is only called at runtime, not at module level.
from .numerical import NumericalBackend  # noqa: E402
