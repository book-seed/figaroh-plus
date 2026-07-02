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

"""Numerical computation backend.

Wraps the existing ``RegressorBuilder`` + ``cyipopt.Problem`` +
``numdifftools`` path.
"""

from __future__ import annotations

import numpy as np
import numdifftools as nd
from typing import Callable, Any

from .base import Backend
from figaroh.tools.regressor import build_regressor_basic


class NumericalBackend(Backend):
    """Numerical computation backend.

    Encapsulates the existing numerical pipeline:

    - ``build_regressor``: delegates to ``build_regressor_basic`` from
      :mod:`figaroh.tools.regressor`.
    - ``gradient`` / ``jacobian``: uses ``numdifftools`` for finite-difference
      approximations.
    - ``create_solver``: wraps ``cyipopt.Problem`` into a callable solver.
    """

    def __init__(self, robot=None):
        """Initialise the backend.

        Args:
            robot: Optional robot model (e.g. a ``pinocchio.RobotWrapper``
                instance). Required when calling ``build_regressor``.
        """
        self._robot = robot

    def build_regressor(
        self,
        q: np.ndarray,
        v: np.ndarray,
        a: np.ndarray,
        identif_config: Any,
    ) -> np.ndarray:
        """Build the stacked base regressor matrix.

        Delegates to :func:`figaroh.tools.regressor.build_regressor_basic`.
        The robot must have been provided at construction time.

        Args:
            q: Joint positions, shape (nq, N) or (N, nq).
            v: Joint velocities, shape (nv, N) or (N, nv).
            a: Joint accelerations, shape (nv, N) or (N, nv).
            identif_config: Identification configuration dictionary.

        Returns:
            Stacked base regressor matrix.
        """
        return build_regressor_basic(self._robot, q, v, a, identif_config)

    def gradient(
        self,
        objective_fn: Callable[[np.ndarray], float],
        x: np.ndarray,
    ) -> np.ndarray:
        """Compute the gradient of *objective_fn* at *x* via finite differences.

        Uses :func:`numdifftools.Gradient`.

        Args:
            objective_fn: Objective function ``f(x) -> float``.
            x: Point at which to evaluate the gradient.

        Returns:
            Gradient vector, same shape as *x*.
        """
        return nd.Gradient(objective_fn)(x)

    def jacobian(
        self,
        constraints_fn: Callable[[np.ndarray], np.ndarray],
        x: np.ndarray,
    ) -> np.ndarray:
        """Compute the Jacobian of *constraints_fn* at *x* via finite differences.

        Uses :func:`numdifftools.Jacobian`.

        Args:
            constraints_fn: Constraint function ``c(x) -> ndarray``.
            x: Point at which to evaluate the Jacobian.

        Returns:
            Jacobian matrix, shape ``(n_constraints, n_vars)``.
        """
        return nd.Jacobian(constraints_fn)(x)

    def create_solver(
        self,
        nlp_def: dict,
        opts: dict,
    ) -> Callable:
        """Create and return a callable IPOPT solver.

        The solver wraps a :class:`cyipopt.Problem` constructed from the
        ``BaseOptimizationProblem`` instance provided in *nlp_def*.

        Args:
            nlp_def: NLP definition. Must contain a ``"problem"`` key with a
                :class:`~figaroh.tools.robotipopt.BaseOptimizationProblem`
                instance. May optionally contain ``"lb"``, ``"ub"``,
                ``"cl"``, ``"cu"`` keys to override the problem's built-in
                bounds.
            opts: Solver options dictionary (e.g. output of
                :meth:`IPOPTConfig.to_ipopt_options`).

        Returns:
            A callable with signature ``solver(x0=None, lbg=None, ubg=None)
            -> dict``. The returned dict contains at least the key ``"x"``
            (optimal solution) and ``"info"`` (IPOPT status information).
        """
        problem = nlp_def["problem"]
        import cyipopt
        lb = nlp_def.get("lb", problem.get_variable_bounds()[0])
        ub = nlp_def.get("ub", problem.get_variable_bounds()[1])

        def solver(
            x0: np.ndarray | list | None = None,
            lbg: list | None = None,
            ubg: list | None = None,
        ) -> dict:
            if x0 is None:
                x0 = problem.get_initial_guess()
            if lbg is None:
                lbg = problem.get_constraint_bounds()[0]
            if ubg is None:
                ubg = problem.get_constraint_bounds()[1]

            nlp = cyipopt.Problem(
                n=len(x0),
                m=len(lbg),
                problem_obj=problem,
                lb=lb,
                ub=ub,
                cl=lbg,
                cu=ubg,
            )
            for key, value in opts.items():
                nlp.add_option(key, value)

            x_opt, info = nlp.solve(x0)
            return {"x": np.asarray(x_opt), "info": info}

        return solver

    @property
    def name(self) -> str:
        """Human-readable backend identifier."""
        return "numerical"
