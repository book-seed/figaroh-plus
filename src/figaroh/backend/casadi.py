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

"""CasADi symbolic computation backend.

This backend uses ``pinocchio.casadi`` for symbolic regressor construction and
CasADi's ``cs.nlpsol('ipopt', nlp)`` for derivative-free IPOPT solving.

Dependencies (optional):

- ``casadi >= 3.7.2`` (PyPI or conda-forge)
- ``pinocchio`` with CasADi bindings (conda-forge)

Lazy import pattern: ``pinocchio.casadi`` and ``casadi`` are imported only when
``CasadiBackend`` methods are first called, not at module import time.
"""

from __future__ import annotations
from typing import Callable, Any, Optional
import warnings

import numpy as np

from .base import Backend

# Lazy import placeholders — pinned at first method call by _lazy_import().
cpin = None  # pinocchio.casadi module
cs = None    # casadi module


def _lazy_import() -> None:
    """Import CasADi dependencies on first use.

    Each dependency is tested independently so that tests can patch one
    (e.g. ``cpin``) while using the real version of the other (e.g. ``cs``).

    Raises:
        ImportError: If ``casadi`` or ``pinocchio.casadi`` is not installed.
    """
    global cpin, cs

    if cs is None:
        try:
            import casadi as _cs
        except ImportError as e:
            raise ImportError(
                "CasADi backend requires the 'casadi' package.\n"
                "Install with:\n"
                "  pip install casadi\n"
                "or:\n"
                "  conda install -c conda-forge casadi"
            ) from e
        cs = _cs

    if cpin is None:
        try:
            import pinocchio.casadi as _cpin
        except ImportError as e:
            # Detect whether the user has PyPI 'pin' (no CasADi bindings)
            # vs no pinocchio at all.
            try:
                import pinocchio as _pin_check
                _pin_check_version = getattr(_pin_check, "__version__", "unknown")
            except ImportError:
                _pin_check_version = None

            if _pin_check_version is not None:
                hint = (
                    "You have the PyPI 'pin' package installed, which does "
                    "NOT include CasADi bindings.\n"
                    "Replace it with conda-forge pinocchio:\n"
                    "  pip uninstall pin\n"
                    "  pixi add --feature casadi casadi pinocchio\n"
                    "or:\n"
                    "  conda install -c conda-forge casadi pinocchio\n\n"
                )
            else:
                hint = (
                    "Install conda-forge pinocchio with CasADi bindings:\n"
                    "  pixi add --feature casadi casadi pinocchio\n"
                    "or:\n"
                    "  conda install -c conda-forge casadi pinocchio\n\n"
                )
            raise ImportError(
                "CasADi backend requires conda-forge pinocchio with CasADi "
                "bindings (the PyPI 'pin' package does NOT support CasADi).\n"
                + hint +
                "Alternative: use backend='numerical' (default)."
            ) from e
        cpin = _cpin


# ---------------------------------------------------------------------------
# IPOPT option name mapping
# ---------------------------------------------------------------------------

_IPOPT_OPTION_MAP: dict[str, str] = {
    "tol": "ipopt.tol",
    "max_iter": "ipopt.max_iter",
    "max_cpu_time": "ipopt.max_cpu_time",
    "print_level": "ipopt.print_level",
    "mu_strategy": "ipopt.mu_strategy",
    "linear_solver": "ipopt.linear_solver",
    "hessian_approximation": "ipopt.hessian_approximation",
    "acceptable_tol": "ipopt.acceptable_tol",
    "acceptable_obj_change_tol": "ipopt.acceptable_obj_change_tol",
    "warm_start_init_point": "ipopt.warm_start_init_point",
    "check_derivatives_for_naninf": "ipopt.check_derivatives_for_naninf",
    "output_file": "ipopt.output_file",
    "nlp_scaling_method": "ipopt.nlp_scaling_method",
    "fixed_variable_treatment": "ipopt.fixed_variable_treatment",
    "adaptive_mu_globalization": "ipopt.adaptive_mu_globalization",
}


def _map_ipopt_options(opts: dict) -> dict:
    """Map FIGAROH IPOPT options to CasADi nlpsol format.

    Args:
        opts: IPOPT options dict (may contain ``bytes`` keys from
            ``IPOPTConfig``).

    Returns:
        CasADi-compatible options dict with ``"ipopt.*"`` string keys.
    """
    mapped: dict[str, Any] = {}
    for key, value in opts.items():
        # Decode bytes keys if needed
        key_str = key.decode() if isinstance(key, bytes) else str(key)

        # Decode bytes values if needed
        val = value.decode() if isinstance(value, bytes) else value

        # Look up in map
        if key_str in _IPOPT_OPTION_MAP:
            mapped[_IPOPT_OPTION_MAP[key_str]] = val
        else:
            # Pass through with warning for unknown options
            warnings.warn(
                f"IPOPT option '{key_str}' not in CasADi option map. "
                "Passing as-is. Verify it is valid for cs.nlpsol.",
                UserWarning,
                stacklevel=2,
            )
            mapped[key_str] = val
    return mapped


# ---------------------------------------------------------------------------
# Column elimination helper (numpy-only, no CasADi dependency)
# ---------------------------------------------------------------------------


class ColumnEliminationCallback:
    """Numpy-based column elimination and Q-less QR.

    QR pivoting and dynamic column elimination cannot be expressed with
    CasADi SX operations. This class wraps the numpy implementations so
    they can be integrated into a CasADi symbolic graph through CasADi's
    Callback mechanism (wrap this class in a ``cs.Callback`` subclass for
    full symbolic integration).

    The ``eval`` method provides the core computation; the other methods
    mirror the CasADi ``Callback`` interface so the class can be used
    inside a ``cs.Callback`` adapter.
    """

    def __init__(self, name: str = "column_elim", opts: Optional[dict] = None):
        """Initialise the callback wrapper.

        Args:
            name: Identifier for the callback instance.
            opts: Optional dictionary of options.
        """
        self._name = name
        self._opts = opts or {}

    def get_n_in(self) -> int:
        """Number of inputs: ``W_full``, ``active_columns_mask``, ``tolerance``."""
        return 3

    def get_n_out(self) -> int:
        """Number of outputs: reduced regressor ``W_b``."""
        return 1

    def get_sparsity_in(self, i: int):
        """Return sparsity pattern for input *i* (``None`` = dense)."""
        return None

    def eval(self, arg) -> list[np.ndarray]:
        """Perform column elimination on the full regressor.

        Args:
            arg: List ``[W_full, active_cols, tol]`` where
                - ``W_full`` is a 2-D numpy array ``(N*nv, n_param)``.
                - ``active_cols`` is a 1-D mask array.
                - ``tol`` is a scalar threshold (as 0-D array).

        Returns:
            List containing the reduced regressor ``W_b``.
        """
        W_full = arg[0]
        active_cols = arg[1]
        tol = float(arg[2])

        # Apply column mask
        W_active = W_full[:, active_cols > 0.5]

        # Eliminate columns with small L2 norm
        col_norms = np.linalg.norm(W_active, axis=0)
        keep_mask = col_norms >= tol

        if not np.any(keep_mask):
            # All columns eliminated — return first column (should not happen)
            return [W_active[:, :1]]

        W_b = W_active[:, keep_mask]
        return [W_b]


# ---------------------------------------------------------------------------
# CasADi backend
# ---------------------------------------------------------------------------


class CasadiBackend(Backend):
    """CasADi symbolic computation backend.

    Builds a symbolic regressor using ``pinocchio.casadi`` on first access
    (lazy initialization), evaluates it vectorised via ``cs.Function.map()``,
    and creates ``cs.nlpsol('ipopt', nlp)`` for analytical-derivative IPOPT
    solving.

    Args:
        robot: ``RobotWrapper`` instance (required for symbolic model).
    """

    def __init__(self, robot: Any):
        """Initialise the CasADi backend.

        Args:
            robot: RobotWrapper instance used to build the symbolic model.
        """
        self._robot = robot
        self._cmodel = None  # pinocchio.casadi Model (lazy)
        self._cdata = None  # pinocchio.casadi Data  (lazy)
        self._W_fun = None  # cs.Function: (q, v, a) -> W

    def _ensure_symbolic_model(self):
        """Build CasADi symbolic model on first use."""
        if self._cmodel is not None:
            return

        _lazy_import()

        self._cmodel = cpin.Model(self._robot.model)
        self._cdata = self._cmodel.createData()

        # Build symbolic regressor function
        cs_q = cs.SX.sym("q", self._cmodel.nq)
        cs_v = cs.SX.sym("v", self._cmodel.nv)
        cs_a = cs.SX.sym("a", self._cmodel.nv)
        W_expr = cpin.computeJointTorqueRegressor(
            self._cmodel, self._cdata, cs_q, cs_v, cs_a
        )
        self._W_fun = cs.Function("W", [cs_q, cs_v, cs_a], [W_expr])

    def build_regressor(
        self,
        q: np.ndarray,
        v: np.ndarray,
        a: np.ndarray,
        identif_config: Any,
    ) -> np.ndarray:
        """Vectorised regressor via CasADi ``map()``.

        Takes ``(nq, N)`` arrays, evaluates the symbolic ``W`` function on
        each column in parallel via ``cs.Function.map()``.

        Args:
            q: Joint positions, shape ``(nq, N)`` or ``(N, nq)``.
            v: Joint velocities, shape ``(nv, N)`` or ``(N, nv)``.
            a: Joint accelerations, shape ``(nv, N)`` or ``(N, nv)``.
            identif_config: Identification configuration dictionary (may
                contain ``"tol"`` for column-elimination threshold).

        Returns:
            Stacked base regressor matrix, shape ``(N * nv, n_param)``.
        """
        self._ensure_symbolic_model()

        # Normalise to (nq/nv, N) format
        q_2d = np.atleast_2d(np.asarray(q, dtype=float))
        v_2d = np.atleast_2d(np.asarray(v, dtype=float))
        a_2d = np.atleast_2d(np.asarray(a, dtype=float))

        if q_2d.shape[0] != self._cmodel.nq:
            q_2d = q_2d.T
            v_2d = v_2d.T
            a_2d = a_2d.T

        N = q_2d.shape[1]

        # Map over N columns
        W_map = self._W_fun.map(N, "serial")
        W_full = np.array(W_map(q_2d, v_2d, a_2d))

        # CasADi map("serial") stacks samples horizontally per joint:
        #   output shape = (nv, N * n_param_per_sample)
        # Convert to interleaved format (N*nv, n_param) where row
        # ordering matches the numerical backend.
        nv = self._cmodel.nv
        n_param_total = W_full.shape[1] // N
        # Reshape to (nv, N, n_param), then transpose to (N, nv, n_param)
        # and flatten to (N*nv, n_param)
        W_stacked = (
            W_full.reshape(nv, N, n_param_total)
            .transpose(1, 0, 2)
            .reshape(N * nv, n_param_total)
        )

        # Append additional columns (friction, actuator inertia, offset)
        # BEFORE column elimination — matches numerical backend order.
        if identif_config:
            has_friction = identif_config.get("has_friction", False)
            has_actuator_inertia = identif_config.get("has_actuator_inertia", False)
            has_joint_offset = identif_config.get("has_joint_offset", False)
            act_idxv = identif_config.get("act_idxv", list(range(nv)))

            if has_friction or has_actuator_inertia or has_joint_offset:
                n_base = W_stacked.shape[1]
                n_extra = (
                    (2 if has_friction else 0) +
                    (1 if has_actuator_inertia else 0) +
                    (1 if has_joint_offset else 0)
                ) * nv
                W_ext = np.zeros((N * nv, n_base + n_extra))
                W_ext[:, :n_base] = W_stacked

                extra_col = n_base
                for j in range(nv):
                    if j in act_idxv:
                        for i in range(N):
                            row = j * N + i
                            if has_friction:
                                W_ext[row, extra_col + j] = v_2d[j, i]  # fv
                                W_ext[row, extra_col + nv + j] = np.sign(v_2d[j, i])  # fs
                            if has_actuator_inertia:
                                W_ext[row, extra_col + 2*nv + j] = a_2d[j, i]  # ia
                            if has_joint_offset:
                                W_ext[row, extra_col + 3*nv + j] = 1.0  # offset
                W_stacked = W_ext

        # Column elimination (threshold-based)
        if identif_config:
            tol = identif_config.get("tol", 1e-6)
        else:
            tol = 1e-6

        col_norms = np.linalg.norm(W_stacked, axis=0)
        keep = col_norms >= tol
        if np.any(keep):
            W_b = W_stacked[:, keep]
        else:
            W_b = W_stacked

        return W_b

    def gradient(
        self,
        objective_fn: Callable[[np.ndarray], float],
        x: np.ndarray,
    ) -> np.ndarray:
        """Compute gradient via CasADi automatic differentiation.

        Args:
            objective_fn: Objective function ``f(x) -> float``.
            x: Point at which to evaluate the gradient.

        Returns:
            Gradient vector, same shape as *x*.
        """
        _lazy_import()

        cs_x = cs.SX.sym("x", len(x))
        cs_obj = cs.SX(objective_fn(np.array(cs_x).reshape(x.shape)))
        grad_fn = cs.Function("grad", [cs_x], [cs.gradient(cs_obj, cs_x)])
        return np.array(grad_fn(x).toarray()).flatten()

    def jacobian(
        self,
        constraints_fn: Callable[[np.ndarray], np.ndarray],
        x: np.ndarray,
    ) -> np.ndarray:
        """Compute Jacobian via CasADi automatic differentiation.

        Args:
            constraints_fn: Constraint function ``c(x) -> ndarray``.
            x: Point at which to evaluate the Jacobian.

        Returns:
            Jacobian matrix, shape ``(n_constraints, n_vars)``.
        """
        _lazy_import()

        cs_x = cs.SX.sym("x", len(x))
        cs_cons = cs.SX(constraints_fn(np.array(cs_x).reshape(x.shape)))
        jac_fn = cs.Function("jac", [cs_x], [cs.jacobian(cs_cons, cs_x)])
        return np.array(jac_fn(x).toarray())

    def create_solver(
        self,
        nlp_def: dict,
        opts: dict,
    ) -> Callable:
        """Create a ``cs.nlpsol('ipopt', nlp)`` solver.

        Args:
            nlp_def: NLP definition with keys:
                - ``'x'``: CasADi SX symbol for decision variables.
                - ``'f'``: CasADi SX expression for objective.
                - ``'g'``: CasADi SX expression for constraints (optional).
                - ``'lbx'``, ``'ubx'``: decision variable bounds (optional).
            opts: IPOPT options dict (will be mapped to CasADi format).

        Returns:
            A callable with signature ``solver(x0, lbg=None, ubg=None) -> dict``.
            The returned dict contains ``'x'``, ``'f'``, ``'g'``, ``'status'``,
            and ``'info'`` (solver stats dict).
        """
        _lazy_import()

        casadi_opts = _map_ipopt_options(opts)
        nlpsol_opts: dict[str, Any] = {
            "ipopt": casadi_opts,
            "print_time": False,
        }

        solver = cs.nlpsol("traj_opt", "ipopt", nlp_def, nlpsol_opts)

        def solve_fn(x0, lbg=None, ubg=None):
            """Solve the CasADi NLP problem.

            Args:
                x0: Initial guess for decision variables.
                lbg: Lower constraint bounds (optional).
                ubg: Upper constraint bounds (optional).

            Returns:
                Dict with keys ``"x"``, ``"f"``, ``"g"``, ``"status"``,
                and ``"info"`` (solver statistics).
            """
            lbg_val = lbg if lbg is not None else []
            ubg_val = ubg if ubg is not None else []
            solution = solver(
                x0=x0,
                lbx=nlp_def.get("lbx", []),
                ubx=nlp_def.get("ubx", []),
                lbg=lbg_val,
                ubg=ubg_val,
            )
            return {
                "x": np.array(solution["x"]).flatten(),
                "f": float(solution["f"]),
                "g": (
                    np.array(solution["g"]).flatten()
                    if solution["g"].numel() > 0
                    else None
                ),
                "status": solver.stats()["return_status"],
                "info": solver.stats(),
            }

        return solve_fn

    @property
    def name(self) -> str:
        """Human-readable backend identifier."""
        return "casadi"
