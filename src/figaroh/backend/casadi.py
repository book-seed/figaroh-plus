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
from typing import Any
import os

import numpy as np

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
                "The Fourier trajectory strategy requires this backend; use "
                "trajectory_type='spline' if CasADi is unavailable."
            ) from e
        cpin = _cpin


class CasadiBackend:
    """CasADi symbolic computation backend.

    Builds a symbolic regressor using ``pinocchio.casadi`` on first access
    (lazy initialization), evaluates it vectorised via ``cs.Function.map()``,
    and exposes the regressor / RNEA functions as properties for the Fourier
    trajectory strategy. There is no longer an abstract ``Backend`` base
    class — ``CasadiBackend`` is the sole backend, created on demand by the
    Fourier trajectory strategy (the sole consumer of the symbolic model).

    Args:
        robot: ``RobotWrapper`` instance (required for symbolic model).

    TODO(shared-symbolic-model): when the identification phase also adopts the
    symbolic regressor, replace ad-hoc construction with a module-level shared
    singleton indexed by robot inertia fingerprint (e.g.
    ``get_symbolic_model(robot)``), reused by both fourier and identification.
    For now, the disk cache (``_cache_key`` + ``~/.figaroh/casadi_cache``)
    already collapses rebuild cost to a single ``cs.Function.load`` on cache
    hit, so independent construction is fine and premature sharing is avoided.
    """

    # Version tag appended to cache keys — bump when the symbolic
    # model generation logic changes so stale caches are invalidated.
    _CACHE_VERSION = "v2"

    def __init__(self, robot: Any):
        """Initialise the CasADi backend.

        Args:
            robot: RobotWrapper instance used to build the symbolic model.
        """
        self._robot = robot
        self._cmodel = None  # pinocchio.casadi Model (lazy)
        self._cdata = None  # pinocchio.casadi Data  (lazy)
        self._W_fun = None  # cs.Function: (q, v, a) -> W
        self._rnea_fun = None  # cs.Function: (q, v, a) -> tau

    @staticmethod
    def _cache_dir() -> str:
        """Return the cache directory, creating it if necessary."""
        d = os.path.join(os.path.expanduser("~"), ".figaroh", "casadi_cache")
        os.makedirs(d, exist_ok=True)
        return d

    @staticmethod
    def _cache_key(robot) -> str:
        """Build a deterministic cache key from robot model inertial parameters."""
        import hashlib
        m = robot.model
        # Fingerprint from all inertial parameters (mass, inertia, com)
        inertia_parts = []
        for i in m.inertias:
            mass = float(i.mass) if abs(float(i.mass)) > 1e-9 else 0.0
            lever = [float(x) for x in i.lever]
            inertia_flat = [float(x) for row in i.inertia for x in row]
            inertia_parts.extend([mass] + lever + inertia_flat)
        fingerprint = f"{m.name}_{m.nq}_{m.nv}_" + "_".join(
            f"{v:.10f}" for v in inertia_parts
        )
        h = hashlib.sha256(fingerprint.encode()).hexdigest()[:16]
        return f"{m.name}_{h}"

    def _ensure_symbolic_model(self):
        """Build (or load from cache) the CasADi symbolic model."""
        if self._cmodel is not None:
            return

        _lazy_import()

        key = self._cache_key(self._robot)
        cache_file = os.path.join(
            self._cache_dir(),
            f"{key}_regressor_{self._CACHE_VERSION}.casadi",
        )

        # ── Try loading from cache ────────────────────────────────
        try:
            self._W_fun = cs.Function.load(cache_file)
            # Rebuild pinocchio model (needed for cdata, nv, nq, etc.)
            self._cmodel = cpin.Model(self._robot.model)
            self._cdata = self._cmodel.createData()
            # Try loading rnea function from companion cache file
            rnea_cache_file = cache_file.replace("regressor", "rnea")
            try:
                self._rnea_fun = cs.Function.load(rnea_cache_file)
            except Exception:
                # Rebuild rnea from the symbolic model
                cs_q = cs.SX.sym("q", self._cmodel.nq)
                cs_v = cs.SX.sym("v", self._cmodel.nv)
                cs_a = cs.SX.sym("a", self._cmodel.nv)
                tau_expr = cpin.rnea(self._cmodel, self._cdata, cs_q, cs_v, cs_a)
                self._rnea_fun = cs.Function("rnea", [cs_q, cs_v, cs_a], [tau_expr])
            return
        except Exception:
            pass  # cache miss — build from scratch

        # ── Build symbolic model ──────────────────────────────────
        self._cmodel = cpin.Model(self._robot.model)
        self._cdata = self._cmodel.createData()

        cs_q = cs.SX.sym("q", self._cmodel.nq)
        cs_v = cs.SX.sym("v", self._cmodel.nv)
        cs_a = cs.SX.sym("a", self._cmodel.nv)
        W_expr = cpin.computeJointTorqueRegressor(
            self._cmodel, self._cdata, cs_q, cs_v, cs_a
        )
        self._W_fun = cs.Function("W", [cs_q, cs_v, cs_a], [W_expr])

        # Build symbolic RNEA function
        tau_expr = cpin.rnea(self._cmodel, self._cdata, cs_q, cs_v, cs_a)
        self._rnea_fun = cs.Function("rnea", [cs_q, cs_v, cs_a], [tau_expr])

        # ── Save to cache ─────────────────────────────────────────
        try:
            self._W_fun.save(cache_file)
            rnea_cache_file = cache_file.replace("regressor", "rnea")
            self._rnea_fun.save(rnea_cache_file)
        except Exception:
            pass  # non-fatal — cache is an optimisation

    @property
    def regressor_function(self):
        """SX Function: (q, v, a) -> W(nv, n_param).

        Returns CasADi Function that computes the joint torque regressor.
        """
        self._ensure_symbolic_model()
        return self._W_fun

    @property
    def rnea_function(self):
        """SX Function: (q, v, a) -> tau(nv).

        Returns CasADi Function that computes the RNEA joint torques.
        """
        self._ensure_symbolic_model()
        return self._rnea_fun
