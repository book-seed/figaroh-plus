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

"""Numerical computation backend (stub for Task 1)."""

import numpy as np
from typing import Callable, Any

from .base import Backend


class NumericalBackend(Backend):
    """Numerical computation backend.

    Stub implementation for Task 1. Full implementation will be provided
    in a subsequent task.
    """

    def build_regressor(
        self,
        q: np.ndarray,
        v: np.ndarray,
        a: np.ndarray,
        identif_config: Any,
    ) -> np.ndarray:
        """Build stacked base regressor matrix (stub)."""
        raise NotImplementedError

    def gradient(
        self,
        objective_fn: Callable[[np.ndarray], float],
        x: np.ndarray,
    ) -> np.ndarray:
        """Compute gradient of objective function (stub)."""
        raise NotImplementedError

    def jacobian(
        self,
        constraints_fn: Callable[[np.ndarray], np.ndarray],
        x: np.ndarray,
    ) -> np.ndarray:
        """Compute Jacobian of constraint functions (stub)."""
        raise NotImplementedError

    def create_solver(
        self,
        nlp_def: dict,
        opts: dict,
    ) -> Callable:
        """Create and return a callable IPOPT solver (stub)."""
        raise NotImplementedError

    @property
    def name(self) -> str:
        """Human-readable backend identifier."""
        return "numerical"
