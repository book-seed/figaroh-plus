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

"""Computation backends for robot dynamics.

Provides a strategy pattern for regressor construction and IPOPT solver
creation. Two backends are available:

- ``numerical`` (default): wraps existing ``RegressorBuilder`` +
  ``cyipopt`` + ``numdifftools``.
- ``casadi``: uses ``pinocchio.casadi`` for symbolic regressors and
  ``cs.nlpsol('ipopt', ...)`` for analytical-derivative IPOPT solving.
"""

from .base import Backend, BackendType, create_backend
from .numerical import NumericalBackend

try:
    from .casadi import CasadiBackend
except ImportError:
    CasadiBackend = None  # type: ignore

__all__ = [
    "Backend",
    "BackendType",
    "NumericalBackend",
    "CasadiBackend",
    "create_backend",
]
