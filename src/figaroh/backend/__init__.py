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

CasadiBackend holds the symbolic regressor / RNEA model built via
``pinocchio.casadi`` and is the sole backend used by the Fourier
trajectory strategy. It is created automatically when
``trajectory_type == 'fourier'``; spline trajectories do not need a
backend.

Dependencies (optional): ``casadi`` + conda-forge ``pinocchio`` with
CasADi bindings. Imported lazily so spline users need not install them.
"""

try:
    from .casadi import CasadiBackend
except ImportError:
    CasadiBackend = None  # type: ignore

__all__ = ["CasadiBackend"]
