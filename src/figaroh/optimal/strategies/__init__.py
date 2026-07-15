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

"""Trajectory optimization strategies.

Provides a strategy-pattern interface for different trajectory optimization
approaches. Two strategies are available:

- ``spline``: cubic spline + cyipopt (existing, default)
- ``fourier``: Fourier series + CasADi nlpsol (new)
"""

from .base_strategy import TrajectoryOptimizationStrategy
from .spline_strategy import SplineOptimizationStrategy

try:
    from .fourier_strategy import FourierOptimizationStrategy
except ImportError:
    FourierOptimizationStrategy = None  # type: ignore

__all__ = [
    "TrajectoryOptimizationStrategy",
    "SplineOptimizationStrategy",
    "FourierOptimizationStrategy",
    "create_strategy",
]


def create_strategy(trajectory_type: str, **kwargs):
    """Factory: create a strategy instance by type.

    Args:
        trajectory_type: ``"spline"`` or ``"fourier"``.
        **kwargs: Strategy-specific keyword arguments.

    Returns:
        A TrajectoryOptimizationStrategy instance.

    Raises:
        ValueError: If trajectory_type is unknown.
    """
    if trajectory_type == "spline":
        return SplineOptimizationStrategy(**kwargs)
    elif trajectory_type == "fourier":
        if FourierOptimizationStrategy is None:
            raise ImportError(
                "FourierOptimizationStrategy requires CasADi. "
                "Install with: pixi add casadi"
            )
        return FourierOptimizationStrategy(**kwargs)
    else:
        raise ValueError(
            f"Unknown trajectory type: '{trajectory_type}'. "
            "Valid options: 'spline', 'fourier'."
        )
