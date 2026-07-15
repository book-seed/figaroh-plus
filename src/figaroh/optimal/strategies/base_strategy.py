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

"""Abstract base class for trajectory optimization strategies."""

from abc import ABC, abstractmethod


class TrajectoryOptimizationStrategy(ABC):
    """Strategy pattern for trajectory optimization.

    Each concrete strategy implements a different trajectory parameterization
    (cubic spline, Fourier series) and its associated optimization pipeline.

    The ``solve()`` method receives a context object (``BaseOptimalTrajectory``
    instance) and must populate ``context.results`` with the standard format.
    """

    @abstractmethod
    def solve(self, context) -> None:
        """Execute the trajectory optimization.

        Args:
            context: ``BaseOptimalTrajectory`` instance providing robot model,
                configuration, base parameter indices, and result storage.

        Returns:
            None. Results are written to ``context.results`` dict.
        """
        ...

    @abstractmethod
    def name(self) -> str:
        """Human-readable strategy identifier."""
        ...
