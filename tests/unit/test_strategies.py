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

"""Tests for trajectory optimization strategy pattern."""

import pytest
import numpy as np
from unittest.mock import MagicMock, patch


class TestStrategyABC:
    """Test TrajectoryOptimizationStrategy ABC contract."""

    def test_abc_cannot_be_instantiated(self):
        """ABC raises TypeError when instantiated directly."""
        from figaroh.optimal.strategies.base_strategy import (
            TrajectoryOptimizationStrategy
        )
        with pytest.raises(TypeError):
            TrajectoryOptimizationStrategy()  # type: ignore

    def test_concrete_strategy_must_implement_solve(self):
        """Subclass missing solve raises TypeError."""
        from figaroh.optimal.strategies.base_strategy import (
            TrajectoryOptimizationStrategy
        )

        class IncompleteStrategy(TrajectoryOptimizationStrategy):
            pass

        with pytest.raises(TypeError):
            IncompleteStrategy()  # type: ignore

    def test_concrete_strategy_with_solve(self):
        """Subclass implementing solve can be instantiated."""
        from figaroh.optimal.strategies.base_strategy import (
            TrajectoryOptimizationStrategy
        )

        class ConcreteStrategy(TrajectoryOptimizationStrategy):
            def solve(self, context) -> None:
                return None

            def name(self) -> str:
                return "concrete"

        strategy = ConcreteStrategy()
        assert strategy.solve(None) is None
        assert strategy.name() == "concrete"


class TestStrategyFactory:
    """Test factory function for strategy creation."""

    def test_create_spline_strategy(self):
        """Factory returns SplineOptimizationStrategy for 'spline'."""
        from figaroh.optimal.strategies import create_strategy

        strategy = create_strategy("spline")
        from figaroh.optimal.strategies.spline_strategy import (
            SplineOptimizationStrategy
        )
        assert isinstance(strategy, SplineOptimizationStrategy)

    def test_create_invalid_strategy(self):
        """Factory raises ValueError for unknown type."""
        from figaroh.optimal.strategies import create_strategy

        with pytest.raises(ValueError, match="Unknown trajectory type"):
            create_strategy("polynomial")

    def test_create_fourier_strategy(self):
        """Factory returns FourierOptimizationStrategy for 'fourier'."""
        pytest.importorskip("casadi")
        from figaroh.optimal.strategies import create_strategy
        from figaroh.optimal.strategies.fourier_strategy import (
            FourierOptimizationStrategy
        )
        strategy = create_strategy("fourier")
        assert isinstance(strategy, FourierOptimizationStrategy)


class TestSplineStrategy:
    """Test SplineOptimizationStrategy."""

    def test_name(self):
        """SplineStrategy.name returns 'spline'."""
        from figaroh.optimal.strategies.spline_strategy import (
            SplineOptimizationStrategy
        )
        strategy = SplineOptimizationStrategy()
        assert strategy.name() == "spline"

    def test_solve_delegates_to_context(self):
        """solve() creates WaypointsGeneration and calls existing flow."""
        from figaroh.optimal.strategies.spline_strategy import (
            SplineOptimizationStrategy
        )

        strategy = SplineOptimizationStrategy()
        mock_context = MagicMock()
        mock_context.trajectory_config = {
            "n_wps": 5, "freq": 100, "t_s": 2.0,
            "soft_lim": 0.05, "max_attempts": 100,
        }
        mock_context.active_joints = ["joint1", "joint2"]
        mock_context.soft_lim_pool = np.full((3, 2), 0.05)
        mock_context.results = {
            'T_F': [], 'P_F': [], 'V_F': [], 'A_F': [],
            'iteration_data': [], 'final_regressor_shape': None,
        }
        mock_context.idx_e = np.array([], dtype=int)
        mock_context.idx_b = np.array([0, 1, 2], dtype=int)

        with patch(
            "figaroh.optimal.strategies.spline_strategy.WaypointsGeneration"
        ) as MockWP:
            mock_wp = MagicMock()
            MockWP.return_value = mock_wp
            mock_wp.gen_rand_pool.return_value = None
            mock_wp.act_idxq = [0, 1]
            mock_wp.act_idxv = [0, 1]
            mock_wp.lower_q = [-1.0, -1.0]
            mock_wp.upper_q = [1.0, 1.0]
            # pool_q needs 2D shape (n_samples, n_joints) for _build_initial_waypoints
            mock_wp.pool_q = np.random.default_rng(42).uniform(
                -1, 1, (20, 2)
            )

            with patch(
                "figaroh.optimal.strategies.spline_strategy."
                "TrajectoryConstraintManager"
            ) as MockConstraint:
                mock_cm = MagicMock()
                MockConstraint.return_value = mock_cm

                def _fake_solve_segment(ctx, s_rep, *args):
                    """Populate results so _prepare_next_segment works."""
                    ctx.results['T_F'].append(np.array([[1.0]]))
                    ctx.results['P_F'].append(np.array([[0.0]]))
                    ctx.results['V_F'].append(np.array([[0.0]]))
                    ctx.results['A_F'].append(np.array([[0.0]]))
                    ctx.results['iteration_data'].append(
                        {'final_waypoint': np.array([0.0, 0.0])}
                    )
                    return True

                with patch.object(
                    strategy, "_solve_segment",
                    side_effect=_fake_solve_segment,
                ) as mock_solve:
                    strategy.solve(mock_context)

            assert mock_solve.called
            assert len(mock_context.results['T_F']) == 2  # stack_reps=2
