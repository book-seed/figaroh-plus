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

"""Spline-based trajectory optimization strategy.

Migrates the existing cubic spline + cyipopt flow from
BaseOptimalTrajectory into the strategy pattern.
"""

import logging
import numpy as np
from typing import Tuple

from figaroh.utils.cubic_spline import WaypointsGeneration
from figaroh.optimal.contraints import TrajectoryConstraintManager

from .base_strategy import TrajectoryOptimizationStrategy


class SplineOptimizationStrategy(TrajectoryOptimizationStrategy):
    """Cubic spline trajectory optimization (existing behavior).

    Uses ndcurves cubic spline interpolation, numerical constraint evaluation,
    and cyipopt IPOPT solver.
    """

    def __init__(self):
        self.logger = logging.getLogger(__name__)
        self.logger.addHandler(logging.NullHandler())

    def name(self) -> str:
        return "spline"

    def solve(self, context) -> None:
        """Execute spline-based optimization. Mirrors existing solve()."""
        traj_cfg = context.trajectory_config
        stack_reps = traj_cfg.get("stack_reps", 2)

        context.WP = WaypointsGeneration(
            context.robot,
            traj_cfg["n_wps"],
            context.active_joints,
            context.soft_lim_pool,
        )

        context.constraint_manager = TrajectoryConstraintManager(
            context.robot, context.WP, traj_cfg, context.identif_config
        )

        context.WP.gen_rand_pool()

        # Build initial waypoints
        wp_init = self._build_initial_waypoints(context)
        vel_wp_init = np.zeros(len(context.WP.act_idxv))
        acc_wp_init = np.zeros(len(context.WP.act_idxv))

        W_stack = None
        for s_rep in range(stack_reps):
            success = self._solve_segment(
                context, s_rep, wp_init, vel_wp_init, acc_wp_init, W_stack
            )
            if not success:
                break
            if s_rep < stack_reps - 1:
                wp_init, W_stack = self._prepare_next_segment(context)

    def _build_initial_waypoints(self, context) -> np.ndarray:
        """Build initial waypoint guess from random pool."""
        rng = np.random.default_rng(100)
        wp_init = np.zeros(len(context.WP.act_idxq))
        for idx in range(len(context.WP.act_idxq)):
            center = (context.WP.lower_q[idx] + context.WP.upper_q[idx]) / 2
            half_range = (context.WP.upper_q[idx] - context.WP.lower_q[idx]) / 2 * 1.0
            q_pool = np.asarray(context.WP.pool_q)[:, idx]
            valid_q = q_pool[
                (q_pool >= (center - half_range)) & (q_pool <= (center + half_range))
            ]
            if valid_q.size == 0:
                raise RuntimeError(
                    f"No pool samples for joint index {idx}"
                )
            wp_init[idx] = float(rng.choice(valid_q))
        return wp_init

    def _generate_feasible_initial_guess(
        self, context, wp_init, vel_wp_init, acc_wp_init
    ):
        """Generate feasible initial guess via random search."""
        from figaroh.utils.pin_interface import calc_torque

        count = 0
        is_constr_violated = True
        max_attempts = context.trajectory_config.get("max_attempts", 500)

        while is_constr_violated and count < max_attempts:
            count += 1
            try:
                wps, vel_wps, acc_wps = context.WP.gen_rand_wp(
                    wp_init, vel_wp_init, acc_wp_init
                )
                tps = np.matrix(
                    [context.trajectory_config["t_s"] * i_wp
                     for i_wp in range(context.trajectory_config["n_wps"])]
                ).transpose()
                t_i, p_i, v_i, a_i = context.WP.get_full_config(
                    context.trajectory_config["freq"], tps, wps, vel_wps, acc_wps
                )
                tau_i = calc_torque(
                    p_i.shape[0], context.robot, p_i, v_i, a_i
                )
                tau_i_np = (
                    np.reshape(tau_i, (v_i.shape[1], v_i.shape[0])).transpose()
                )
                is_constr_violated = context.WP.check_cfg_constraints(
                    p_i, v_i, tau_i_np
                )
            except Exception:
                continue

        return wps, vel_wps, acc_wps, tps, t_i, p_i, v_i, a_i

    def _solve_segment(
        self, context, s_rep, wp_init, vel_wp_init, acc_wp_init, W_stack
    ) -> bool:
        """Solve a single spline segment."""
        try:
            wps, vel_wps, acc_wps, tps, _, p_i, _, _ = (
                self._generate_feasible_initial_guess(
                    context, wp_init, vel_wp_init, acc_wp_init
                )
            )
            tps = (
                context.trajectory_config["t_s"]
                * (context.trajectory_config["n_wps"] - 1)
                * s_rep
                + tps
            )

            problem = context.create_ipopt_problem(
                len(context.active_joints),
                context.trajectory_config["n_wps"],
                p_i.shape[0],
                tps,
                vel_wps,
                acc_wps,
                wp_init,
                vel_wp_init,
                acc_wp_init,
                W_stack,
            )

            success, result_data = problem.solve_with_waypoints(wps)

            if success:
                context.results['T_F'].append(result_data['t_f'])
                context.results['P_F'].append(result_data['p_f'])
                context.results['V_F'].append(result_data['v_f'])
                context.results['A_F'].append(result_data['a_f'])
                context.results['iteration_data'].append(
                    result_data['iter_data']
                )
                return True
            return False
        except Exception as e:
            self.logger.error(f"Error solving segment {s_rep + 1}: {e}")
            return False

    def _prepare_next_segment(
        self, context
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Prepare initial conditions for next segment."""
        last_result = context.results['iteration_data'][-1]
        wp_init = last_result['final_waypoint']
        last_p_f = context.results['P_F'][-1]
        last_v_f = context.results['V_F'][-1]
        last_a_f = context.results['A_F'][-1]
        W_stack = context._stack_base_regressors(
            last_p_f, last_v_f, last_a_f
        )
        return wp_init, W_stack
