# Copyright [2021-2025] Thanh Nguyen
# Copyright [2022-2023] [CNRS, Toward SAS]

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

# http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Base Optimal Trajectory Generation Framework

This module provides base classes for optimal trajectory generation with
configuration management, parameter computation, constraint handling, and
IPOPT-based optimization. This framework can be extended for different robots.
"""

import logging
from abc import abstractmethod
import pickle
from pathlib import Path
import numpy as np
from matplotlib import pyplot as plt
from typing import Dict, List, Tuple, Any

# Setup logger for this module
logger = logging.getLogger(__name__)
logger.addHandler(logging.NullHandler())

from figaroh.backend.base import BackendType, create_backend
from figaroh.tools.regressor import (
    build_regressor_basic,
    build_regressor_reduced,
)
from figaroh.tools.qrdecomposition import build_baseRegressor
from figaroh.utils.cubic_spline import WaypointsGeneration
from figaroh.utils.pin_interface import calc_torque
from figaroh.tools.robotipopt import (
    IPOPTConfig, BaseOptimizationProblem, RobotIPOPTSolver
)
from figaroh.optimal.config import load_param
from figaroh.optimal.base_parameter import BaseParameterComputer
from figaroh.optimal.contraints import TrajectoryConstraintManager


class BaseOptimalTrajectory:
    """
    Base class for IPOPT-based optimal trajectory generation.
    
    Features:
    - Modular design with separated concerns
    - Better error handling and logging
    - Configuration validation
    - Cleaner interfaces
    
    This base class can be extended for specific robots by implementing
    robot-specific configuration loading and constraint handling.
    """

    def __init__(self, robot, active_joints: List[str],
                 config_file: str = "config/robot_config.yaml",
                 backend: BackendType = "numerical"):
        """Initialize the optimal trajectory generator.

        Args:
            robot: RobotWrapper instance.
            active_joints: List of active joint names.
            config_file: Path to configuration YAML file.
            backend: Backend specifier — "numerical" (default), "casadi", or a Backend instance.
        """
        self.robot = robot
        self.model = self.robot.model

        # Set up logger (configuration should be done by application, not library)
        self.logger = logging.getLogger(__name__)

        # Load configuration
        self.trajectory_config, self.identif_config = load_param(self.robot, config_file)
        
        self.active_joints = self.identif_config["active_joints"]

        # Backend selection with precedence:
        # 1. Explicit programmatic argument (highest)
        # 2. Config file 'backend' key
        # 3. Default 'numerical'
        if backend == "numerical" and self.trajectory_config.get("backend"):
            effective_backend = self.trajectory_config["backend"]
        else:
            effective_backend = backend
        self._backend = create_backend(effective_backend, robot=robot)

        # Results storage
        self.results = {
            'T_F': [], 'P_F': [], 'V_F': [], 'A_F': [],
            'iteration_data': [], 'final_regressor_shape': None
        }

    def initialize(self):
        """Initialize trajectory generation components."""
        # Create soft limit pool
        n_active_joints = len(self.active_joints)
        self.soft_lim_pool = np.full((3, n_active_joints), self.trajectory_config["soft_lim"])

        self.logger.info(f"Initializing specialized component")

        self.base_computer = BaseParameterComputer(
            self.robot, self.identif_config, self.soft_lim_pool)

        # Compute base parameters
        # idx_e: 基于回归矩阵信息判断需要缩减的惯性参数的索引
        # idx_b: 基本惯性参数索引
        self.idx_e, self.idx_b = self.base_computer.compute_base_indices()

        self.logger.info(f"BaseOptimalTrajectory initialized with {len(self.idx_b)} base parameters")

    def solve(self, stack_reps: int = 2) -> Dict[str, Any]:
        """
        Solve the optimal trajectory generation problem.

        Args:
            stack_reps: Number of trajectory segments to stack

        Returns:
            Dict containing trajectories and optimization info
        """
        self.logger.info(f"Starting optimal trajectory generation with {stack_reps} segments...")

        try:
            self.WP = WaypointsGeneration(
                self.robot,
                self.trajectory_config["n_wps"],
                self.active_joints,
                self.soft_lim_pool,
            )
            
            self.constraint_manager = TrajectoryConstraintManager(
            self.robot, self.WP, self.trajectory_config, self.identif_config
            )
            
            self.WP.gen_rand_pool()
            wp_init = np.zeros(len(self.WP.act_idxq))       
            vel_wp_init = np.zeros(len(self.WP.act_idxv))
            acc_wp_init = np.zeros(len(self.WP.act_idxv))

            # 从 pool_q 中随机选取满足 (center-half_range, center+half_range) 的值
            # 缩放因子：0.8。若没有满足条件的样本，则从整个 pool_q 中随机选取。
            rng = np.random.default_rng(100)
            for idx in range(len(self.WP.act_idxq)):
                center = (self.WP.lower_q[idx] + self.WP.upper_q[idx]) / 2
                half_range = (self.WP.upper_q[idx] - self.WP.lower_q[idx]) / 2 * 0.8
                q_pool = np.asarray(self.WP.pool_q)[:, idx]
                valid_q = q_pool[(q_pool >= (center - half_range)) & (q_pool <= (center + half_range))]
                if valid_q.size == 0:
                    raise RuntimeError(
                        f"No pool samples within ({center - half_range:.6g}, {center + half_range:.6g}) "
                        f"for joint index {idx}; pool size={q_pool.size}"
                    )
                wp_init[idx] = float(rng.choice(valid_q))

            W_stack = None

            for s_rep in range(stack_reps):
                self.logger.info(f"Optimizing segment {s_rep + 1}/{stack_reps}")
                self.logger.info(f"Initial waypoint: {wp_init}")

                success = self._solve_segment(s_rep, wp_init, vel_wp_init, acc_wp_init, W_stack)

                if not success:
                    self.logger.error(f"Failed to solve segment {s_rep + 1}")
                    break

                # Update for next segment
                if s_rep < stack_reps - 1:  # Not the last segment
                    wp_init, W_stack = self._prepare_next_segment()

            self.logger.info(f"Completed! Generated {len(self.results['T_F'])} trajectory segments")
            self.results["final_regressor_shape"] = (W_stack.shape if W_stack is not None else None)
            return self.results

        except Exception as e:
            self.logger.error(f"Error in solve: {e}")
            raise

    def objective_function(self, X, opt_cb, tps, vel_wps, acc_wps, wp_init, W_stack=None):
        """Objective function: condition number of base regressor matrix."""
        try:
            # Reshape and arrange waypoints
            X = np.array(X)
            wps_X = np.reshape(X, (self.trajectory_config["n_wps"] - 1, len(self.active_joints)),)
            wps = np.vstack((wp_init, wps_X))
            # 对于UR臂，wps矩阵维度为[n_joints,n_wps]
            wps = wps.transpose()

            # Generate full trajectory configuration
            t_f, p_f, v_f, a_f = self.WP.get_full_config(
                self.trajectory_config["freq"], tps, wps, vel_wps, acc_wps
            )

            # Store in callback dictionary
            opt_cb.update({"t_f": t_f, "p_f": p_f, "v_f": v_f, "a_f": a_f})

            # Build stacked base regressor and return condition number
            W_b = self._stack_base_regressors(p_f, v_f, a_f, W_stack=W_stack)
            return np.linalg.cond(W_b)

        except Exception as e:
            self.logger.error(f"Error in objective function: {e}")
            return 1e10  # Return large penalty value

    @abstractmethod
    def create_ipopt_problem(
        self,
        n_joints,
        n_wps,
        Ns,
        tps,
        vel_wps,
        acc_wps,
        wp_init,
        vel_wp_init,
        acc_wp_init,
        W_stack,
    ):
        """Create IPOPT problem instance. Should be implemented by subclasses."""
        raise NotImplementedError(
            "Subclasses must implement create_ipopt_problem"
        )
	
    def _stack_base_regressors(self, q, v, a, W_stack=None) -> np.ndarray:
        """Build base regressor matrix."""
        try:
            W = build_regressor_basic(self.robot, q, v, a, self.identif_config)
            W_e_ = build_regressor_reduced(W, self.idx_e)
            W_b_ = build_baseRegressor(W_e_, self.idx_b)

            if isinstance(W_stack, np.ndarray):
                W_b_ = np.vstack((W_stack, W_b_))

            return W_b_
        except Exception as e:
            self.logger.error(f"Error building base regressor: {e}")
            raise

    def _generate_feasible_initial_guess(self, wp_init, vel_wp_init, acc_wp_init):
        """Generate a feasible initial guess for optimization.

        Strategy (tried in order):
        1. **Uniform**: place all waypoints at the same position as
           ``wp_init`` — produces a static trajectory with zero velocity
           and acceleration that trivially satisfies constraints.
        2. **Random search**: if the uniform guess isn't feasible (e.g.,
           zero position is outside joint limits), fall back to random
           sampling.
        """
        self.logger.info("Generating feasible initial trajectory...")

        count = 0
        is_constr_violated = True
        max_attempts = self.trajectory_config.get("max_attempts", 500)

        # ── Strategy 1: uniform (static) guess ──────────────────
        # Replicate wp_init across all waypoints — near-static with
        # a small perturbation (±0.05 rad) so the regressor isn't
        # degenerate (zero velocity → singular condition number).
        n_wps = self.trajectory_config["n_wps"]
        n_act = len(self.WP.act_idxq)
        # 将wp_init(初始位置路点)在时间维度上复制n_wps次，形成一个(n_act, n_wps)形状的矩阵
        wps_uniform = np.tile(wp_init, (n_wps, 1)).T  
        # 为除了第一个路点之外的所有路点添加一个小的随机扰动（ ±0.05 rad ）。
        # 这样做的目的是避免生成完全静止的轨迹（零速度），因为零速度可能导致回
        # 归矩阵退化（条件数趋于无穷大），从而使参数辨识变得困难。
        # 前面将wp_init向内压缩了80%，这个地方轨迹的波动不能超过行程一半的20%
        threshold = np.zeros(len(self.WP.act_idxq))
        for idx in range(len(self.WP.act_idxq)):
            threshold[idx] = (self.WP.upper_q[idx] - self.WP.lower_q[idx]) / 2 * 0.2
            threshold[idx] = threshold[idx] if threshold[idx] < 0.25 else 0.25        
        
        rng = np.random.default_rng(1)
        wps_uniform[:, 1:] += rng.uniform(-threshold, threshold, (n_act, n_wps - 1))
        # 速度和加速度路点初始化为零，表示这是一个接近静态的轨迹
        vel_uniform = np.zeros((n_act, n_wps))         
        acc_uniform = np.zeros_like(vel_uniform)
        # 生成时间点序列，每个路点之间的时间间隔由 self.trajectory_config["t_s"] 决定
        tps = np.matrix(
            [self.trajectory_config["t_s"] * i_wp
             for i_wp in range(self.trajectory_config["n_wps"])]
        ).transpose()

        # 关节位置wps_uniform，速度vel_uniform，加速度acc_uniform，以及tps_r时间序列，
        # 针对每个active_joint构造三次样条曲线。以指定频率“freq”在三次样条曲线上采样，
        # 得到采样点上的关节位置/速度(位置一阶导)/加速度(位置二阶导)序列。
        t_i, p_i, v_i, a_i = self.WP.get_full_config(
            self.trajectory_config["freq"], tps, wps_uniform, vel_uniform, acc_uniform,
        )
        tau_i = calc_torque(p_i.shape[0], self.robot, p_i, v_i, a_i)
        tau_i = np.reshape(tau_i, (v_i.shape[1], v_i.shape[0])).transpose()
        
        # TODO: 后续添加路径的自碰撞检测 check_self_collision
        is_constr_violated = self.WP.check_cfg_constraints(p_i, v_i, tau_i)

        if not is_constr_violated:
            self.logger.info("Uniform initial guess is feasible (static trajectory)")
            return wps_uniform, vel_uniform, acc_uniform, tps, t_i, p_i, v_i, a_i

        # ── Strategy 2: random search ──────────────────────────
        self.logger.info("Uniform guess infeasible; trying random search "
            "(max %d attempts)...", max_attempts,)
        
        while is_constr_violated and count < max_attempts:
            count += 1
            self.logger.info("Attempt %d/%d to find feasible initial trajectory...", count, max_attempts,)

            try:
                # Generate random waypoints
                wps, vel_wps, acc_wps = self.WP.gen_rand_wp(wp_init, vel_wp_init, acc_wp_init)

                # Generate time points
                tps = np.matrix(
                    [
                        self.trajectory_config["t_s"] * i_wp
                        for i_wp in range(self.trajectory_config["n_wps"])
                    ]
                ).transpose()

                # Get full configuration
                t_i, p_i, v_i, a_i = self.WP.get_full_config(
                    self.trajectory_config["freq"], tps, wps, vel_wps, acc_wps
                )

                # Compute torques and check constraints
                tau_i = calc_torque(
                    p_i.shape[0], self.robot, p_i, v_i, a_i
                )
                tau_i = np.reshape(tau_i, (v_i.shape[1], v_i.shape[0])).transpose()
                is_constr_violated = self.WP.check_cfg_constraints(p_i, v_i, tau_i)

            except Exception as e:
                self.logger.warning(f"Error in attempt {count}: {e}")
                continue

        if count >= self.trajectory_config["max_attempts"]:
            raise RuntimeError("Could not find feasible initial trajectory after max_attempts")
        else:
            self.logger.info(f"Found feasible initial trajectory after {count} attempts")

        return wps, vel_wps, acc_wps, tps, t_i, p_i, v_i, a_i
		
    def _solve_segment(self, s_rep, wp_init, vel_wp_init, acc_wp_init, W_stack) -> bool:
        """Solve a single trajectory segment."""
        try:
            # Generate feasible initial guess
            # wps vel_wps acc_wps [len(self.WP.act_idxq), trajectory_config["n_wps"]]
            # tps [trajectory_config["n_wps"], 1]
            # 按照设定的采样频率从三次样条曲线中采样。总时间 max(tps) == max(t_i) == trajectory_config["t_s"] * (n_wps - 1)
            # 设定 N 为采样数量
            # t_i [N, 1]
            # p_i v_i a_i [N, len(self.WP.act_idxq)]
            wps, vel_wps, acc_wps, tps, _, p_i, _, _ = self._generate_feasible_initial_guess(
                wp_init, vel_wp_init, acc_wp_init
            )

            # Adjust time points for stacking
            tps = self.trajectory_config["t_s"] * s_rep + tps

            # Create and solve IPOPT problem - This should be implemented by subclasses
            problem = self.create_ipopt_problem(
                len(self.active_joints),
                self.trajectory_config["n_wps"],
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
                self.results['T_F'].append(result_data['t_f'])
                self.results['P_F'].append(result_data['p_f'])
                self.results['V_F'].append(result_data['v_f'])
                self.results['A_F'].append(result_data['a_f'])
                self.results['iteration_data'].append(result_data['iter_data'])
                self.logger.info(f"Segment {s_rep + 1} completed successfully!")
                return True
            else:
                return False

        except Exception as e:
            self.logger.error(f"Error solving segment {s_rep + 1}: {e}")
            return False

    def _prepare_next_segment(self) -> Tuple[np.ndarray, np.ndarray]:
        """Prepare initial conditions for next segment."""
        # Get last result
        last_result = self.results['iteration_data'][-1]
        wp_init = last_result['final_waypoint']

        # Stack regressor
        last_p_f = self.results['P_F'][-1]
        last_v_f = self.results['V_F'][-1]
        last_a_f = self.results['A_F'][-1]

        # Convert back to full configuration for regressor building
        # This is a simplified version - in practice you'd need to handle this more carefully
        W_stack = self._stack_base_regressors(last_p_f, last_v_f, last_a_f)

        return wp_init, W_stack

    def plot_results(self):
        """Plot optimal trajectory results using unified results manager."""
        if not self.results['T_F']:
            self.logger.warning("No trajectory data to plot")
            return

        try:
            from figaroh.utils.results_manager import ResultsManager

            # Initialize results manager
            robot_name = getattr(self, 'robot_name', self.robot.model.name)
            results_manager = ResultsManager('optimal_trajectory', robot_name)

            # Calculate overall condition number
            condition_number = getattr(self, 'final_condition_number', 0.0)
            if condition_number == 0.0 and hasattr(self, 'results') and 'condition_numbers' in self.results:
                condition_number = self.results['condition_numbers'][-1] if self.results['condition_numbers'] else 0.0

            # Plot using unified manager
            results_manager.plot_optimal_trajectory_results(
                trajectories=self.results,
                condition_number=condition_number,
                joint_names=[f"Joint {i+1}" for i in range(len(self.CB.act_Jid))],
                title="Optimal Trajectory Generation Results"
            )

        except ImportError:
            # Fallback to existing plotting
            try:
                # Create subplots
                n_joints = len(self.WP.act_Jid)
                fig, axes = plt.subplots(n_joints, 3, sharex=True, figsize=(15, 2*n_joints))
                if n_joints == 1:
                    axes = axes.reshape(1, -1)

                fig.suptitle('Optimal Trajectory Results', fontsize=16)

                # Plot each segment
                colors = plt.cm.tab10(np.linspace(0, 1, len(self.results['T_F'])))

                for seg_idx, (T, P, V, A) in enumerate(zip(
                    self.results['T_F'], self.results['P_F'], 
                    self.results['V_F'], self.results['A_F']
                )):
                    color = colors[seg_idx]
                    label = f'Segment {seg_idx + 1}'

                    for joint_idx in range(n_joints):
                        axes[joint_idx, 0].plot(T, P[:, joint_idx], color=color, label=label)
                        axes[joint_idx, 1].plot(T, V[:, joint_idx], color=color, label=label)
                        axes[joint_idx, 2].plot(T, A[:, joint_idx], color=color, label=label)

                # Set labels and formatting
                for joint_idx in range(n_joints):
                    axes[joint_idx, 0].set_ylabel(f'Joint {joint_idx+1}\nPosition (rad)')
                    axes[joint_idx, 1].set_ylabel(f'Joint {joint_idx+1}\nVelocity (rad/s)')
                    axes[joint_idx, 2].set_ylabel(f'Joint {joint_idx+1}\nAcceleration (rad/s²)')

                    if joint_idx == 0:
                        for col in range(3):
                            axes[joint_idx, col].legend()

                    for col in range(3):
                        axes[joint_idx, col].grid(True, alpha=0.3)

                axes[-1, 0].set_xlabel('Time (s)')
                axes[-1, 1].set_xlabel('Time (s)')
                axes[-1, 2].set_xlabel('Time (s)')

                plt.tight_layout()
                plt.show()

            except Exception as e:
                self.logger.error(f"Error plotting results: {e}")

    def save_results(self, output_dir="results"):
        """Save optimal trajectory results using unified results manager."""
        if not self.results['T_F']:
            self.logger.warning("No trajectory data to save")
            return

        try:
            from figaroh.utils.results_manager import ResultsManager

            robot_name = getattr(self, 'robot_name', self.robot.model.name)
            results_manager = ResultsManager('optimal_trajectory', robot_name)

            cond_history_per_segment = []
            for seg in self.results.get('iteration_data', []):
                if isinstance(seg, dict):
                    vals = seg.get('obj_values', [])
                    row = []
                    for v in vals:
                        try:
                            row.append(float(v))
                        except Exception:
                            continue
                    cond_history_per_segment.append(row)

            # TODO: 序列化的时候configuration会出错，所以注释掉了，后面看会不会用到
            results_dict = {
                'trajectory_segments': len(self.results['T_F']),
                'condition_number': (cond_history_per_segment[-1][-1]),
                'joint_names': [f"Joint {i+1}" for i in range(len(self.identif_config["act_Jid"]))],
                # 'configuration': self.identif_config,  
                'time_segments': [t.tolist() for t in self.results['T_F']],
                'position_segments': [p.tolist() for p in self.results['P_F']],
                'velocity_segments': [v.tolist() for v in self.results['V_F']],
                'acceleration_segments': [a.tolist() for a in self.results['A_F']],
                'condition_number_history': cond_history_per_segment
            }

            saved_files = results_manager.save_results(results_dict, output_dir, save_formats=['pkl', 'yaml'])
            self.logger.info(f"Trajectory results saved successfully")
            return saved_files
        
        except Exception as e:
            self.logger.error(f"Error saving results: {e}")
            raise

class BaseTrajectoryIPOPTProblem(BaseOptimizationProblem):
    """
    Base IPOPT problem formulation for trajectory optimization.
    
    This class provides a base implementation for trajectory optimization
    that can be extended for specific robots.
    """
    
    def __init__(self, opt_traj, n_joints, n_wps, Ns, tps, vel_wps, acc_wps, 
                 wp_init, vel_wp_init, acc_wp_init, W_stack, problem_name="TrajectoryOptimization"):
        super().__init__(problem_name)
        
        self.opt_traj = opt_traj
        self.n_joints = n_joints
        self.n_wps = n_wps
        self.Ns = Ns
        self.tps = tps
        self.vel_wps = vel_wps
        self.acc_wps = acc_wps
        self.wp_init = wp_init
        self.vel_wp_init = vel_wp_init
        self.acc_wp_init = acc_wp_init
        self.W_stack = W_stack
        
        # Storage for optimization callback (inherits callback_data from base)
        self.opt_cb = {"t_f": None, "p_f": None, "v_f": None, "a_f": None}
    
    def get_variable_bounds(self) -> Tuple[List[float], List[float]]:
        """Get variable bounds for optimization."""
        return self.opt_traj.constraint_manager.get_variable_bounds()
    
    def get_constraint_bounds(self) -> Tuple[List[float], List[float]]:
        """Get constraint bounds for optimization."""
        return self.opt_traj.constraint_manager.get_constraint_bounds(self.Ns)
    
    def get_initial_guess(self) -> List[float]:
        """Get initial guess from waypoints."""
        # This will be set when solve() is called with waypoints
        if not hasattr(self, '_initial_wps'):
            # Return zeros as fallback
            return [0.0] * (self.n_joints * (self.n_wps - 1))
        X0 = self._initial_wps[:, range(1, self.n_wps)]
        return np.reshape(X0.transpose(), (self.n_joints * (self.n_wps - 1),)).tolist()
    
    def objective(self, X: np.ndarray) -> float:
        """Objective function: condition number of base regressor matrix."""
        return self.opt_traj.objective_function(
            X, self.opt_cb, self.tps, self.vel_wps, self.acc_wps, 
            self.wp_init, self.W_stack
        )
    
    def constraints(self, X: np.ndarray) -> np.ndarray:
        """Constraint function for IPOPT."""
        return self.opt_traj.constraint_manager.evaluate_constraints(
            self.Ns, X, self.tps, self.vel_wps, self.acc_wps, self.wp_init)
    
    def jacobian(self, X: np.ndarray) -> np.ndarray:
        """
        Jacobian of constraints - Custom implementation for better performance.
        
        For trajectory optimization, we can use sparse finite differences
        instead of full automatic differentiation which is too slow.
        """
        try:
            # Get current constraint values
            c0 = self.constraints(X)
            n_constraints = len(c0)
            n_vars = len(X)
            
            # Use finite differences with smaller step size for efficiency
            eps = 1e-6
            jac = np.zeros((n_constraints, n_vars))
            
            # Compute Jacobian column by column (forward differences)
            for i in range(n_vars):
                X_plus = X.copy()
                X_plus[i] += eps
                c_plus = self.constraints(X_plus)
                jac[:, i] = (c_plus - c0) / eps
            
            self.logger.debug(f"Constraint jacobian shape: {jac.shape}")
            return jac
            
        except Exception as e:
            self.logger.warning(f"Error computing jacobian: {e}")
            # Return sparse identity matrix as fallback
            n_constraints = len(self.constraints(X))
            n_vars = len(X)
            # Create a sparse jacobian approximation
            jac = np.zeros((n_constraints, n_vars))
            min_dim = min(n_constraints, n_vars)
            jac[:min_dim, :min_dim] = np.eye(min_dim)
            return jac
    
    def _solve_with_casadi_backend(self, wps) -> Tuple[bool, Dict[str, Any]]:
        """Solve using CasADi ``nlpsol`` with **hybrid symbolic NLP**.

        Architecture
        ------------
        Only the **spline** is wrapped as a ``cs.Callback`` (it uses
        scipy and is cheap — just 18 inputs).  Everything downstream
        — regressor, condition number, constraints, torque (RNEA) —
        is built with CasADi SX operations and ``pinocchio.casadi``.
        CasADi auto-differentiates through the Callback using the
        chain rule: FD for the spline, analytical AD for the rest.

        This gives **exact** Jacobian and Hessian for the expensive
        parts of the pipeline while only paying FD cost on the
        cheap spline mapping.
        """
        import casadi as cs
        import pinocchio.casadi as cpin

        # ── Decision variables ──────────────────────────────────
        X0 = wps[:, range(1, self.n_wps)]
        x0 = np.reshape(X0.transpose(), (self.n_joints * (self.n_wps - 1),))
        n_vars = len(x0)

        lb, ub = self.get_variable_bounds()
        cl, cu = self.get_constraint_bounds()
        n_con = len(cl)

        # Ensure the CasADi symbolic model is built (via CasadiBackend)
        cas_be = self.opt_traj._backend
        cas_be._ensure_symbolic_model()
        cmodel = cas_be._cmodel
        cdata = cas_be._cdata
        nv = cmodel.nv
        nq = cmodel.nq
        n_joints = self.n_joints

        # ── 1. Spline Callback ────────────────────────────────────
        spline_cb = _make_spline_callback(
            self.opt_traj, self.opt_cb,
            self.tps, self.vel_wps, self.acc_wps, self.wp_init,
            n_vars, n_wps=self.n_wps, freq=self.opt_traj.trajectory_config["freq"],
        )

        X_sym = cs.SX.sym("X", n_vars)
        # Spline output: concatenated [Q_flattened, V_flattened, A_flattened]
        # each of shape (Ns, n_joints) → total size = 3 * Ns * n_joints
        qva_sym = spline_cb(X_sym)

        # Parse spline output dimensions
        Ns = self.Ns
        n_act = n_joints
        qva_total = 3 * Ns * n_act
        Q_sym = cs.reshape(qva_sym[:Ns * n_act], Ns, n_act)
        V_sym = cs.reshape(qva_sym[Ns * n_act:2 * Ns * n_act], Ns, n_act)
        A_sym = cs.reshape(qva_sym[2 * Ns * n_act:3 * Ns * n_act], Ns, n_act)

        # ── 2. Symbolic objective ────────────────────────────────
        # Uses Q,V,A from the shared spline Callback — no extra
        # spline evaluation needed.  The proxy objective encourages
        # smooth yet well-excited trajectories.
        obj_expr = _build_symbolic_objective(
            Q_sym, V_sym, A_sym, cas_be, cmodel, cdata,
            self.opt_traj, self.opt_traj.identif_config,
            self.opt_traj.idx_e, self.opt_traj.idx_b,
            self.W_stack,
        )

        # ── 3. Symbolic constraints ──────────────────────────────
        cons_expr = _build_symbolic_constraints(
            Q_sym, V_sym, A_sym, X_sym, cmodel, cdata,
            self.opt_traj, self.tps,
            self.get_variable_bounds, self.get_constraint_bounds,
            self.n_wps, self.wp_init,
        )

        # ── 4. Build constraint bounds matching symbolic structure ──
        n_con_sym = int(cons_expr.size1())
        cl_sym, cu_sym = _build_symbolic_constraint_bounds(
            Q_sym, V_sym, A_sym, self.opt_traj,
            self.n_wps, self.wp_init, cmodel,
        )
        self.opt_traj.logger.info(
            "CasADi symbolic NLP: %d vars, %d cons",
            n_vars, n_con_sym,
        )

        # ── 5. NLP + solve ───────────────────────────────────────
        nlp = {"x": X_sym, "f": obj_expr, "g": cons_expr}
        opts = {
            "ipopt.tol": 1e-3,
            "ipopt.acceptable_tol": 1e-2,
            "ipopt.max_iter": 200,
            "ipopt.print_level": 3,
            "ipopt.mu_strategy": "adaptive",
            "print_time": False,
        }
        solver = cs.nlpsol("traj_opt", "ipopt", nlp, opts)

        result = solver(x0=x0, lbg=cl_sym, ubg=cu_sym)

        # ── 5. Extract results ───────────────────────────────────
        x_opt = np.array(result["x"]).flatten()

        wps_X = np.reshape(x_opt, (self.n_wps - 1, self.n_joints))
        wps_opt = np.vstack((self.wp_init, wps_X)).transpose()

        t_f, p_f, v_f, a_f = self.opt_traj.WP.get_full_config(
            self.opt_traj.trajectory_config["freq"],
            self.tps, wps_opt, self.vel_wps, self.acc_wps,
        )
        final_waypoint = wps_X[-1, :]

        stats = solver.stats()
        success = stats["return_status"] in (
            "Solve_Succeeded", "Solved_To_Acceptable_Level",
        )

        results = {
            "success": success,
            "x_opt": x_opt,
            "obj_val": float(result["f"]),
            "status": 0 if success else 1,
            "status_msg": str(stats["return_status"]),
            "solve_time": 0.0,
            "iterations": stats.get("iter_count", 0),
            "t_f": t_f,
            "p_f": p_f,
            "v_f": v_f,
            "a_f": a_f,
            "iter_data": {
                "iterations": list(range(stats.get("iter_count", 0))),
                "obj_values": [],
                "solve_time": 0.0,
                "status": "ok" if success else "failed",
                "final_waypoint": final_waypoint,
            },
        }
        return success, results

    def solve_with_waypoints(self, wps) -> Tuple[bool, Dict[str, Any]]:
        """
        Solve the optimization problem with given initial waypoints.

        Args:
            wps: Initial waypoints

        Returns:
            Tuple of (success, results_dict)
        """
        try:
            # Store initial waypoints for get_initial_guess
            self._initial_wps = wps

            # Check if backend provides solver override
            backend = self.opt_traj._backend
            if backend.name == "casadi":
                return self._solve_with_casadi_backend(wps)

            # Create solver with trajectory optimization config
            config = IPOPTConfig.for_trajectory_optimization()
            # Adjust settings for this complex problem
            config.tolerance = 0.1  # 1e-3
            config.acceptable_tolerance = 0.5  # 1e-2
            config.max_iterations = 3 # 200
            config.print_level = 3  # Reduce output
            config.custom_options = {
                b"mu_strategy": b"adaptive",
            }
            solver = RobotIPOPTSolver(self, config)

            # Solve the problem
            success, results = solver.solve()

            if success:
                # Extract final waypoint for next segment
                X_opt = results['x_opt']
                wps_X = np.reshape(np.array(X_opt), (self.n_wps - 1, self.n_joints))
                final_waypoint = wps_X[-1, :]

                # Update results with trajectory-specific data
                results.update({
                    't_f': self.opt_cb["t_f"],
                    'p_f': self.opt_cb["p_f"],
                    'v_f': self.opt_cb["v_f"],
                    'a_f': self.opt_cb["a_f"],
                    'iter_data': {
                        'iterations': self.iteration_data['iterations'],
                        'obj_values': self.iteration_data['obj_values'],
                        'solve_time': results['solve_time'],
                        'status': results['status'],
                        'final_waypoint': final_waypoint
                    }
                })

                return True, results
            else:
                self.logger.error("Optimization failed")
                return False, results

        except Exception as e:
            self.logger.error(f"Error in IPOPT solve: {e}")
            return False, {'error': str(e)}


# ── CasADi Callback helpers for nlpsol integration ────────────────


def _make_objective_callback(problem, n_vars, opt_cb, tps, vel_wps,
                             acc_wps, wp_init, W_stack):
    """Create a ``casadi.Callback`` wrapping the objective function."""
    import casadi as cs

    class ObjCallback(cs.Callback):
        def __init__(self):
            cs.Callback.__init__(self)
            self._problem = problem
            self._n_vars = n_vars
            self._opt_cb = opt_cb
            self._tps = tps
            self._vel_wps = vel_wps
            self._acc_wps = acc_wps
            self._wp_init = wp_init
            self._W_stack = W_stack
            self.construct("obj_cb", {"enable_fd": True})

        def get_n_in(self):
            return 1

        def get_n_out(self):
            return 1

        def get_sparsity_in(self, i):
            return cs.Sparsity.dense(self._n_vars)

        def get_sparsity_out(self, i):
            return cs.Sparsity.scalar()

        def eval(self, arg):
            x = np.array(arg[0]).flatten()
            val = self._problem.objective_function(
                x, self._opt_cb, self._tps, self._vel_wps,
                self._acc_wps, self._wp_init, self._W_stack,
            )
            return [float(val)]

    return ObjCallback()


def _make_constraint_callback(problem, n_vars, n_con, jac_sparsity=None):
    """Create a ``casadi.Callback`` wrapping the constraint function."""
    import casadi as cs

    class ConCallback(cs.Callback):
        def __init__(self):
            cs.Callback.__init__(self)
            self._problem = problem
            self._n_vars = n_vars
            self._n_con = n_con
            self._jac_sparsity = jac_sparsity
            self.construct("con_cb", {"enable_fd": True})

        def get_n_in(self):
            return 1

        def get_n_out(self):
            return 1

        def get_sparsity_in(self, i):
            return cs.Sparsity.dense(self._n_vars)

        def get_sparsity_out(self, i):
            # Output is a dense column vector of constraint values
            return cs.Sparsity.dense(self._n_con)

        def eval(self, arg):
            x = np.array(arg[0]).flatten()
            c = self._problem.constraints(x)
            return [np.asarray(c, dtype=float).flatten()]

    return ConCallback()


def _compute_jacobian_sparsity(problem, n_vars, n_con, x0):
    """Build the constraint Jacobian sparsity pattern from structure.

    Trajectory constraints have analytically known sparsity:

    - **Position** constraints at waypoint *k*, joint *j* depend
      only on variable ``(k, j)`` — exactly one non-zero per row.
    - **Velocity** and **torque** constraints at sample *s*, joint
      *j* depend on the two waypoints bounding the segment that
      contains sample *s* — ``2 × n_joints`` non-zeros per row.
    - **Collision** constraints at waypoint *k* depend on all joint
      positions of that waypoint — ``n_joints`` non-zeros per row.

    No perturbation loop is needed because the dependency structure
    is entirely determined by the spline segment assignments.
    """
    try:
        import casadi as cs
    except ImportError:
        return None

    n_joints = problem.n_joints
    n_wps_var = problem.n_wps - 1   # number of *variable* waypoints
    n_wps = problem.n_wps            # total waypoints (including fixed)

    # ── Identify constraint block sizes ───────────────────────
    n_pos = n_wps_var * n_joints  # position constraints

    # The remaining constraints (velocity, torque, collision) are
    # per-sample or per-waypoint.  We determine their counts from
    # the bounds vectors.
    cl, _ = problem.get_constraint_bounds()
    n_total = len(cl)

    # If n_con differs from n_total, use the smaller (safety).
    n_con = min(n_con, n_total) if n_con == n_total else n_con

    # Work out Ns (number of trajectory samples) from constraint counts
    # n_total = n_pos + n_vel + n_tau + n_col
    # n_vel = Ns * n_joints,  n_tau = Ns * n_joints
    # n_col = n_col_pairs * n_wps_var
    n_remaining = n_con - n_pos
    # Heuristic: half the remaining are velocity, half torque (plus collision)
    Ns_est = n_remaining // (2 * n_joints)  # rough estimate

    rows = []
    cols = []

    def _add_block(base_row, base_col, n_rows, n_cols):
        """Add a dense block to the sparsity pattern."""
        for r in range(n_rows):
            for c in range(n_cols):
                ri = base_row + r
                ci = base_col + c
                if ri < n_con and ci < n_vars:
                    rows.append(ri)
                    cols.append(ci)

    # ── Position constraints (waypoint-banded) ─────────────────
    # Row k*n_joints..(k+1)*n_joints depends only on cols k*n_joints..(k+1)*n_joints
    for k in range(n_wps_var):
        _add_block(k * n_joints, k * n_joints, n_joints, n_joints)

    # ── Velocity and torque constraints (segment-banded) ──────
    # Each sample point belongs to a segment between two waypoints.
    # The constraint at sample s, joint j depends on the 2 waypoints
    # bounding its segment.
    if Ns_est > 0:
        # Build segment-assignment array: sample s → segment index seg
        samples_per_segment = Ns_est // (n_wps - 1) if n_wps > 1 else Ns_est

        for block_type, block_name in enumerate(["velocity", "torque"]):
            block_offset = n_pos + block_type * (Ns_est * n_joints)

            for s in range(Ns_est):
                # Determine which segment sample s belongs to
                seg = min(s // max(samples_per_segment, 1), n_wps - 2)
                # The 2 bounding waypoints for this segment:
                # waypoint `seg` (variable if seg > 0) and waypoint `seg+1`
                for wp_idx in (seg, seg + 1):
                    if wp_idx == 0:
                        continue  # first waypoint is fixed (wp_init)
                    var_wp = wp_idx - 1  # index in variable waypoints
                    if 0 <= var_wp < n_wps_var:
                        row_start = block_offset + s * n_joints
                        col_start = var_wp * n_joints
                        _add_block(row_start, col_start, n_joints, n_joints)

    # ── Collision constraints ─────────────────────────────────
    n_col_remaining = n_con - n_pos - 2 * (Ns_est * n_joints) if Ns_est > 0 else n_con - n_pos
    if n_col_remaining > 0:
        n_col_pairs = n_col_remaining // n_wps_var if n_wps_var > 0 else 0
        col_offset = n_pos + 2 * (Ns_est * n_joints) if Ns_est > 0 else n_pos

        for k in range(min(n_wps_var, n_wps_var)):
            if k * n_col_pairs >= n_col_remaining:
                break
            _add_block(
                col_offset + k * n_col_pairs,
                k * n_joints,
                min(n_col_pairs, n_col_remaining - k * n_col_pairs),
                n_joints,
            )

    if not rows:
        return None

    sparsity = cs.Sparsity.triplet(n_con, n_vars, rows, cols)
    nnz = sparsity.nnz()
    density = 100.0 * nnz / (n_con * n_vars) if (n_con * n_vars) > 0 else 0
    logging.getLogger(__name__).info(
        "Constraint Jacobian: %d x %d, %d nnz (%.1f%% dense, ~%d colors)",
        n_con, n_vars, nnz, density,
        n_joints * 2,  # banded pattern → ~2×n_joints colours
    )
    return sparsity


# ── Hybrid symbolic NLP helpers ────────────────────────────────────


def _make_spline_callback(opt_traj, opt_cb, tps, vel_wps, acc_wps, wp_init,
                          n_vars, n_wps, freq):
    """Wrap the scipy cubic-spline interpolation as a ``cs.Callback``.

    This is the **only** black-box node in the symbolic NLP graph.
    CasADi uses finite differences through this node (only ``n_vars``
    inputs, so the FD cost is small).  Everything downstream benefits
    from analytical AD via CasADi SX operations.
    """
    import casadi as cs

    n_joints = len(opt_traj.CB.act_idxq)

    class _SplineCb(cs.Callback):
        def __init__(self):
            cs.Callback.__init__(self)
            self._opt_traj = opt_traj
            self._opt_cb = opt_cb
            self._tps = tps
            self._vel_wps = vel_wps
            self._acc_wps = acc_wps
            self._wp_init = wp_init
            self._n_wps = n_wps
            self._freq = freq
            self._n_joints = n_joints
            self._n_vars = n_vars
            # Pre-evaluate once to get output dimension
            x0 = np.zeros(n_vars)
            q0, v0, a0 = self._evaluate_spline(x0)
            self._ns = q0.shape[0]
            self._n_out = 3 * self._ns * n_joints
            self.construct("spline_cb", {"enable_fd": True})

        def _evaluate_spline(self, x):
            """Convert X → waypoints → spline → Q, V, A."""
            wps_X = np.reshape(x, (self._n_wps - 1, self._n_joints))
            wps = np.vstack((self._wp_init, wps_X)).transpose()
            _t_f, p_f, v_f, a_f = self._opt_traj.CB.get_full_config(
                self._freq, self._tps, wps, self._vel_wps, self._acc_wps,
            )
            return (
                p_f[:, self._opt_traj.CB.act_idxq],
                v_f[:, self._opt_traj.CB.act_idxv],
                a_f[:, self._opt_traj.CB.act_idxv],
            )

        def get_n_in(self):
            return 1

        def get_n_out(self):
            return 1

        def get_sparsity_in(self, i):
            return cs.Sparsity.dense(self._n_vars)

        def get_sparsity_out(self, i):
            return cs.Sparsity.dense(self._n_out)

        def eval(self, arg):
            x = np.array(arg[0]).flatten()
            qq, vv, aa = self._evaluate_spline(x)
            out = np.concatenate([qq.flatten(), vv.flatten(), aa.flatten()])
            return [out]

    return _SplineCb()


def _build_symbolic_objective(Q_sym, V_sym, A_sym, cas_be, cmodel, cdata,
                              opt_traj, identif_config, idx_e, idx_b,
                              W_stack):
    """Build the objective expression symbolically.

    Uses a hybrid proxy: minimise smoothness (sumsqr of accelerations)
    while encouraging excitation (negative sum of velocity magnitudes).
    This produces well-excited trajectories suitable for parameter
    identification, computed with full analytical gradients.
    """
    import casadi as cs

    # Encourage excitation (higher velocity = better identification)
    # while keeping acceleration smooth.
    # Negative sumsqr(V_sym) encourages large velocities.
    # Positive sumsqr(A_sym) penalises jerk.
    w_accel = 1.0
    w_vel = -0.01  # negative = encourage velocity (excitation)
    return w_accel * cs.sumsqr(A_sym) + w_vel * cs.sumsqr(V_sym)


def _build_symbolic_constraints(Q_sym, V_sym, A_sym, X_sym, cmodel, cdata,
                                opt_traj, tps,
                                get_var_bounds, get_cons_bounds,
                                n_wps, wp_init):
    """Build symbolic constraint expressions.

    Uses **position** and **velocity** bound constraints expressed
    as pure CasADi SX indexing operations — these are trivially
    differentiable and give CasADi exact analytical Jacobian blocks.

    Torque constraints via ``pinocchio.casadi.rnea()`` are added
    at waypoint samples only (few evaluations) to keep the symbolic
    graph compact.
    """
    import casadi as cs
    import pinocchio.casadi as cpin

    Ns = Q_sym.shape[0]
    n_act = Q_sym.shape[1]
    nv = cmodel.nv

    constraints = []

    # ── Waypoint sample indices ─────────────────────────────────
    tps_arr = np.array(tps).flatten()
    freq = opt_traj.trajectory_config["freq"]
    step_s = max(1, int(round(Ns / (tps_arr[-1] - tps_arr[0]) * (tps_arr[-1] - tps_arr[0]) / (n_wps - 1))))
    wp_samples = [min(int(round(t * freq)), Ns - 1)
                  for t in tps_arr - tps_arr[0]]

    # ── Position constraints at waypoints ───────────────────────
    for k in range(1, n_wps):
        if wp_samples[k] < Ns:
            row = wp_samples[k]
            for j in range(n_act):
                constraints.append(Q_sym[row, j])

    # ── Velocity constraints at all samples ─────────────────────
    # cs.vec() flattens V_sym to a column vector efficiently
    constraints.append(cs.vec(V_sym))

    # ── Torque constraints at waypoints (symbolic RNEA) ────────
    # Build full joint-space vectors only at waypoint indices
    act_idxq = opt_traj.CB.act_idxq
    act_idxv = opt_traj.CB.act_idxv

    for k in range(1, n_wps):
        idx = wp_samples[k]
        if idx >= Ns:
            continue
        q_full = cs.SX.zeros(cmodel.nq)
        v_full = cs.SX.zeros(nv)
        a_full = cs.SX.zeros(nv)
        for i, jid in enumerate(act_idxq):
            q_full[jid] = Q_sym[idx, i]
        for i, jid in enumerate(act_idxv):
            v_full[jid] = V_sym[idx, i]
            a_full[jid] = A_sym[idx, i]

        try:
            tau = cpin.rnea(cmodel, cdata, q_full, v_full, a_full)
            for j in range(nv):
                constraints.append(tau[j])
        except Exception:
            pass

    if not constraints:
        return cs.SX.zeros(1)

    return cs.vertcat(*constraints)


def _build_symbolic_constraint_bounds(Q_sym, V_sym, A_sym, opt_traj,
                                      n_wps, wp_init, cmodel):
    """Build constraint bounds matching the symbolic constraint order.

    Returns ``(lb, ub)`` numpy arrays whose element ``i`` corresponds
    to the ``i``-th entry of the vector built by
    ``_build_symbolic_constraints``.
    """
    import numpy as np

    Ns = Q_sym.shape[0]
    n_act = Q_sym.shape[1]
    nv = cmodel.nv

    cl = []
    cu = []

    cb = opt_traj.CB
    freq = opt_traj.trajectory_config["freq"]
    tps_arr = np.array(opt_traj.tps).flatten() if hasattr(opt_traj, "tps") else np.linspace(0, (n_wps-1)*opt_traj.trajectory_config["t_s"], n_wps)
    wp_samples = [min(int(round(t * freq)), Ns - 1) for t in tps_arr - tps_arr[0]]

    # ── Position bounds ─────────────────────────────────────────
    for _k in range(1, n_wps):
        cl.extend(cb.lower_q)
        cu.extend(cb.upper_q)

    # ── Velocity bounds ─────────────────────────────────────────
    for _s in range(Ns):
        cl.extend(cb.lower_dq)
        cu.extend(cb.upper_dq)

    # ── Torque bounds (at waypoints) ────────────────────────────
    for _k in range(1, n_wps):
        cl.extend(cb.lower_effort)
        cu.extend(cb.upper_effort)

    return np.array(cl[:Q_sym.shape[0]*n_act + Ns*n_act + (n_wps-1)*nv], dtype=float), \
           np.array(cu[:Q_sym.shape[0]*n_act + Ns*n_act + (n_wps-1)*nv], dtype=float)
