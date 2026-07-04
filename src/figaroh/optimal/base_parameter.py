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

import logging
import numpy as np
from typing import Dict, List, Tuple, Any

# Setup logger for this module
logger = logging.getLogger(__name__)
logger.addHandler(logging.NullHandler())

from figaroh.identification.identification_tools import get_standard_parameters
from figaroh.utils.cubic_spline import (
    CubicSpline,
    WaypointsGeneration,
)
from figaroh.tools.qrdecomposition import get_baseIndex
from figaroh.tools.regressor import (
    build_regressor_basic,
    build_regressor_reduced,
    get_index_eliminate,
)
from figaroh.identification.parameter import (
    add_standard_additional_parameters,
    add_custom_parameters
)


class BaseParameterComputer:
    """Handles base parameter computation and indexing."""

    def __init__(self, robot, identif_config, soft_lim_pool):
        self.robot = robot
        self.model = self.robot.model
        self.standard_parameter: list | None = None
        self.identif_config = identif_config
        self.active_joints = identif_config["active_joints"]
        self.soft_lim_pool = soft_lim_pool
        self.logger = logging.getLogger(__name__)

    def compute_base_indices(self) -> Tuple[np.ndarray, np.ndarray]:
        """Compute base parameter indices from random trajectory."""
        self.logger.info(f"Computing base parameter indices from random trajectory")

        try:
            # Generate random trajectory for base parameter computation
            n_wps_r = 100   #TODO 路径点数量（和yaml里的参数有没有关系？？？）
            freq_r = 100    #TODO 和yaml里的参数有没有关系？？？
            # CB_r = CubicSpline(self.robot, n_wps_r, self.active_joints)
            WP_r = WaypointsGeneration(self.robot, n_wps_r, self.active_joints, self.soft_lim_pool)
            
            # 1. 每个joint在关节位置、速度，加速度限位内均匀生成10个候选点
            WP_r.gen_rand_pool()
            
            # 2. 基于步骤1生成的候选点，每个关节生成关节位置/速度/加速度序列（n_wps_r个点），要求相邻点位置/速度/加速度不同。
            #    参数：vel_set_zero=True, acc_set_zero=True,因此生成的轨迹中速度和加速度都为0
            wps_r, vel_wps_r, acc_wps_r = WP_r.gen_rand_wp(vel_set_zero=True, acc_set_zero=True)
            
            # 3. 基于步骤2生成的关节位置/速度/加速度序列，以及tps_r时间序列，针对每个active_joint构造三次样条曲线。
            #    以指定频率freq_r在三次样条曲线上采样，得到采样点上的关节位置/速度(位置一阶导)/加速度(位置二阶导)序列。
            tps_r = np.matrix([0.5 * i for i in range(n_wps_r)]).transpose()    
            t_r, p_r, v_r, a_r = WP_r.get_full_config(freq_r, tps_r, wps_r, vel_wps_r, acc_wps_r)
            WP_r.plot_spline(t_r, p_r, v_r, a_r)      
          
            # 4. 基于步骤3得到的关节位置/速度/加速度序列，以及标准参数，利用QR分解得到基参数索引idx_b和消元参数索引idx_e。
            idx_e, idx_b = self._get_idx_from_random(p_r, v_r, a_r)
            self.logger.info(f"Computed {len(idx_b)} base parameters successfully")

            return idx_e, idx_b

        except Exception as e:
            self.logger.error(f"Error computing base indices: {e}")
            raise

    def _get_idx_from_random(self, q, v, a) -> Tuple[np.ndarray, np.ndarray]:
        """Get indices of eliminate and base parameters."""
        # 堆叠矩阵
        W = build_regressor_basic(self.robot, q, v, a, self.identif_config)
        # 全量惯性参数
        self.standard_parameter = get_standard_parameters(
            self.robot.model, self.identif_config
        )
        # additional parameters can be added in robot-specific subclass
        if (
            self.identif_config.get("has_friction", False)
            or self.identif_config.get("has_actuator_inertia", False)
            or self.identif_config.get("has_joint_offset", False)
        ):
            self.additional_parameters = add_standard_additional_parameters(
                self.model, self.identif_config
            )
            self.standard_parameter.update(self.additional_parameters)

        # Add custom parameters specific to the robot
        if self.identif_config.get("has_custom_parameters", False):
            self.custom_parameters = add_custom_parameters(
                self.model, self.identif_config.get("custom_parameters", {})
            )
            self.standard_parameter.update(self.custom_parameters)
        
        # 基于回归矩阵的信息来判断哪些参数是可辨识的。如果一个参数在给定的运动轨迹下没有足够强的“激发”，
        # 那么它的系数在最小二乘辨识中会非常不确定，甚至可能导致病态问题。通过消除这些弱激发的参数，可
        # 以提高辨识结果的质量和稳定性。
        idx_e_, par_r_ = get_index_eliminate(W, self.standard_parameter, tol_e=0.001)
        # Convert to numpy arrays
        idx_e_ = np.array(idx_e_, dtype=int)
        # 据提供的索引列表，从原始的回归矩阵 W 中删除指定的列 ，从而构建一个简化（或称“缩减”）的回归矩阵
        W_e_ = build_regressor_reduced(W, idx_e_)
        idx_base_ = get_baseIndex(W_e_, par_r_)
        idx_base_ = np.array(idx_base_, dtype=int)

        return idx_e_, idx_base_
