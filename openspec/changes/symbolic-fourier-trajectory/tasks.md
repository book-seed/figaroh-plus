## 1. 环境依赖与验证

- [ ] 1.1 检查 pixi 环境中 CasADi 版本及 IPOPT 可用性，确认 `pinocchio.casadi` 绑定正常
- [ ] 1.2 安装/验证 HSL 线性求解器库（`libhsl.so` 或 `libcoinhsl.so`），确认 IPOPT 可调用 `ma57`
- [ ] 1.3 编写环境验证脚本，输出 CasADi、IPOPT、HSL、pinocchio.casadi 各组件状态
- [ ] 1.4 更新 `pixi.toml` casadi feature 依赖声明（如有缺失的 HSL/IPOPT 包）

## 2. 配置解析

- [ ] 2.1 在 `src/figaroh/optimal/config.py` 中新增字段解析：`trajectory_type`（默认 `"spline"`）、`fourier_frequency`（默认 `null` → `2π/T`）、`n_samples`（默认 200）、`n_harmonics`（默认 5）、`reg_lambda`（默认 1e-6）、`tanh_alpha_opt`（默认 10）、`tanh_alpha_id`（默认 100）
- [ ] 2.2 添加配置验证：`trajectory_type` 只接受 `"spline"` 或 `"fourier"`，拒绝非法值
- [ ] 2.3 编写配置解析单元测试：缺省字段走默认值、非法值报错、合法值正确覆盖

## 3. 轨迹模块抽象

- [ ] 3.1 新建 `src/figaroh/utils/base_trajectory.py`，定义 `BaseTrajectory` 抽象基类（接口：生成 q/v/a、约束检查、可视化）
- [ ] 3.2 重构 `CubicSpline`/`WaypointsGeneration` 为 `CubicSplineTrajectory(BaseTrajectory)`，保持原有接口不变
- [ ] 3.3 新建 `src/figaroh/utils/fourier_trajectory.py`，实现 `FourierTrajectory(BaseTrajectory)`，包含：
  - 傅里叶级数解析参数表达式（numpy 求值，用于数值后端和系数初始化）
  - 傅里叶级数 CasADi SX 符号表达式构建方法（用于 CasADi NLP）

## 4. CasADi 符号化管线核心

- [ ] 4.1 移植 `CasadiBackend._ensure_symbolic_model()` 到傅里叶优化路径：`cpin.Model` + `cmodel.createData()` + 磁盘缓存
- [ ] 4.2 实现傅里叶轨迹的 CasADi SX 表达式：q(t, coeffs)、v(t, coeffs) = ∂q/∂t、a(t, coeffs) = ∂²q/∂t²
- [ ] 4.3 实现符号化完整回归矩阵 W 构建：`cpin.computeJointTorqueRegressor` → 追加摩擦/惯量/偏置列（`tanh(α·v)`）→ QR `idx_b` 列选择 → `W_b`
- [ ] 4.4 实现 D-最优目标函数 `-log det(W_bᵀW_b/N_s + λI)` 为 CasADi SX 表达式
- [ ] 4.5 实现符号化约束：位置限位 `q_lower ≤ q ≤ q_upper`、速度限位 `v_lower ≤ v ≤ v_upper`、力矩限位 `cpin.rnea` 约束，均 CasADi SX
- [ ] 4.6 确保构建顺序：先构建符号 W(q,v,a) 函数 + CasADi AD 求导，再 `map` 到采样点并行求值

## 5. 系数初始化

- [ ] 5.1 实现智能初始化策略：偏移项 = 关节中位值，谐波系数 = 限位范围的 5-10% 小随机振幅
- [ ] 5.2 实现初始轨迹约束验证：检查初始傅里叶系数生成的轨迹是否违反关节限位/力矩约束，违反时缩小振幅重试
- [ ] 5.3 添加初始化失败处理：超过最大重试次数时抛出有意义错误信息

## 6. IPOPT 求解器集成

- [ ] 6.1 构建 CasADi `nlpsol("ipopt", nlp)` NLP 定义（变量边界、约束边界、目标、约束）
- [ ] 6.2 配置 IPOPT 选项：`linear_solver: ma57`（回退 `mumps`）、`tol: 1e-6`、`max_iter: 500`、`mu_strategy: adaptive`
- [ ] 6.3 实现 HSL 可用性检测和自动回退逻辑
- [ ] 6.4 实现求解结果提取：最优傅里叶系数 → 最优轨迹 q/v/a/τ

## 7. BaseOptimalTrajectory 集成

- [ ] 7.1 修改 `BaseOptimalTrajectory.__init__` 根据 `trajectory_type` 选择 `FourierTrajectory` 或 `CubicSplineTrajectory`
- [ ] 7.2 傅里叶路径跳过 `_generate_feasible_initial_guess` 的样条随机搜索，改用 `FourierTrajectory` 初始化策略
- [ ] 7.3 傅里叶路径的 `solve()` 使用 CasADi 符号管线（跳过 `WaypointsGeneration`、跳过 `TrajectoryConstraintManager` 数值约束）
- [ ] 7.4 确保 `save_results()` 和 `plot_results()` 对傅里叶轨迹输出格式一致

## 8. 约束管理适配

- [ ] 8.1 在 `TrajectoryConstraintManager` 中新增 `build_symbolic_constraints()` 方法，返回 CasADi SX 约束表达式和边界向量
- [ ] 8.2 确保符号约束边界与数值约束边界使用相同的关节限位数据源（`CubicSpline` 的 `lower_q`/`upper_q` 等）

## 9. CasADi Backend 清理

- [ ] 9.1 删除 `_make_spline_callback()` — 样条 Callback，不再需要
- [ ] 9.2 删除 `_make_objective_callback()` — 代理目标 Callback，不再需要
- [ ] 9.3 删除 `_make_constraint_callback()` — 约束 Callback，不再需要
- [ ] 9.4 删除 `objective_function()` — numpy `cond(W_b)`，不再需要
- [ ] 9.5 删除 `_compute_jacobian_sparsity()` — 数值雅可比回退，不再需要
- [ ] 9.6 保留 `CasadiBackend._ensure_symbolic_model()` 和 `build_regressor()` 供辨识阶段使用

## 10. 测试与验证

- [ ] 10.1 编写 `FourierTrajectory` 单元测试：傅里叶表达式 v/a 与解析导数一致
- [ ] 10.2 编写 D-最优目标单元测试：已知 W_b 手动计算 log det 对照
- [ ] 10.3 编写符号约束/回归雅可比与有限差分对照测试（容差 1e-5）
- [ ] 10.4 编写配置解析回归测试：现有配置文件不受影响
- [ ] 10.5 在 UR10 示例上端到端验证：`trajectory_type: "fourier"` 成功生成轨迹，条件数优于随机轨迹
- [ ] 10.6 验证 `tanh(α·v)` 在不同 α 值下的 CasADi 可微性

## 11. 文档

- [ ] 11.1 编写环境搭建文档：pixi 环境创建、CasADi+IPOPT+HSL 安装步骤、依赖验证脚本使用方法
- [ ] 11.2 编写配置文档：所有新增字段说明、默认值、取值约束、配置示例
- [ ] 11.3 编写运行文档：从 URDF 加载到激励轨迹生成的完整步骤、命令行参数说明
- [ ] 11.4 编写验证文档：如何验证轨迹有效性、如何验证 AD 正确性、如何对比样条与傅里叶轨迹辨识效果
