## Why

`BaseOptimalTrajectory.__init__` 在公共层创建 `self._backend = CasadiBackend(robot=...)` 并 import `figaroh.backend.casadi`，但全工程仅 `FourierOptimizationStrategy.solve` 一处消费 `context._backend`。spline 路径从不使用它，公共基类不应感知 CasADi 的存在。这是上次 `remove-trajectory-backend-optionality` 重构删除 backend 可选性后遗留的公共层赋值——既然 backend 已由 `trajectory_type` 自动决定且仅 fourier 需要，它就应归 fourier 策略所有，而非挂在轨迹生成基类上。

## What Changes

- 移除 `BaseOptimalTrajectory.__init__` 中 `_backend` 的创建逻辑（包括 `CasadiBackend` 实例化与 `None` 分支）以及顶部的 `from figaroh.backend.casadi import CasadiBackend`。
- `FourierOptimizationStrategy.solve` 内部按需 `CasadiBackend(robot=context.robot)`，用局部变量 `cas_be` 替代 `context._backend` 读取。
- 在 `CasadiBackend` 类上留 TODO 注释，标注未来 identification 阶段接入符号模型时，应改为模块级按 robot 指纹索引的共享单例（`get_symbolic_model(robot)`），而非挂在某个轨迹生成实例上。

## Capabilities

### New Capabilities
<!-- 无新增 capability -->

### Modified Capabilities
<!-- 无 spec 级别需求变更；本次为纯实现下沉，行为不变，不改任何已有 spec 的验收场景。 -->

## Impact

- 受影响文件：`src/figaroh/optimal/base_optimal_trajectory.py`、`src/figaroh/optimal/strategies/fourier_strategy.py`、`src/figaroh/backend/casadi.py`（仅注释）。
- 行为不变：fourier 路径仍创建并使用 `CasadiBackend`；spline 路径不再触发 casadi 模块 import（懒加载本就推迟到首次调用，现在连基类引用也移除）。
- 无接口变化：`_backend` 本是内部属性（下划线前缀），无外部消费者。
- 依赖不变：CasADi 仍是 fourier 路径的强依赖（懒加载 + 磁盘缓存机制不受影响）。
