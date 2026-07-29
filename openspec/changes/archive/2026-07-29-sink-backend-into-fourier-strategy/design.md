# Design: 将 _backend 下沉到 fourier 策略

## 背景

`BaseOptimalTrajectory.__init__` 当前按 `trajectory_type` 创建 `self._backend`：

```python
traj_type = self.trajectory_config.get("trajectory_type", "spline")
if traj_type == "fourier":
    self._backend = CasadiBackend(robot=robot)
else:
    self._backend = None
```

唯一消费方是 `FourierOptimizationStrategy.solve` 中的 `cas_be = context._backend`。spline 策略与 identification 阶段均不使用符号模型。

## 方案

### 1. `base_optimal_trajectory.py`

- 删除顶部 `from figaroh.backend.casadi import CasadiBackend`。
- 删除 `__init__` 中 `_backend` 创建块（`if traj_type == "fourier"` 的 backend 分支与 `else: self._backend = None`）。
- strategy 创建逻辑保持不变（fourier 仍走 `create_strategy("fourier", fourier_config=...)`）。

### 2. `fourier_strategy.py`

`solve` 开头改为局部构造：

```python
from figaroh.backend.casadi import CasadiBackend
cas_be = CasadiBackend(robot=context.robot)
cas_be._ensure_symbolic_model()
cmodel = cas_be._cmodel
```

其余对 `cas_be` 的引用（`regressor_function`、`rnea_function`）保持不变。

### 3. `casadi.py`

在 `CasadiBackend` 类 docstring/注释处补 TODO：

> 未来 identification 阶段接入符号模型时，应改为模块级按 robot 指纹索引的共享单例（如 `get_symbolic_model(robot)`），由 fourier 与 identification 共同复用，而非挂在轨迹生成实例上。当前磁盘缓存（`_cache_key` + `~/.figaroh/casadi_cache`）已使独立构造的开销降低到一次 `cs.Function.load`，故暂不需要上提共享。

## 行为等价性

- fourier 路径：仍创建 `CasadiBackend`，仍走懒加载 + 磁盘缓存，符号模型构建时机与内容不变。
- spline 路径：原本 `self._backend = None` 即不创建符号模型；删除后基类不再 import casadi 模块，spline 环境更干净。
- `_backend` 为下划线内部属性，无外部消费者，删除不影响任何公开接口。

## 不在本次范围

- 不引入 identification 的符号模型路径（未来单独变更）。
- 不新增模块级共享单例（YAGNI，等辨识真接入时再做）。
