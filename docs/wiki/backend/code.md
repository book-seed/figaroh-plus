# backend 代码页

> 模块：[src/figaroh/backend/](../../../src/figaroh/backend/) · 实现见 [algorithm](algorithm.md) · 唯一消费者 [optimal/algorithm](../optimal/algorithm.md)

## 模块定位

`figaroh.backend` 是 CasADi **符号计算后端**。它把 [RNEA 回归子](../algorithms/rnea-regressor.md)的解析雅可比编译成可缓存、可 `.map()` 并行的 CasADi `SX Function`，供 Fourier 轨迹优化策略构造 MX 层 NLP。

该模块是工程内**唯一**的后端——已 sink 进 `FourierOptimizationStrategy`：策略在 `solve()` 内部按需 `CasadiBackend(robot=context.robot)`，不经过 `BaseOptimalTrajectory._backend`（spec 曾要求经 `context._backend`，实际偏离）。无 `Backend` ABC、无 `create_backend` 工厂、无单例。符号模型在首次访问时直接构建（无磁盘缓存），`cpin.computeJointTorqueRegressor` 和 `cpin.rnea` 为 C++ 原生实现，构造开销在毫秒级。

## 文件清单

| 文件 | 职责 |
|------|------|
| [`__init__.py`](../../../src/figaroh/backend/__init__.py) | 包出口；`try` 导入 `CasadiBackend`，缺依赖时置 `None`，`__all__ = ["CasadiBackend"]` |
| [`casadi.py`](../../../src/figaroh/backend/casadi.py) | 全部实现：惰性导入、缓存键、符号模型构建、属性出口 |

## 公共 API

| 成员 | 签名 / 值 | 说明 |
|------|-----------|------|
| `CasadiBackend.__init__` | `(self, robot)` | 仅存 `self._robot`，`_cmodel/_W_fun/_rnea_fun = None`；不构造符号模型 |
| `_lazy_import()` | 模块函数 | 分 `cs`/`cpin` 两条独立 `try/except`，首次方法调用时触发 |
| `_ensure_symbolic_model()` | `(self)` | 从零构建符号模型（幂等）；`cpin` C++ 原生实现，构造开销毫秒级 |
| `regressor_function` | `@property` | 触发 `_ensure_symbolic_model`，返回 `W_fun: (q,v,a)->W` |
| `rnea_function` | `@property` | 触发 `_ensure_symbolic_model`，返回 `rnea_fun: (q,v,a)->τ` |

## 调用链

```
FourierOptimizationStrategy.solve(context)
        │
        │  cas_be = CasadiBackend(robot=context.robot)
        │  cas_be._ensure_symbolic_model()
        ▼
CasadiBackend.__init__(robot)          # 仅存引用，不构建
        │
        │  首次访问 .regressor_function / .rnea_function
        ▼
_ensure_symbolic_model()
        │
        └─ cpin.Model + computeJointTorqueRegressor + rnea
           cs.Function("W") + cs.Function("rnea")
        │
        ▼
regressor_function / rnea_function     # SX Function
        │
        │  外层 MX: W_fun.map(N,"openmp") / rnea_fun.map(N,"openmp")
        ▼
Fourier NLP (cs.nlpsol ipopt)          # 见 ../optimal/algorithm.md
```

## 关键 Gotcha

- **PyPI `pin` 无 CasADi 绑定**：`_lazy_import` 探测 `pinocchio.__version__` 存在但 `pinocchio.casadi` 导入失败时，给出卸载 `pin`、装 conda-forge `pinocchio` 的提示。
- **无 ABC / 工厂 / 单例**：`CasadiBackend` 不继承任何基类；每次 `solve()` 都 `new` 一个实例，符号模型在首次访问时直接构建，`cpin` C++ 原生实现开销在毫秒级。

> 测试见 [tests/unit/test_backend.py](../../../tests/unit/test_backend.py)

→ 算法设计见 [algorithm](algorithm.md)
