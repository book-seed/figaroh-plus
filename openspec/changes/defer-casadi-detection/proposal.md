## Why

`strategies/__init__.py` 用顶层 `try/except ImportError` 捕获 `FourierOptimizationStrategy` 的导入失败，并在 `create_strategy()` 中以 `if FourierOptimizationStrategy is None` 抛带安装提示的友好错误。但 `fourier_strategy.py` 顶层只 import `logging`/`typing`/`numpy`/`.base_strategy`，casadi 是刻意延迟导入（写在 `solve()` 内），因此缺 casadi 时顶层导入不会失败，`FourierOptimizationStrategy` 永不为 `None`，`create_strategy` 的友好报错拿不到机会执行；真正缺 casadi 时要等到 `BaseOptimalTrajectory.solve()` → `strategy.solve()` 才抛原生 `ModuleNotFoundError: No module named 'casadi'`，用户拿不到安装指引。

## What Changes

- 将 casadi 可用性检测推迟到真正首次使用它的点（`FourierOptimizationStrategy.solve` 的延迟导入处），在该处抛带安装提示的友好 `ImportError`。
- 清理 `strategies/__init__.py` 中已失效的 `FourierOptimizationStrategy is None` 判定语义，使其与"casadi 延迟导入"的事实一致。
- 不改变 `create_strategy` 的公开签名与返回类型，不影响 spline 路径。

## Capabilities

### New Capabilities

（无）

### Modified Capabilities

（无）——casadi 缺失的报错处理属实现细节。现有 `symbolic-casadi-pipeline` spec 只覆盖 OpenMP/HSL 不可用场景，`fourier-trajectory-parameterization` 只覆盖 Fourier 参数化数学，均无"casadi 模块未安装时如何报错"的验收场景，故不产生 delta spec。

## Impact

- `src/figaroh/optimal/strategies/__init__.py`：调整 `create_strategy("fourier")` 分支的 casadi 可用性判定逻辑与文案。
- `src/figaroh/optimal/strategies/fourier_strategy.py`：在延迟导入 casadi 处补充友好错误抛出。
- 公开 API 不变；spline 用户行为不变；仅 fourier 路径缺 casadi 时的报错时机与文案改善。
