## Context

`fourier_strategy.py` 的 casadi 是延迟导入：`solve()` 内 `import casadi as cs` 与 `from figaroh.backend.casadi import CasadiBackend`（约 line 67-68）。这是为了让只走 spline 路径的用户不必安装 casadi 而刻意设计的（`backend/__init__.py` 注释："Imported lazily so spline users need not install them"）。

副作用：`strategies/__init__.py` 顶层 `try: from .fourier_strategy import FourierOptimizationStrategy except ImportError: FourierOptimizationStrategy = None` 永不因 casadi 触发，于是 `create_strategy` 里 `if FourierOptimizationStrategy is None: raise ImportError("requires CasADi...")` 成为死代码；缺 casadi 的真实错误延后到 `solve()` 才以原生 `ModuleNotFoundError` 出现，丢失安装提示。

## Goals / Non-Goals

**Goals:**

- 缺 casadi 时在 fourier 路径上抛出带 `pixi add casadi` 安装提示的友好错误。
- 让检测点与延迟导入点一致，消除死代码与误导文案。

**Non-Goals:**

- 不把 casadi 改为顶层强导入（会牺牲 spline 用户免装 casadi 的初衷）。
- 不改变 `create_strategy` 签名、返回类型或 spline 路径行为。
- 不改动 `CasadiBackend` 构造位置或 import 路径。
- 不引入 delta spec（无 spec 级验收场景变化）。

## Decisions

- **检测点选在 `FourierOptimizationStrategy.solve` 的延迟导入处而非 `create_strategy`**：casadi 实际首次使用就在 `solve` 的 `import casadi as cs` / `from figaroh.backend.casadi import CasadiBackend`。把友好检测放在此处，与延迟导入同处，最贴近真实失败点，且不破坏 `__init__` 的零成本构造。
  - 备选 A：在 `__init__` 里就 `import casadi` 做检测——会让构造失败提前（即使不立即 solve 也要装 casadi），与"按需"语义略有出入；tweak 范围内选最小侵入，弃。
  - 备选 B：保留 `__init__.py` 顶层 try/except 来检测 casadi——已证无效，弃。
- **友好错误形式**：捕获原生 `ImportError`/`ModuleNotFoundError`，重抛为带安装提示的 `ImportError`，文案沿用现有措辞（`"FourierOptimizationStrategy requires CasADi. Install with: pixi add casadi"`）以保持一致。
- **`__init__.py` 清理**：`create_strategy("fourier")` 不再以 `FourierOptimizationStrategy is None` 判定 casadi；顶层 `try/except ImportError → None` 仍保留作为"模块文件本身无法导入"（如 `numpy`/`.base_strategy` 损坏）的兜底，但 None 分支文案改为描述真实成因而非 casadi。

## Risks / Trade-offs

- [友好错误仍延后到 `solve` 才抛] → 与原 `create_strategy` 立即抛相比时机更晚；但这是延迟导入的固有代价，且 `solve` 正是真实使用点，可接受。
- [改动 `__init__.py` None 文案可能影响依赖旧文案的下游] → 无外部消费者依赖此内部错误文案；风险低。
