## 1. 友好错误检测

- [x] 1.1 在 `fourier_strategy.py` 的 casadi 延迟导入处（`solve` 开头）捕获 `ImportError`/`ModuleNotFoundError`，重抛带 `pixi add casadi` 安装提示的友好 `ImportError`。
- [x] 1.2 清理 `strategies/__init__.py` 的 `create_strategy("fourier")` 分支：移除/改写已失效的 `FourierOptimizationStrategy is None → casadi` 判定，使 None 文案与真实成因（模块导入失败）一致，保留顶层 try/except 兜底。

## 2. 验证

- [x] 2.1 运行策略相关导入/单元测试，确认 spline 路径不受影响、fourier 路径在缺 casadi 时抛友好错误（环境已装 casadi 时验证正常路径不回归）。
