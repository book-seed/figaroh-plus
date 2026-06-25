# Brainstorm Summary

- Change: migrate-to-pixi
- Date: 2026-06-25

## 确认的技术方案

采用 **pixi Feature 分层组合** 模式管理多环境依赖：

| 环境 | 组成 | 用途 |
|------|------|------|
| `default` | core + dev + docs + figaroh(editable) | 日常开发 |
| `docs` | core + docs | CI 文档构建 |
| `test` | core + dev | CI 测试 |
| `examples` | core + examples | Jupyter 演示 |

核心依赖结构：
- `[feature.core.dependencies]`：python==3.12.* + cyipopt（唯一 conda 依赖，走 conda-forge）
- `[feature.core.pypi-dependencies]`：numpy, scipy, pin, matplotlib, pyyaml, meshcat, pandas, rospkg, picos, numdifftools, ndcurves, figaroh(editable)
- `[feature.dev.pypi-dependencies]`：pytest>=6.0, pytest-cov, black, isort, flake8, mypy
- `[feature.docs.pypi-dependencies]`：sphinx, sphinx-rtd-theme, sphinx-autodoc-typehints
- `[feature.examples.pypi-dependencies]`：jupyter, notebook, ipywidgets

7 个 pixi tasks：shell, test, lint, format, docs, build, clean

CI 迁移：`setup-python` → `prefix-dev/setup-pixi@v0`，删除手动 apt/pip 步骤

pyproject.toml 精简为构建元数据骨架，删除 [dependencies] 和 [optional-dependencies]

## 关键取舍与风险

| 决策 | 取舍 |
|------|------|
| pyproject.toml 共存而非删除 | 保持 pip install 兼容性，代价是两文件维护 |
| pin/rospkg 走 PyPI 非 conda | 避免 conda 名称冲突(gin)/平台缺失(rospkg)，代价是混用 conda+PyPI |
| python 精确 pin 3.12 | 与当前环境一致、tegra 已验证，代价是未来需手动升级 |
| examples 独立 feature 非合并 default | 减小 default 体积，代价是多一个环境维护 |

| 风险 | 缓解 |
|------|------|
| 部分包在 aarch64 conda-forge 缺失 | PyPI fallback |
| cyipopt 版本冲突 | pixi SAT solver 自动检测，必要时降级 |
| pixi solve 超时 | 首次 5-10min，lock 永久缓存 |
| 开发者不熟悉 pixi | README quickstart |

## 测试策略

金字塔：静态检查 → 环境验证 → 集成验证
- 静态：pixi validate, pyproject.toml check, YAML lint
- 环境：三环境 pixi install + import figaroh
- 集成：pixi run test/lint/format/docs/build 全部通过
- 跨平台：检查 pixi.lock 包含三平台解析条目

## Spec Patch

无 — 现有 delta spec 验收场景完整，无需回写。
