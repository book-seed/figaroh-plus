## Context

figaroh-plus 是一个 Python 机器人动力学/标定工具箱（v0.4.3），当前依赖管理方式：

```
当前状态：
  pyproject.toml  → 声明 PyPI 依赖（numpy、scipy、pin 等）+ hatchling 构建
  environment.yml → 声明 conda 环境（python 3.12 + cyipopt）
  .github/workflows/docs.yml → 手动 apt/pip 安装依赖构建文档
  无 lock 文件 → 环境不可复现
```

pixi (v0.71.0) 已安装在开发机上。项目运行在 NVIDIA Jetson (linux-aarch64)，同时需要支持 x86 CI 和 macOS 开发。

## Goals / Non-Goals

**Goals:**
- 统一依赖声明到 `pixi.toml`（conda + PyPI）
- 生成三平台（linux-aarch64、linux-64、osx-arm64）`pixi.lock`
- 定义 7 个 pixi tasks：test、lint、format、docs、build、shell、clean
- 多环境支持：default（核心开发）、docs（文档构建）、docs 和 test
- 精简 pyproject.toml 为构建元数据骨架
- 更新 CI docs.yml 使用 pixi

**Non-Goals:**
- 不修改源码和 API
- 不改变 Python 版本要求（保持 >=3.8）
- 不引入新运行时依赖
- 不迁移到其他构建系统（保持 hatchling）

## Decisions

### 1. 共存模式：pixi.toml + pyproject.toml（精简）

**选择**：pixi.toml 管理所有依赖和 tasks，pyproject.toml 精简为构建元数据骨架。

**Alternatives considered:**
- 纯 pixi.toml（删除 pyproject.toml）→ 失去 Python 生态兼容（pip install 失效、PyPI 发布受影响）
- pip-tools / Poetry → 不原生支持 conda 依赖（cyipopt），需要额外脚本桥接

**Rationale**：pixi 原生支持 conda-forge 和 PyPI，同时保持 pyproject.toml 兼容标准 Python 工具链。

### 2. 纯 pixi 模式

**选择**：`pixi.toml` 作为依赖唯一来源，pyproject.toml 不保留 `[dependencies]`。

**Rationale**：避免两处声明依赖导致的不一致风险。pixi 的 `[pypi-dependencies]` 和 `[dependencies]`（conda）覆盖所有用例。

### 3. 多环境设计

**选择**：三个 pixi feature/environment：

| 环境 | 内容 | 用途 |
|------|------|------|
| `default` | 完整 conda + PyPI 依赖 | 日常开发 |
| `docs` | sphinx + 文档依赖 | CI 文档构建 |
| `test` | pytest + 核心依赖 | CI 测试 |

**Rationale**：CI 环境不需要完整 GUI 依赖（meshcat、matplotlib），减少解析时间和体积。

### 4. tasks 设计

| task | 环境 | 命令 |
|------|------|------|
| `shell` | default | 启动带激活环境的 shell |
| `test` | default | `pytest tests/` |
| `lint` | default | `flake8 src/ && mypy src/` |
| `format` | default | `black src/ tests/ && isort src/ tests/` |
| `docs` | docs | `cd docs && make html` |
| `build` | default | `python -m build` |
| `clean` | default | `rm -rf dist/ build/ *.egg-info` |

### 5. 平台策略

三个目标平台按 tier 分级：
- **Tier 1 (linux-aarch64)**：主开发平台，lock 必须完整
- **Tier 1 (linux-64)**：CI 平台，lock 必须完整
- **Tier 2 (osx-arm64)**：开发者平台，尽力完整；已知 `rospkg` 可能不可用，必要时降级为可选项

### 6. CI 迁移

docs.yml 从 `apt install + pip install` 迁移为 `pixi run docs`：
- 使用 `ghcr.io/prefix-dev/pixi` Docker 镜像或 `setup-pixi` action
- 依赖安装简化为 `pixi install -e docs`

## Risks / Trade-offs

| Risk | Mitigation |
|------|-----------|
| `pin` 在 conda-forge 中名称冲突（pin 是 Python 包，也是系统工具） | pixi 的 `[pypi-dependencies]` 直接从 PyPI 安装，避免冲突 |
| `rospkg` 在 osx-arm64 conda-forge 不可用 | 尝试 PyPI fallback；若彻底不可用则从 osx lock 中排除，文档记录 |
| `cyipopt` 版本与 numpy/scipy 有严格兼容矩阵 | 锁定 cyipopt 具体版本，`pixi.lock` 记录精确 pin |
| Tegra aarch64 包解析慢/失败 | 预测试 linux-aarch64 解析；必要时接受较长解析时间 |
| pixi 团队经验少，学习曲线 | 提供 README 中的 quickstart 说明 |

## Open Questions

- `pin`、`rospkg`、`ndcurves`、`picos`、`numdifftools` 在 conda-forge 三平台的可用性需在 `pixi add` 时逐一验证
- 当前 CI 仅有 docs workflow，是否需要新增 test workflow？（本次不新增，但 task 设计预留扩展空间）
