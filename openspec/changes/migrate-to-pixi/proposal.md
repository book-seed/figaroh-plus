## Why

figaroh-plus 当前依赖管理分散在两处（pyproject.toml + environment.yml），没有 lock 文件导致环境不可复现，且 conda/pip 手动操作繁琐。迁移到 pixi 包管理器可以统一 conda 和 PyPI 依赖、生成跨平台可复现 lock 文件、并通过 task 系统自动化日常工作流。

## What Changes

- **新增** `pixi.toml`：主配置文件，统一声明所有依赖（conda + PyPI）、多环境定义、任务定义
- **新增** `pixi.lock`：跨平台可复现锁文件（linux-aarch64、linux-64、osx-arm64）
- **简化** `pyproject.toml`：删除 `[project.dependencies]` 和 `[project.optional-dependencies]`，仅保留构建元数据
- **删除** `environment.yml`：由 pixi.toml 替代
- **更新** `.github/workflows/docs.yml`：使用 pixi 替代原始 pip/apt 安装
- **更新** `.gitignore`：添加 pixi 相关条目（.pixi/ 目录等）

## Capabilities

### New Capabilities

- `pixi-environment`: 通过 pixi 统一管理 conda 和 PyPI 依赖，支持多环境（default、docs、test）和多平台（linux-aarch64、linux-64、osx-arm64）
- `pixi-tasks`: 自动化的 pixi task 定义，覆盖 test、lint、format、docs、build、shell、clean

### Modified Capabilities

_无_ — 本次变更为工具链改造，不影响现有功能 API。

## Impact

- **配置文件**：pixi.toml（新增）、pyproject.toml（精简）、environment.yml（删除）
- **CI/CD**：docs.yml 的依赖安装步骤改为 pixi 命令
- **开发流程**：开发者从 `conda env create` + `pip install -e .` 切换为 `pixi install` + `pixi run shell`
- **依赖**：所有现有运行时依赖保持不变，仅管理方式变更
