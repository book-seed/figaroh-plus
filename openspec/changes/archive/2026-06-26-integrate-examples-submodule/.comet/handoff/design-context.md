# Comet Design Handoff

- Change: integrate-examples-submodule
- Phase: design
- Mode: compact
- Context hash: a177330d90246d52320e66471b7053bfb51ed7c882d5ca15dc91402fe93b5229

Generated-by: comet-handoff.sh

OpenSpec remains the canonical capability spec. This handoff is a deterministic, source-traceable context pack, not an agent-authored summary.

## openspec/changes/integrate-examples-submodule/proposal.md

- Source: openspec/changes/integrate-examples-submodule/proposal.md
- Lines: 1-39
- SHA256: 3a7792f23400aaa0d34c0b3fee68639faf179d88400d7f3c6f79a3c6489d0c9a

```md
# Proposal: Integrate figaroh-examples as Git Submodule

## Why

`figaroh-examples`（含 111MB 模型文件、示例脚本和 web-interface）目前作为独立仓库存在于 `book-seed/figaroh-examples`，与主库 `figaroh-plus` 分离管理。用户需要分别克隆两个仓库，且依赖配置分散在 `environment.yml`、`requirements.txt` 和 README 中，导致依赖漂移和维护负担。将 examples 以 git submodule 形式集成可保持主库轻量（18MB），同时统一依赖管理到 pixi。

## What Changes

- 将 `figaroh-examples` 作为 git submodule 添加到仓库根目录，指向 `https://github.com/book-seed/figaroh-examples.git`（main 分支）
- 删除 figaroh-examples 仓库中的 `environment.yml` 和 `requirements.txt`（pixi 接管）
- 在 pixi examples feature 中补全缺失依赖：`viser`、`hppfcl`
- 从 `pyproject.toml` `[project.dependencies]` 中移除 `build`（属 build-system，非运行时依赖）
- 更新 `.gitignore` 中过时的 examples 注释
- 更新 `pyproject.toml` 和 `README.md` 中 examples URL，统一指向 `book-seed/figaroh-examples`
- 更新 figaroh-examples 内 README（顶层和各 robot），删除 pip install 指令，替换为 pixi 使用说明
- **BREAKING**: `build` 包从运行时依赖中移除，依赖 build 工具的脚本需通过 `pixi run build` 执行

## Capabilities

### New Capabilities

- `examples-submodule`: git submodule 方式的 examples 集成，支持 `git clone --recurse-submodules` 一键获取主库和例程
- `examples-pixi-deps`: pixi examples 环境包含完整运行依赖（viser、hppfcl），替代分散的 conda/pip 配置

### Modified Capabilities

- `pixi-environment`: examples feature 新增 `viser`、`hppfcl` 依赖；移除 `build` 运行时依赖声明；环境定义调整为适配 submodule 路径

## Impact

| 影响范围 | 详情 |
|---------|------|
| `.gitmodules` | 新建，声明 submodule URL/path/branch |
| `.gitignore` | 更新 examples/models 相关注释 |
| `pyproject.toml` | 移除 `build` 依赖，更新 URL，pixi examples feature 补充依赖 |
| `README.md` | 克隆说明改为 `--recurse-submodules`，更新 examples URL |
| `figaroh-examples/` | 目录从 untracked copy 变为 submodule；删除 `environment.yml`、`requirements.txt`；更新 README |
| `src/figaroh/tools/load_robot.py` | 无需修改（已有相对路径回退链，兼容 submodule 位置） |
| pixi.lock | 需重新生成（依赖变更） |
```

## openspec/changes/integrate-examples-submodule/design.md

- Source: openspec/changes/integrate-examples-submodule/design.md
- Lines: 1-103
- SHA256: 87bfa4640be0f8b120b00ef78f037c4bc77efacd35ec172182aeb6d715695e90

[TRUNCATED]

```md
# Design: Integrate figaroh-examples as Git Submodule

## Context

当前 `figaroh-examples/` 目录是以完整 clone 方式手动复制到工作区的 untracked 目录。主库 18MB，examples（含 111MB 模型文件）315MB。`load_robot.py` 已有多级回退查找链（import → sys.path → 相对路径），兼容 submodule 位置。pixi 已在 `pyproject.toml` 中声明 `examples` feature 和 environment，但缺少 `viser` 和 `hppfcl` 依赖。

## Goals / Non-Goals

**Goals:**
- 将 figaroh-examples 以 git submodule 集成到 figaroh-plus，保持主库轻量
- 统一所有依赖管理到 pixi，消除 `environment.yml`、`requirements.txt`、README 中的分散声明
- 补全 examples 运行所需缺失依赖（viser, hppfcl）
- 清理错误/冗余依赖（build 运行时声明、cvxpy 文档引用）
- UR10 calibration 例程可通过 pixi 环境成功运行

**Non-Goals:**
- 在 figaroh-examples 仓库内添加独立 pixi 配置
- 修改示例脚本功能逻辑
- 为核心库添加新功能

## Decisions

### D1: Git Submodule vs 其他方案

| 方案 | 主库体积 | 版本锁定 | 独立更新 | 复杂度 |
|------|:---:|:---:|:---:|:---:|
| **Submodule** | 轻量（指针） | ✓ 锁定 commit | ✓ 独立 PR | 中 |
| Git Subtree | 重（完整副本） | ✗ 混入主库历史 | ✗ 需手动同步 | 高 |
| 直接复制源码 | 重 | ✗ | ✗ | 低 |
| 保持独立仓库 | 轻量 | ✗ 无关联 | ✓ | 低（现状） |

**决定：Submodule。** 核心原因：主库保持 18MB，用户可以按需 `--recurse-submodules` 获取 examples。版本锁定确保主库的每个 commit 指向已知可用的 examples 版本。

### D2: Submodule URL 协议

**决定：HTTPS 为主（`https://github.com/book-seed/figaroh-examples.git`）**。SSH 用户通过 git `insteadOf` 配置自动切换，无需在 `.gitmodules` 中特殊处理。

### D3: 依赖管理架构

```
pixi 环境层次：
┌─────────────────────────────────────────┐
│ examples env = core + examples features  │
│                                         │
│ core feature:                           │
│   conda: [python, cyipopt]              │
│   pypi: [figaroh (editable)]            │
│                                         │
│ examples feature:                        │
│   conda: [pinocchio]  ← 新增             │
│   pypi: [jupyter, notebook, ipywidgets,  │
│          viser, hppfcl]  ← viser/hppfcl  │
└─────────────────────────────────────────┘
```

**决定：examples feature 只声明 examples 专属依赖。** 核心依赖（numpy, scipy, matplotlib, pandas, pin, meshcat, picos 等）通过 `figaroh` 包的可编辑安装传递获得，不在 examples feature 中重复声明。

**注意：`hppfcl` 作为 `pin` (pinocchio) 的传递依赖 `coal` 可能已提供。** 需在实现阶段验证：若 `coal` 已暴露 `hppfcl` 模块，无需额外声明；否则需添加。

### D4: `build` 包处理

`build>=1.5.0,<2` 声明在 `[project.dependencies]` 中，但全代码零 import。`build` 是 PEP 517 构建工具，已在 `[build-system] requires = ["hatchling"]` 中管理。

**决定：从 `[project.dependencies]` 中移除。** `pixi run build` 任务通过 pixi 环境提供 build 包，不影响打包流程。

### D5: figaroh-examples 内配置文件处理

**决定：删除 `environment.yml` 和 `requirements.txt`。** Submodule 用户通过父仓库 pixi 环境运行，独立 conda/pip 路径保留在仓库中会产生混淆。各 robot README 中的 `pip install` 段替换为 `pixi run -e examples python <script>` 指令。

### D6: `load_robot.py` 兼容性

当前回退链：
```
Method 1: import figaroh_examples (pip package)
Method 2: sys.path search (installed egg/wheel)
Method 3: relative path from src/figaroh/tools → ../../figaroh-examples
```

Method 3 的路径 `src/figaroh/tools → ../../ → figaroh-plus 根目录 → figaroh-examples/`，submodule 位于仓库根目录的 `figaroh-examples/`，路径可解析。

```

Full source: openspec/changes/integrate-examples-submodule/design.md

## openspec/changes/integrate-examples-submodule/tasks.md

- Source: openspec/changes/integrate-examples-submodule/tasks.md
- Lines: 1-41
- SHA256: 140c09dc3fa1d515e01eaa6db94be3b72e1326a75e17ae621d7e05bdfa3b090c

```md
# Tasks: Integrate figaroh-examples as Git Submodule

## 1. Submodule Setup

- [ ] 1.1 删除当前 untracked `figaroh-examples/` 目录
- [ ] 1.2 执行 `git submodule add https://github.com/book-seed/figaroh-examples.git figaroh-examples`
- [ ] 1.3 进入 submodule，checkout main 分支，验证 commit `3983de5` 为最新

## 2. figaroh-examples 仓库清理

- [ ] 2.1 删除 `environment.yml`
- [ ] 2.2 删除 `requirements.txt`
- [ ] 2.3 更新顶层 `README.md`：安装说明改为 pixi 使用方式
- [ ] 2.4 更新 `examples/ur10/README.md`：删除 pip install 段（含错误 cvxpy），替换为 pixi 指令
- [ ] 2.5 检查并更新其他 robot README（staubli_TX40, talos, tiago, templates），如有 pip install 段一并替换

## 3. figaroh-plus 配置更新

- [ ] 3.1 从 `pyproject.toml` `[project.dependencies]` 中移除 `build>=1.5.0,<2`
- [ ] 3.2 pixi examples feature 添加 `viser`（pypi-dependencies）
- [ ] 3.3 确认 `hppfcl` 已由 `coal`（pin 传递依赖）提供——design 阶段已验证，无需添加到 examples feature
- [ ] 3.4 更新 `pyproject.toml` 中 Examples URL 为 `https://github.com/book-seed/figaroh-examples`
- [ ] 3.5 更新 `.gitignore` 中 "# Examples and models moved to separate repository" 注释为 "# Examples and models managed as git submodule"
- [ ] 3.6 更新 `README.md`：克隆说明改为 `git clone --recurse-submodules`，更新 examples URL

## 4. 依赖验证与重建

- [ ] 4.1 运行 `pixi update` 重新生成 `pixi.lock`
- [ ] 4.2 验证 `pixi run -e examples python -c "import viser; import hppfcl; import pinocchio"` 成功
- [ ] 4.3 验证 `pixi run -e examples python -c "from figaroh.tools.robot import load_robot; print(load_robot._get_models_directory())"` 正确解析 models 目录

## 5. UR10 例程验收

- [ ] 5.1 运行 `pixi run -e examples python examples/ur10/calibration.py`，验证正常执行并输出标定结果
- [ ] 5.2 运行 `pixi run test`，确认现有 212 个测试不受影响

## 6. 提交

- [ ] 6.1 在 figaroh-examples 仓库提交清理变更并推送到 main
- [ ] 6.2 更新 submodule 指针到最新 main commit
- [ ] 6.3 在 figaroh-plus 提交所有变更（含 `.gitmodules`、submodule 指针、配置更新）
```

