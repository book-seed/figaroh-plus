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

**决定：无需修改 `load_robot.py`。**

## Risks / Trade-offs

| 风险 | 缓解措施 |
|------|---------|
| 用户 clone 时忘记 `--recurse-submodules`，examples 目录为空 | README 显著位置说明；`load_robot.py` 错误信息已包含获取指引 |
| figaroh-examples 仓库 main 分支更新后，submodule 指针过期 | 定期 `git submodule update --remote` 或在 figaroh-examples 发版时手动更新指针 |
| `hppfcl` 模块可能由 `coal`（pin 传递依赖）提供，重复声明导致冲突 | 实现阶段验证：先检查 `coal` 是否已暴露 `hppfcl`，是则不添加 |
| pixi.lock 重新生成可能引入不兼容版本 | 在 `pixi update` 后运行全量测试验证 |
| submodule 内的 web-interface 依赖 `viser`，viser 版本可能与其他依赖冲突 | 实现阶段验证 viser 与 pin/pinocchio 版本兼容性 |

## Migration Plan

1. 删除当前 untracked `figaroh-examples/` 目录
2. 执行 `git submodule add https://github.com/book-seed/figaroh-examples.git figaroh-examples`
3. 进入 submodule，checkout main 分支，拉取最新 commit
4. 在 submodule 内删除 `environment.yml`、`requirements.txt`，更新 README
5. 在 figaroh-plus 中更新 `pyproject.toml`、`.gitignore`、`README.md`
6. 运行 `pixi update` 重新生成 lock 文件
7. 验证 UR10 calibration 例程可运行
8. 提交 submodule 变更到 figaroh-examples 仓库
9. 提交 figaroh-plus 变更（含 `.gitmodules` 和 submodule 指针）
