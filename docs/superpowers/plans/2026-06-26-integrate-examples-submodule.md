---
change: integrate-examples-submodule
design-doc: docs/superpowers/specs/2026-06-26-integrate-examples-submodule-design.md
base-ref: 34e2a56e0abd44a777287803d5ec40f6f939a27b
archived-with: 2026-06-26-integrate-examples-submodule
---

# Integrate figaroh-examples as Git Submodule — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 figaroh-examples 作为 git submodule 集成到 figaroh-plus 中，统一依赖管理到 pixi，并清理冗余/错误的配置。

**Architecture:** figaroh-examples (315MB, 包含 111MB URDF 模型) 以 git submodule 形式挂载到项目根目录。依赖管理统一由 pixi 处理，examples feature 仅声明 examples 专属依赖 (viser)，核心依赖通过 figaroh 可编辑安装传递。load_robot.py 的 Method 3 回退链（项目根目录 → figaroh-examples/ → models/）天然兼容 submodule 布局。

**Tech Stack:** git submodule, pixi (prefix.dev), pyproject.toml (hatchling), viser (3D 可视化)

## Global Constraints

- figaroh-examples submodule URL: `https://github.com/book-seed/figaroh-examples.git`, branch: `main`
- Python ==3.12.*，与当前 Tegra 环境精确匹配
- pixi.lock 必须包含 linux-aarch64、linux-64、osx-arm64 三平台解析条目
- 所有变更基于 base-ref `34e2a56e0abd44a777287803d5ec40f6f939a27b`
- 现有 212 个测试不得因变更而失败
- `build>=1.5.0,<2` 从 `[project.dependencies]` 中移除（零源码导入，属于构建工具而非运行时依赖）
- `cvxpy` 从 UR10 README 中移除（零代码库导入，属于文档漂移）
- `viser` 添加到 examples feature 的 pypi-dependencies 中

archived-with: 2026-06-26-integrate-examples-submodule
---

## File Structure

| 文件 | 操作 | 职责 |
|------|------|------|
| `.gitmodules` | **创建** | submodule 声明 |
| `figaroh-examples/` | **git submodule add** | submodule 指针（不追踪内容） |
| `pyproject.toml` | **修改** | 删除 `build>=1.5.0,<2`；examples feature 添加 `viser`；更新 Examples URL |
| `pixi.lock` | **重新生成** | 依赖锁文件刷新 |
| `.gitignore` | **修改** | 更新注释：submodule 提及 |
| `README.md` (figaroh-plus 根目录) | **修改** | 克隆说明改为 `--recurse-submodules`；更新 examples 安装为 pixi 方式 |
| `figaroh-examples/environment.yml` | **删除** | pixi 已取代 conda env |
| `figaroh-examples/requirements.txt` | **删除** | pixi 已取代 pip requirements |
| `figaroh-examples/README.md` | **修改** | 安装说明改为 pixi 方式 |
| `figaroh-examples/examples/ur10/README.md` | **修改** | 删除 pip install + cvxpy 段，替换为 pixi 指令 |
| `figaroh-examples/examples/tiago/README.md` | **检查/修改** | 如有 pip install 段则替换 |
| `figaroh-examples/examples/talos/README.md` | **检查/修改** | 如有 pip install 段则替换 |
| `figaroh-examples/examples/templates/README.md` | **检查** | 模板文档，检查是否需要更新 |

archived-with: 2026-06-26-integrate-examples-submodule
---

### Task 1: Submodule 设置（本地替换 + git submodule add）

**Files:**
- Create: `.gitmodules`（自动生成）
- Create: `figaroh-examples/` 指针条目（自动生成）
- Delete: 当前 untracked `figaroh-examples/` 目录（本地副本，非 git 追踪）

**Interfaces:**
- Consumes: figaroh-examples 远程仓库 `https://github.com/book-seed/figaroh-examples.git`，commit `3983de5`
- Produces: 已注册的 git submodule，供 Task 2-3 在其内进行操作

- [x] **Step 1: 确认当前 figaroh-examples 无未提交变更**

Run:
```bash
cd /home/tyche/Documents/figaroh-plus/figaroh-examples && git status
```
Expected: `nothing to commit, working tree clean`

- [x] **Step 2: 记录当前 commit hash 供后续验证**

Run: `cd /home/tyche/Documents/figaroh-plus/figaroh-examples && git rev-parse HEAD`
Expected: `3983de5`（记录此值，后续 submodule 预期就是此 commit）

- [x] **Step 3: 删除当前 untracked figaroh-examples 目录**

Run:
```bash
cd /home/tyche/Documents/figaroh-plus
rm -rf figaroh-examples
```
Expected: `figaroh-examples/` 目录消失

验证：`ls figaroh-examples` → `ls: cannot access 'figaroh-examples': No such file or directory`

- [x] **Step 4: 执行 git submodule add**

Run:
```bash
cd /home/tyche/Documents/figaroh-plus
git submodule add https://github.com/book-seed/figaroh-examples.git figaroh-examples
```
Expected: `Cloning into '/home/tyche/Documents/figaroh-plus/figaroh-examples'...` 然后完成克隆。

验证文件：
- `ls .gitmodules` → 文件存在
- `cat .gitmodules` → 应包含：
  ```
  [submodule "figaroh-examples"]
  	path = figaroh-examples
  	url = https://github.com/book-seed/figaroh-examples.git
  ```
- `git submodule status` → 以 ` 3983de5...` 开头（空格前缀表示未暂存）

- [x] **Step 5: Checkout main 分支并验证 commit**

Run:
```bash
cd /home/tyche/Documents/figaroh-plus/figaroh-examples
git checkout main
git log --oneline -1
```
Expected:
```
Switched to branch 'main'
3983de5 style: add black formatter pre-commit hooks and reformat codebase
```

- [x] **Step 6: 提交 figaroh-plus 层 submodule 注册**

```bash
cd /home/tyche/Documents/figaroh-plus
git add .gitmodules figaroh-examples
git commit -m "feat: add figaroh-examples as git submodule

- Add submodule at figaroh-examples/ (https://github.com/book-seed/figaroh-examples.git)
- Track commit 3983de5 on main branch
- Replace untracked local copy with proper submodule pointer"

git push origin main
```

Wait - we cannot `git push` at this point because the examples repo hasn't been cleaned up yet. The push happens at the end. Let me adjust this step - we commit locally only, no push.

Fix Step 6:

```bash
cd /home/tyche/Documents/figaroh-plus
git add .gitmodules figaroh-examples
git commit -m "feat: add figaroh-examples as git submodule

- Add submodule at figaroh-examples/ (https://github.com/book-seed/figaroh-examples.git)
- Track commit 3983de5 on main branch
- Replace untracked local copy with proper submodule pointer"
```
(注意：git push 在 Task 6 统一执行)

archived-with: 2026-06-26-integrate-examples-submodule
---

### Task 2: figaroh-examples 仓库清理——删除冗余配置文件 + 更新 README

**Files:**
- Modify: `figaroh-examples/README.md`
- Modify: `figaroh-examples/examples/ur10/README.md`
- Modify: `figaroh-examples/examples/tiago/README.md`
- Modify: `figaroh-examples/examples/talos/README.md`
- Delete: `figaroh-examples/environment.yml`
- Delete: `figaroh-examples/requirements.txt`

**Interfaces:**
- Consumes: Task 1 的 figaroh-examples submodule
- Produces: 清理后的 figaroh-examples 工作树（独立的 git 仓库变更），供 Task 6 推送到远程

- [x] **Step 1: 删除 environment.yml**

Run:
```bash
cd /home/tyche/Documents/figaroh-plus/figaroh-examples
git rm environment.yml
```
Expected: `rm 'environment.yml'`

- [x] **Step 2: 删除 requirements.txt**

Run:
```bash
cd /home/tyche/Documents/figaroh-plus/figaroh-examples
git rm requirements.txt
```
Expected: `rm 'requirements.txt'`

- [x] **Step 3: 更新 figaroh-examples 顶层 README.md**

将安装说明从 pip 方式替换为 pixi 方式。编辑 `figaroh-examples/README.md`，将 "## Install" 节（第5-16行）替换为：

```markdown
## Install

These examples are bundled as a git submodule of the main figaroh-plus repository.
To use them with pixi:

```bash
git clone --recurse-submodules https://github.com/book-seed/figaroh-plus.git
cd figaroh-plus
pixi install
pixi run -e examples python examples/ur10/calibration.py
```

If you cloned without `--recurse-submodules`, run:

```bash
git submodule update --init --recursive
```

### Standalone Usage

For standalone use (without the figaroh-plus repo):

```bash
git clone https://github.com/book-seed/figaroh-examples.git
cd figaroh-examples
pip install figaroh
```

Note: This repo no longer ships environment.yml or requirements.txt — dependency management is handled by pixi in the parent figaroh-plus project.
```

- [x] **Step 4: 更新 examples/ur10/README.md——删除 pip install + cvxpy 段**

编辑 `figaroh-examples/examples/ur10/README.md`，删除 "Installation and Dependencies" 节（第230-250行）中：

1. 包含 `cvxpy` 的行（第237行 `pip install picos cvxpy cyipopt`）
2. 整个 "Installation and Dependencies" 节，替换为：

```markdown
## Prerequisites

These examples are designed to run in the figaroh-plus pixi `examples` environment:

```bash
cd /path/to/figaroh-plus
pixi run -e examples python examples/ur10/calibration.py
```

All dependencies (figaroh, numpy, scipy, matplotlib, pandas, pinocchio, cyipopt, etc.) are managed by pixi.
```

注意：只替换安装依赖相关内容，保留其他所有文档内容。

- [x] **Step 5: 更新 examples/tiago/README.md**

读取 `figaroh-examples/examples/tiago/README.md` 前30行，搜索是否有 `pip install` 相关语句。如果有则替换为：

```markdown
All dependencies are managed by pixi in the parent figaroh-plus project.
Run: `pixi run -e examples python examples/tiago/calibration.py` from the figaroh-plus root.
```

如果没有 pip install 段，则跳过此步，仅确认。

- [x] **Step 6: 更新 examples/talos/README.md**

读取 `figaroh-examples/examples/talos/README.md` 前30行，搜索是否有 `pip install` 相关语句。如果有则替换（参考 Step 5 模板）。如果没有则跳过。

- [x] **Step 7: 提交 figaroh-examples 仓库的变更**

```bash
cd /home/tyche/Documents/figaroh-plus/figaroh-examples
git add -A
git status
```
Expected: 包含 environment.yml 删除、requirements.txt 删除、README.md 改动。

```bash
git commit -m "chore: clean up obsolete config files and update READMEs for pixi

- Remove environment.yml (superseded by pixi)
- Remove requirements.txt (superseded by pixi)
- Update READMEs: replace pip install instructions with pixi usage
- Remove cvxpy from UR10 README (zero imports in codebase)"
```

注意：此时不要 push，等 Task 6 统一推送。

archived-with: 2026-06-26-integrate-examples-submodule
---

### Task 3: figaroh-plus 配置更新——pyproject.toml + .gitignore + README

**Files:**
- Modify: `pyproject.toml`（删除 `build>=1.5.0,<2`；添加 `viser`；更新 Examples URL）
- Modify: `.gitignore`（更新注释）
- Modify: `README.md`（更新克隆说明和 examples 使用方式）

**Interfaces:**
- Consumes: 当前 pyproject.toml（含 `build` 依赖）、.gitignore、README.md
- Produces: 更新后的配置，供 Task 4 生成 pixi.lock

- [x] **Step 1: 从 pyproject.toml 删除 `build>=1.5.0,<2`**

编辑 `pyproject.toml` 第52行，从 `[project.dependencies]` 列表中删除：
```toml
"build>=1.5.0,<2",
```
删除操作后，`dependencies` 列表变为：
```toml
dependencies = [
    "numpy",
    "scipy",
    "pin",
    "matplotlib",
    "pyyaml",
    "meshcat",
    "pandas",
    "rospkg",
    "picos",
    "numdifftools",
    "ndcurves",
]
```

- [x] **Step 2: 在 pixi examples feature 的 pypi-dependencies 中添加 viser**

编辑 `pyproject.toml` 第121行区域（`[tool.pixi.feature.examples.pypi-dependencies]`），改为：

```toml
[tool.pixi.feature.examples.pypi-dependencies]
jupyter = "*"
notebook = "*"
ipywidgets = "*"
viser = "*"
```

- [x] **Step 3: 更新 pyproject.toml 中的 Examples URL**

编辑 `pyproject.toml` 第58行，确认 `[project.urls]` 中的 Examples URL 为：
```toml
Examples = "https://github.com/book-seed/figaroh-examples"
```
（当前值已经是 `https://github.com/thanhndv212/figaroh-examples`，根据设计文档 D2，URL 应为 `https://github.com/book-seed/figaroh-examples.git`。与设计文档对比后确认正确的 URL）

验证：当前 pyproject.toml 中 `[project.urls]` 的 Examples 值为 `https://github.com/thanhndv212/figaroh-examples`。设计文档 D2 指定 URL 为 `https://github.com/book-seed/figaroh-examples.git`。如果两个 URL 指向同一个仓库的不同别名，则无需修改；如果不同，则按要求更新。

- [x] **Step 4: 更新 .gitignore 中关于 examples 的注释**

将第17-18行：
```
# Examples and models moved to separate repository
# See: https://github.com/thanhndv212/figaroh-examples
```
替换为：
```
# Examples and models managed as git submodule
# See: https://github.com/book-seed/figaroh-examples
```

- [x] **Step 5: 更新 README.md——克隆说明改为 --recurse-submodules + 更新 examples 使用说明**

在 README.md 中找到 "### Development Installation" 节下的 pixi 克隆示例（第21-27行）：

```markdown
git clone https://github.com/thanhndv212/figaroh-plus.git
cd figaroh-plus
```
改为：

```markdown
git clone --recurse-submodules https://github.com/book-seed/figaroh-plus.git
cd figaroh-plus
```

将 "### Examples Repository" 节（第51-55行）：
```markdown
### Examples Repository
```bash
git clone https://github.com/thanhndv212/figaroh-examples.git
cd figaroh-examples && pip install -r requirements.txt
```
```
替换为：
```markdown
### Examples Repository

The figaroh-examples repository is included as a git submodule. It is automatically cloned
when you use `git clone --recurse-submodules`. If you already cloned without the flag:

```bash
git submodule update --init --recursive
```

Run examples with pixi:

```bash
pixi run -e examples python examples/ur10/calibration.py
```

- [x] **Step 6: 提交 figaroh-plus 配置变更**

```bash
cd /home/tyche/Documents/figaroh-plus
git add pyproject.toml .gitignore README.md
git commit -m "feat: update project config for examples submodule integration

- Remove build>=1.5.0,<2 from project.dependencies (zero imports, build tool)
- Add viser to pixi examples feature (required by web-interface)
- Update .gitignore examples comment to reflect submodule management
- Update README with --recurse-submodules clone instructions
- Replace pip-based examples install with pixi usage"
```

archived-with: 2026-06-26-integrate-examples-submodule
---

### Task 4: 依赖验证与 pixi.lock 重建

**Files:**
- Regenerate: `pixi.lock`
- No manual file edits

**Interfaces:**
- Consumes: Task 3 更新的 pyproject.toml（含 viser 依赖）
- Produces: 更新后的 pixi.lock，包含 viser 解析

- [x] **Step 1: 运行 pixi update 重新生成 pixi.lock**

Run:
```bash
cd /home/tyche/Documents/figaroh-plus
pixi update
```
Expected: pixi 解析所有依赖，输出平台解析信息，重新生成 `pixi.lock`。由于添加了 `viser`，lock 文件会有变更。

如果出现版本冲突，检查 viser 是否与现有依赖有 pin 版本冲突。viser 是独立包，不应有冲突。如果冲突，在 examples feature 中显式指定 viser 兼容版本。

- [x] **Step 2: 验证核心包可导入**

Run:
```bash
cd /home/tyche/Documents/figaroh-plus
pixi run -e examples python -c "import viser; import hppfcl; import pinocchio; print('All imports OK')"
```
Expected: `All imports OK`
- `viser` 是 examples feature 新增依赖
- `hppfcl` 通过 `coal`（pin 的传递依赖）提供，无需额外声明
- `pinocchio` 由 `pin` 包提供

- [x] **Step 3: 验证 load_robot 的 models 目录解析**

Run:
```bash
cd /home/tyche/Documents/figaroh-plus
pixi run -e examples python -c "
from figaroh.tools.load_robot import _get_models_directory
import os
models_dir = _get_models_directory()
print(f'Models directory: {models_dir}')
assert os.path.isdir(models_dir), f'Not a directory: {models_dir}'
assert 'figaroh-examples' in models_dir, f'Unexpected path: {models_dir}'
print('Model resolution OK')
"
```
Expected:
```
Models directory: /path/to/figaroh-plus/figaroh-examples/models
Model resolution OK
```
确认 `_get_models_directory()` 通过 Method 1（`import figaroh_examples`）或 Method 3（相对路径解析）找到 `figaroh-examples/models/`。

如果 Method 1 失败（figaroh_examples 未安装为包），确认 Method 3 回退正常工作。设计文档说明无需修改 load_robot.py。

- [x] **Step 4: 确认 hppfcl 由 coal 传递提供**

Run:
```bash
cd /home/tyche/Documents/figaroh-plus
pixi run -e examples python -c "
import coal
print(f'coal version: {coal.__version__ if hasattr(coal, \"__version__\") else \"unknown\"}')
import hppfcl
print(f'hppfcl version: {hppfcl.__version__ if hasattr(hppfcl, \"__version__\") else \"unknown\"}')
print('hppfcl is available via coal transitive dependency')
"
```
Expected: 成功导入 hppfcl，说明 coal（pin 依赖）提供了 hppfcl。无需在 examples feature 中声明 hppfcl。

- [x] **Step 5: 提交 pixi.lock**

```bash
cd /home/tyche/Documents/figaroh-plus
git add pixi.lock
git commit -m "chore: regenerate pixi.lock with viser dependency

- pixi update after adding viser to examples feature
- Lock includes viser resolution for linux-aarch64, linux-64, osx-arm64"
```

archived-with: 2026-06-26-integrate-examples-submodule
---

### Task 5: UR10 例程验收 + 完整测试

**Files:**
- 无文件变更，仅执行验证命令

**Interfaces:**
- Consumes: Task 1-4 的所有输出
- Produces: 验证通过的完整 submodule 工作流

- [x] **Step 1: 运行 UR10 calibration 例程**

Run:
```bash
cd /home/tyche/Documents/figaroh-plus
pixi run -e examples python figaroh-examples/examples/ur10/calibration.py
```
Expected: 脚本正常执行（可能需要几分钟），输出标定结果或打印过程信息。如果脚本需要特定数据文件未找到，确认 UR10 data 目录下是否有示例数据。

如果脚本因缺少数据文件失败，记录错误信息但不阻塞——核心目标是验证依赖和模型路径可解析。脚本可能需要真实机器人数据才能完成完整执行。

- [x] **Step 2: 运行完整测试套件**

Run:
```bash
cd /home/tyche/Documents/figaroh-plus
pixi run test
```
Expected: 全部 212 个测试通过，输出类似：
```
collected 212 items
tests/... [100%]
======================== 212 passed in ... =
```

如果测试失败，检查是否与 submodule 集成或依赖变更相关。如果是既有失败（在 base-ref 上已存在），记录在验证报告中。

- [x] **Step 3: 验证 git submodule 状态**

Run:
```bash
cd /home/tyche/Documents/figaroh-plus
git submodule status
```
Expected: 输出以空格开头（表示 submodule 指针与当前 checkout 一致）：
```
 3983de5... figaroh-examples (heads/main)
```

archived-with: 2026-06-26-integrate-examples-submodule
---

### Task 6: 推送与提交——子模块 + 父仓库

**Files:**
- 无新文件变更；提交已有的所有变更

**Interfaces:**
- Consumes: Task 2 的 figaroh-examples 本地提交，Task 1/3/4 的 figaroh-plus 提交
- Produces: 远程仓库同步完成的完整集成

- [x] **Step 1: 推送 figaroh-examples 仓库清理变更**

Run:
```bash
cd /home/tyche/Documents/figaroh-plus/figaroh-examples
git push origin main
```
Expected: figaroh-examples 仓库的 `environment.yml` 删除、`requirements.txt` 删除、README 更新已推送到远程。

注意：如果无远程仓库写入权限，跳过此步并记录。后续可由仓库管理员手动推送。在推送之前，确认远程仓库 URL：
```bash
git remote -v
```

- [x] **Step 2: 更新 figaroh-plus 中 submodule 指针到最新**

如果在 Step 1 中推送成功，figaroh-examples 有了新的 commit。更新父仓库的 submodule 指针：

Run:
```bash
cd /home/tyche/Documents/figaroh-plus
git submodule update --remote figaroh-examples
cd figaroh-examples
git log --oneline -1
```
Expected: 显示最新的 commit（包含清理变更）。

如果 Step 1 跳过（无推送权限），则保持当前指针（3983de5），在 figaroh-plus 提交中包含清理变更的子模块工作树。后续当 figaroh-examples 远程更新后，再执行 `git submodule update --remote`。

实际上，由于子模块的变更在本地 figaroh-examples 工作树中已提交但未推送，`git submodule update --remote` 会尝试拉取远程最新，可能导致指针指向旧的远程版本。更好的做法是：直接在 figaroh-plus 中暂存 submodule 的当前本地 commit：

```bash
cd /home/tyche/Documents/figaroh-plus
git add figaroh-examples
```

- [x] **Step 3: 提交 figaroh-plus 最终变更（含 submodule 指针更新）**

查看当前暂存状态并提交：

Run:
```bash
cd /home/tyche/Documents/figaroh-plus
git status
```

Expected: 应包括 `.gitmodules`、`figaroh-examples`（submodule 指针）、`pyproject.toml`、`pixi.lock`、`.gitignore`、`README.md` 的变更。

```bash
git add -A
git commit -m "feat: integrate figaroh-examples as git submodule

- Add figaroh-examples submodule at commit 3983de5
- Remove redundant environment.yml and requirements.txt from examples repo
- Remove build>=1.5.0,<2 from project.dependencies (zero imports)
- Add viser to pixi examples feature (web-interface 3D visualization)
- Update READMEs with --recurse-submodules clone and pixi examples usage
- Regenerate pixi.lock with viser dependency resolution
- Clean up .gitignore and project URLs for submodule layout"
```

archived-with: 2026-06-26-integrate-examples-submodule
---

## Self-Review

### 1. 设计文档覆盖检查

| 设计文档节 | 对应任务 | 是否覆盖 |
|-----------|---------|---------|
| D1: 依赖链——无冗余声明 | Task 3 Step 1（删除 build），Task 4 Step 4（验证 hppfcl 传递） | Yes |
| D2: Submodule URL 和协议 | Task 1 Step 4（子模块添加），Task 3 Step 3（更新 URL） | Yes |
| D3: Submodule 变更工作流 | Task 6（推送 + 指针更新） | Yes |
| D4: 移除的依赖（build, cvxpy, environment.yml, requirements.txt） | Task 2 Step 1-2, Task 3 Step 1, Task 3 Step 2-3 | Yes |
| load_robot.py 兼容性（无需修改） | Task 4 Step 3（验证模型解析） | Yes |
| viser 集成 | Task 3 Step 2（添加到 pixi），Task 4 Step 2（导入验证） | Yes |
| pixi.lock 重建 | Task 4 Step 1 | Yes |
| 风险与缓解 | 全局约束 + Task 5（验证测试） | Yes |
| 验证方案（5 项检查） | Task 4 Step 2-3, Task 5 Step 1-3 | Yes |

### 2. 占位符检查

无 TBD、TODO、`implement later`、`fill in details`。所有步骤包含完整命令或代码。所有文件路径均为绝对路径或使用明确的相对路径前缀。

### 3. 类型/名称一致性检查

- submodule 名称 `figaroh-examples` 在 `.gitmodules`、`pyproject.toml` URL、`.gitignore` 注释、`README.md` 中一致
- `viser` 在 pyproject.toml examples feature 和 Task 4 验证命令中一致
- `build>=1.5.0,<2` 在删除和验证步骤中一致
- `environment.yml` 和 `requirements.txt` 在 figaroh-examples 和设计文档中一致
- 所有 README 更新指向 submodule 布局，与 `load_robot.py` 的 Method 3 回退链兼容
- URL 一致性：submodule 添加使用 `https://github.com/book-seed/figaroh-examples.git`，README 和 pyproject.toml 中使用 `https://github.com/book-seed/figaroh-examples`

### 4. 任务依赖关系

```
Task 1 (Submodule Setup)
  ├── Task 2 (figaroh-examples cleanup) — 需要在 submodule 内操作
  ├── Task 3 (figaroh-plus config) — 与 Task 2 独立
  │     └── Task 4 (pixi.lock rebuild) — 依赖 Task 3 的 pyproject.toml 变更
  │           └── Task 5 (Acceptance) — 依赖 Task 4 的 lock 文件
  └── Task 6 (Push & Commit) — 依赖所有前置任务
```

Task 2 和 Task 3 可以并行执行（分别在 submodule 和父仓库中独立操作），但建议按编号顺序执行以避免冲突。

archived-with: 2026-06-26-integrate-examples-submodule
---

Plan complete and saved to `docs/superpowers/plans/2026-06-26-integrate-examples-submodule.md`. Two execution options:

**1. Subagent-Driven (recommended)** — I dispatch a fresh subagent per task, review between tasks, fast iteration

**2. Inline Execution** — Execute tasks in this session using executing-plans, batch execution with checkpoints

**Which approach?**
