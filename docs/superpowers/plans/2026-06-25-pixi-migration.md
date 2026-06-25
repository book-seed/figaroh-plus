---
change: migrate-to-pixi
design-doc: docs/superpowers/specs/2026-06-25-pixi-migration-design.md
base-ref: 793aff08702102f0cbc6e6c12c220da5ea6af55f
---

# pixi 迁移实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 figaroh-plus 的依赖管理从 conda + pip 手动模式迁移到 pixi Feature 分层组合模式，精简 pyproject.toml 为构建元数据骨架。

**Architecture:** pixi.toml 以 Feature 分层组织依赖（core/dev/docs/examples），通过环境矩阵组合不同场景。pyproject.toml 仅保留构建元数据，依赖声明全部迁移至 pixi.toml。CI 使用 prefix-dev/setup-pixi action 替代手动安装步骤。

**Tech Stack:** pixi (prefix.dev 包管理器), conda-forge (cyipopt 预编译), PyPI (其余纯 Python 包), hatchling (构建后端)

## Global Constraints

- Python ==3.12.*，与当前 Tegra 环境精确匹配
- cyipopt 必须走 conda-forge channel（pip 无法直接安装，需要编译 + coin-libipopt）
- pin 和 rospkg 走 PyPI（conda-forge 中 pin 是系统工具 pinentry，rospkg osx-arm64 缺失）
- ndcurves、numdifftools、picos 走 PyPI（conda-forge aarch64 缺失）
- pixi.lock 必须包含 linux-aarch64、linux-64、osx-arm64 三平台解析条目
- 所有变更基于 base-ref `793aff08702102f0cbc6e6c12c220da5ea6af55f`

---

## File Structure

| 文件 | 操作 | 职责 |
|------|------|------|
| `pixi.toml` | **创建** | pixi 项目配置：feature 分层、环境组合、task 定义 |
| `pixi.lock` | **创建**（自动生成） | 依赖锁文件，确保可复现环境 |
| `pyproject.toml` | **修改** | 删除 `[project.dependencies]` 和 `[project.optional-dependencies]`，保留构建元数据，添加注释说明依赖由 pixi 管理 |
| `environment.yml` | **删除** | 旧 conda 环境配置，不再需要 |
| `.github/workflows/docs.yml` | **修改** | 替换为 prefix-dev/setup-pixi action，删除手动 apt-get/pip install 步骤 |
| `.gitignore` | **修改** | 添加 `.pixi/` 目录排除 |
| `README.md` | **修改** | 添加 pixi quickstart 开发环境说明 |

---

### Task 1: 创建 pixi.toml（Feature + 环境 + Task 完整配置）

**Files:**
- Create: `pixi.toml`

**Interfaces:**
- Consumes: 设计文档中 Feature 分层和环境矩阵定义
- Produces: 完整的 `pixi.toml`，供 Task 2 生成 lock 文件

- [x] **Step 1: 创建 pixi.toml 文件**

在项目根目录创建 `pixi.toml`，内容如下：

```toml
[project]
name = "figaroh"
version = "0.4.3"
platforms = ["linux-aarch64", "linux-64", "osx-arm64"]
channels = ["conda-forge"]

[tasks]

[tasks.shell]
cmd = "bash"

[tasks.test]
cmd = "pytest tests/"

[tasks.lint]
cmd = "flake8 src/ && mypy src/"

[tasks.format]
cmd = "black src/ tests/ && isort src/ tests/"

[tasks.clean]
cmd = "rm -rf dist/ build/ *.egg-info"

[tasks.build]
cmd = "python -m build"

[tasks.docs]
cmd = "cd docs && make html"

[feature.core.dependencies]
python = "3.12.*"
cyipopt = { version = "*", channel = "conda-forge" }

[feature.core.pypi-dependencies]
numpy = "*"
scipy = "*"
pin = "*"
matplotlib = "*"
pyyaml = "*"
meshcat = "*"
pandas = "*"
rospkg = "*"
picos = "*"
numdifftools = "*"
ndcurves = "*"
figaroh = { path = ".", editable = true }

[feature.dev.pypi-dependencies]
pytest = ">=6.0"
pytest-cov = "*"
black = "*"
isort = "*"
flake8 = "*"
mypy = "*"

[feature.docs.pypi-dependencies]
sphinx = "*"
sphinx-rtd-theme = "*"
sphinx-autodoc-typehints = "*"

[feature.examples.pypi-dependencies]
jupyter = "*"
notebook = "*"
ipywidgets = "*"

[environments]
default = { features = ["core", "dev", "docs"], solve-group = "default" }
docs = { features = ["core", "docs"], solve-group = "default" }
test = { features = ["core", "dev"], solve-group = "default" }
examples = { features = ["core", "examples"], solve-group = "default" }
```

- [x] **Step 2: 验证 pixi.toml 语法**

Run: `pixi validate`
Expected: `The project is valid` 或类似无错误输出。如果 pixi 未安装或命令不同，运行 `pixi project validate` 或检查 `pixi --help`。

- [x] **Step 3: 提交**

```bash
git add pixi.toml
git commit -m "feat(pixi): add pixi.toml with feature-based dependency management

- Define core/dev/docs/examples features
- Configure default/docs/test/examples environments
- Add 7 tasks: shell, test, lint, format, docs, build, clean
- Set platforms to linux-aarch64, linux-64, osx-arm64
- Channel: conda-forge"
```

---

### Task 2: 生成 pixi.lock 锁文件

**Files:**
- Create: `pixi.lock`（自动生成）
- Modify: 无

**Interfaces:**
- Consumes: Task 1 的 `pixi.toml`
- Produces: `pixi.lock`，供后续任务验证依赖一致性

- [x] **Step 1: 运行 pixi install 生成 lock 文件**

Run: `pixi install`
Expected: pixi 解析依赖并生成 `pixi.lock`，输出类似于 "Created pixi.lock" 或 "Lock file generated"。首次安装可能需要 5-10 分钟（在 aarch64 上），因 SAT solver 需解析大量依赖。

如果遇到 cyipopt 或 numpy/scipy 版本冲突，观察 solver 输出中冲突信息。手动干预方式：在 `[feature.core.dependencies]` 中为冲突包显式指定兼容版本。

- [x] **Step 2: 验证 pixi.lock 包含三平台解析条目**

Run: `grep -E "linux-aarch64|linux-64|osx-arm64" pixi.lock | head -5`
Expected: 输出中至少出现 `linux-aarch64`、`linux-64`、`osx-arm64` 三个平台标识之一，确认 lock 包含多平台解析。

- [x] **Step 3: 验证 default 环境可用**

Run: `pixi shell`（或 `pixi run python -c "import figaroh; print(figaroh.__version__)"`）
Expected: 成功导入 figaroh，打印版本号 `0.4.3`。如果 `pixi shell` 需要交互式终端，使用 `pixi run` 替代。

- [x] **Step 4: 提交**

```bash
git add pixi.lock
git commit -m "feat(pixi): generate pixi.lock with multi-platform dependency resolution

- Generated via pixi install on linux-aarch64
- Includes resolutions for linux-aarch64, linux-64, osx-arm64
- Verified figaroh imports correctly in default environment"
```

---

### Task 3: 精简 pyproject.toml——删除依赖声明，保留构建元数据

**Files:**
- Modify: `pyproject.toml`

**Interfaces:**
- Consumes: `pyproject.toml` 当前内容（已在 base-ref 中有完整依赖声明）
- Produces: 精简后的 `pyproject.toml`，供 Task 7 验证 `pixi run build` 可构建 wheel

- [x] **Step 1: 删除 `[project.dependencies]` 节（第43-55行）**

删除如下内容：
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

- [x] **Step 2: 删除 `[project.optional-dependencies]` 节（第57-76行）**

删除如下内容：
```toml
[project.optional-dependencies]
dev = [
    "pytest>=6.0",
    "pytest-cov",
    "black",
    "isort",
    "flake8",
    "mypy",
    "pre-commit",
]
docs = [
    "sphinx",
    "sphinx-rtd-theme",
    "sphinx-autodoc-typehints",
]
examples = [
    "jupyter",
    "notebook",
    "ipywidgets",
]
```

- [x] **Step 3: 在 `[project]` 元数据后添加注释**

在 `classifiers` 列表之后（原本 `dependencies` 所在位置之前）添加：
```toml
# Dependencies are managed by pixi.toml — run `pixi install` to set up the environment
```

修改后 `pyproject.toml` 的 `[project]` 节应如下（确保保留所有构建元数据）：

```toml
[build-system]
requires = ["hatchling"]
build-backend = "hatchling.build"

[project]
name = "figaroh"
version = "0.4.3"
description = "A Python toolbox for dynamics identification and geometric calibration of robots and humans"
readme = "README.md"
requires-python = ">=3.8"
license = { file = "LICENSE" }
authors = [
    { name = "Thanh D. V. Nguyen", email = "thanhndv212@gmail.com" },
]
maintainers = [
    { name = "Thanh D. V. Nguyen", email = "thanhndv212@gmail.com" },
]
keywords = [
    "robotics",
    "dynamics",
    "calibration",
    "identification",
    "urdf",
    "pinocchio",
    "optimization",
]
classifiers = [
    "Development Status :: 4 - Beta",
    "Intended Audience :: Science/Research",
    "Intended Audience :: Developers",
    "License :: OSI Approved :: BSD License",
    "Operating System :: OS Independent",
    "Programming Language :: Python :: 3",
    "Programming Language :: Python :: 3.8",
    "Programming Language :: Python :: 3.9",
    "Programming Language :: Python :: 3.10",
    "Programming Language :: Python :: 3.11",
    "Programming Language :: Python :: 3.12",
    "Topic :: Scientific/Engineering :: Artificial Intelligence",
    "Topic :: Scientific/Engineering :: Physics",
    "Topic :: Software Development :: Libraries :: Python Modules",
]
# Dependencies are managed by pixi.toml — run `pixi install` to set up the environment

[tool.hatch.build.targets.wheel]
packages = ["src/figaroh"]

[tool.hatch.build.targets.wheel.force-include]

[tool.hatch.build.targets.sdist]
exclude = [
    "examples/",
    "models/",
    "docs/build/",
]

[project.urls]
Homepage = "https://github.com/thanhndv212/figaroh-plus"
Repository = "https://github.com/thanhndv212/figaroh-plus"
Issues = "https://github.com/thanhndv212/figaroh-plus/issues"
Examples = "https://github.com/thanhndv212/figaroh-examples"
```

- [x] **Step 4: 提交**

```bash
git add pyproject.toml
git commit -m "refactor(pyproject): remove dependency declarations, now managed by pixi.toml

- Delete [project.dependencies] and [project.optional-dependencies]
- Add comment noting dependencies are managed by pixi
- Preserve build-system, project metadata, hatch config, and project.urls"
```

---

### Task 4: 清理旧配置文件

**Files:**
- Delete: `environment.yml`
- Modify: `.gitignore`

**Interfaces:**
- Produces: 无依赖残留，`environment.yml` 已删除，`.gitignore` 排除 `.pixi/`

- [x] **Step 1: 删除 environment.yml**

Run: `git rm environment.yml`

- [x] **Step 2: 更新 .gitignore——添加 `.pixi/` 目录排除**

在 `.gitignore` 末尾添加一行：
```
# pixi environment directory
.pixi/
```

修改后 `.gitignore` 完整内容：
```gitignore
# Private planning docs
ROADMAP_PRIVATE.md
old_files/
data_list.docx
**.pyc
**/.DS_Store
build/*
cmake/*
.vscode/*
.github/skills

# Python packaging
dist/
*.egg-info/
__pycache__/

# Examples and models moved to separate repository
# See: https://github.com/thanhndv212/figaroh-examples

# pixi environment directory
.pixi/
```

- [x] **Step 3: 提交**

```bash
git add environment.yml .gitignore
git commit -m "chore: remove environment.yml and add .pixi/ to .gitignore

- environment.yml replaced by pixi.toml
- .pixi/ is pixi's local environment directory, should not be tracked"
```

---

### Task 5: 更新 CI 工作流

**Files:**
- Modify: `.github/workflows/docs.yml`

**Interfaces:**
- Consumes: 现有 CI workflow（base-ref 状态）
- Produces: 使用 pixi action 的 CI workflow

- [x] **Step 1: 替换 docs.yml 中的依赖安装步骤**

将 `.github/workflows/docs.yml` 替换为以下内容：

```yaml
name: Build and Deploy Docs

on:
  push:
    branches: [ main ]
  pull_request:
    branches: [ main ]

permissions:
  contents: write
  pages: write
  id-token: write

jobs:
  build-and-deploy:
    runs-on: ubuntu-latest
    steps:
      - name: Checkout
        uses: actions/checkout@v4

      - name: Setup pixi
        uses: prefix-dev/setup-pixi@v0
        with:
          pixi-version: latest
          cache: true
          environments: docs

      - name: Build Documentation
        run: |
          pixi run -e docs docs
          cd docs/build/html && touch .nojekyll

      - name: Deploy to GitHub Pages
        uses: JamesIves/github-pages-deploy-action@v4
        with:
          token: ${{ secrets.GITHUB_TOKEN }}
          branch: gh-pages
          folder: docs/build/html
          clean: true
          force: true
```

注意：`pixi run -e docs docs` 中第一个 `docs` 为 task 名称，`-e docs` 指定 docs 环境。如果 pixi 不支持 `-e` 参数（取决于版本），改用 `pixi run --environment docs docs`。验证本地 pixi 版本支持的语法。

- [x] **Step 2: 验证 workflow YAML 语法**

Run: `python -c "import yaml; yaml.safe_load(open('.github/workflows/docs.yml')); print('YAML valid')"`
Expected: 输出 "YAML valid"

如果没有安装 pyyaml，使用系统 python3 运行：
Run: `python3 -c "import yaml; yaml.safe_load(open('.github/workflows/docs.yml')); print('YAML valid')" 2>/dev/null || echo "yaml not available, check manually"`

- [x] **Step 3: 提交**

```bash
git add .github/workflows/docs.yml
git commit -m "ci(docs): migrate to pixi-based setup in CI workflow

- Replace actions/setup-python@v4 with prefix-dev/setup-pixi@v0
- Remove manual apt-get and pip install steps
- Use pixi run -e docs docs for environment-aware doc build
- Update actions/checkout to v4"
```

---

### Task 6: 更新 README.md 开发环境说明

**Files:**
- Modify: `README.md`

**Interfaces:**
- Produces: README 中包含 pixi quickstart，替换旧 conda 安装指南

- [x] **Step 1: 替换 Installation 节中的 conda 方法**

将现有 "Development Installation" 节（第23-41行）替换为：

```markdown
### Development Installation

For development or local installation from source:

**Method 1: pixi (Recommended)**
```bash
git clone https://github.com/thanhndv212/figaroh-plus.git
cd figaroh-plus
# Install all dependencies and activate default environment
pixi install
# Activate the development environment
pixi shell
```

**Method 2: Direct pip installation (Simple)**
```bash
git clone https://github.com/thanhndv212/figaroh-plus.git
cd figaroh-plus
pip install -e .
```
Note: Method 2 may not include cyipopt and other conda-managed dependencies.

### pixi Environments

pixi provides multiple pre-configured environments for different use cases:

| Environment | Features | Use Case |
|-------------|----------|----------|
| `default` | core + dev + docs | Daily development |
| `docs` | core + docs | Documentation building |
| `test` | core + dev | Running tests |
| `examples` | core + examples | Jupyter notebooks |

Switch environments with:
```bash
pixi run -e test pytest tests/
pixi run -e docs docs
```

### pixi Tasks

Common development tasks are configured as pixi tasks:

```bash
pixi run test     # Run pytest
pixi run lint     # Run flake8 + mypy
pixi run format   # Run black + isort
pixi run build    # Build wheel
pixi run docs     # Build documentation
```
```

- [x] **Step 2: 提交**

```bash
git add README.md
git commit -m "docs(readme): add pixi quickstart and environment documentation

- Replace conda installation method with pixi
- Document pixi environments and tasks
- Keep pip installation as secondary option"
```

---

### Task 7: 完整集成验证

**Files:**
- 无文件变更，仅执行验证命令

**Interfaces:**
- Consumes: 全部 Task 1-6 的输出
- Produces: 验证通过的完整 pixi 工作流

- [ ] **Step 1: 运行 pixi validate**

Run: `pixi validate`
Expected: 无错误输出，确认 pixi.toml 和 lock 文件一致

- [ ] **Step 2: 验证所有环境可安装**

Run:
```bash
pixi install                  # default 环境
pixi install -e docs          # docs 环境
pixi install -e test          # test 环境
```
Expected: 每个环境都能成功安装。注意：lock 已生成的情况下，后续 install 应快很多（秒级）。

- [ ] **Step 3: 运行 `pixi run shell`——验证 figaroh 可导入**

Run: `pixi run python -c "import figaroh; print(figaroh.__version__)"`
Expected: 输出 `0.4.3`

- [ ] **Step 4: 运行 `pixi run test`——验证测试通过**

Run: `pixi run test`
Expected: pytest 运行并通过所有测试。注意：如部分测试需要特殊硬件或长时间运行，可确认测试框架启动正常即可。

- [ ] **Step 5: 运行 `pixi run lint`——验证代码检查**

Run: `pixi run lint`
Expected: flake8 和 mypy 无报错。如果 mypy 配置较严格有类型错误，先确认这些错误是否在迁移前就存在（可通过 `git stash` 切回 base-ref 对比）。

- [ ] **Step 6: 运行 `pixi run build`——验证 wheel 可构建**

Run: `pixi run build`
Expected: 在 `dist/` 目录下生成 `.whl` 和 `.tar.gz` 文件。验证产物：
Run: `ls dist/`
Expected: 包含 `figaroh-0.4.3-*.whl` 和 `figaroh-0.4.3.tar.gz`

- [ ] **Step 7: 运行 `pixi run docs`——验证文档可构建**

Run: `pixi run docs`
Expected: docs 构建成功，`docs/build/html/` 目录中包含 `index.html`。

- [ ] **Step 8: 清理 build 产物**

Run: `pixi run clean`
Expected: `dist/`、`build/`、`*.egg-info` 被删除。

---

## Self-Review

### 1. 设计文档覆盖检查

| 设计文档节 | 对应任务 | 是否覆盖 |
|-----------|---------|---------|
| 2.1 Feature 分层——core | Task 1 | yes |
| 2.1 Feature 分层——dev/docs/examples | Task 1 | yes |
| 2.2 关键决策（pin/rospkg PyPI，cyipopt conda-forge） | Task 1 | yes |
| 2.3 环境矩阵——4 个环境 | Task 1 | yes |
| 3. Tasks 设计——7 个 task | Task 1 | yes |
| 4. pyproject.toml 精简 | Task 3 | yes |
| 5. CI 迁移 | Task 5 | yes |
| 6. 风险与缓解（R1-R5） | 全局约束 | yes |
| 7. 测试策略 | Task 7 | yes |
| 8. 文件变更清单——7 个文件 | Task 1-6 | yes |

### 2. 占位符检查

无 TBD、TODO、`implement later`、`fill in details`。全部步骤包含完整代码或命令。

### 3. 类型/名称一致性检查

- `pixi.toml` 中 feature 名称与 environments 引用一致：core, dev, docs, examples
- task 名称与设计文档一致：shell, test, lint, format, docs, build, clean
- pyproject.toml 中 `name = "figaroh"` 与 pixi.toml 中 `name = "figaroh"` 一致
- 环境名称 default/docs/test/examples 在 README 和 pixi.toml 中一致
- 平台列表 `linux-aarch64, linux-64, osx-arm64` 在 Task 1 和 CI（ubuntu-latest = linux-64）中一致

---

Plan complete and saved to `docs/superpowers/plans/2026-06-25-pixi-migration.md`. Two execution options:

**1. Subagent-Driven (recommended)** — I dispatch a fresh subagent per task, review between tasks, fast iteration

**2. Inline Execution** — Execute tasks in this session using executing-plans, batch execution with checkpoints

**Which approach?**
