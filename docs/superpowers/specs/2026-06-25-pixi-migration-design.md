---
comet_change: migrate-to-pixi
role: technical-design
canonical_spec: openspec
---

# pixi 迁移技术设计

## 1. 架构总览

将 figaroh-plus 的依赖管理从 conda + pip 手动模式迁移到 pixi Feature 分层组合模式，pyproject.toml 精简为构建元数据骨架。

```
┌─────────────────────────────────────────────────────────────┐
│                     pixi.toml                               │
│  ┌──────────────────────────────────────────────────────┐  │
│  │ [feature.core]    [feature.dev]    [feature.docs]     │  │
│  │ python, cyipopt   pytest, black,   sphinx,            │  │
│  │ numpy, scipy,     isort, flake8,   sphinx-rtd-theme   │  │
│  │ pin, matplotlib,  mypy,            [feature.examples]  │  │
│  │ pyyaml, meshcat,  pytest-cov        jupyter, notebook  │  │
│  │ pandas, rospkg,                                       │  │
│  │ picos, ndcurves,                                      │  │
│  │ figaroh (editable)                                    │  │
│  └──────────────────────────────────────────────────────┘  │
│                                                             │
│  [environments]                                             │
│  default = core + dev + docs                               │
│  docs    = core + docs                                     │
│  test    = core + dev                                      │
│  examples = core + examples                                │
│                                                             │
│  [tasks]  test | lint | format | docs | build | shell | clean │
└─────────────────────────────────────────────────────────────┘
```

## 2. 环境与依赖设计

### 2.1 Feature 分层

| Feature | 依赖类型 | 内容 |
|---------|---------|------|
| `core` | conda + PyPI | python==3.12.*, cyipopt(conda-forge), numpy, scipy, pin, matplotlib, pyyaml, meshcat, pandas, rospkg, picos, numdifftools, ndcurves, figaroh(editable) |
| `dev` | PyPI | pytest>=6.0, pytest-cov, black, isort, flake8, mypy |
| `docs` | PyPI | sphinx, sphinx-rtd-theme, sphinx-autodoc-typehints |
| `examples` | PyPI | jupyter, notebook, ipywidgets |

### 2.2 关键决策

**pin / rospkg 走 PyPI**：
- `pin` 在 conda-forge 中为系统工具 pinentry，PyPI 才是 Python 包 `pin`
- `rospkg` 为纯 Python 包，PyPI 三平台可用，避免 conda-forge osx-arm64 缺失

**cyipopt 走 conda-forge**：唯一 conda 依赖。pip 无法直接安装（需要编译 + coin-libipopt），conda 提供预编译包。

**python ==3.12.\***：与当前 Tegra 环境精确匹配，已验证可用。未来升级时修改此处 + 重新 `pixi update`。

### 2.3 环境矩阵

| 环境 | Feature 组合 | 典型场景 |
|------|-------------|---------|
| `default` | core + dev + docs | 日常开发 |
| `docs` | core + docs | CI 文档构建（无 GUI 依赖） |
| `test` | core + dev | CI 测试（无 GUI 依赖、无文档依赖） |
| `examples` | core + examples | 运行 Jupyter 示例 |

## 3. Tasks 设计

| Task | 环境 | 命令 | 依赖 |
|------|------|------|------|
| `shell` | default | `bash` | — |
| `test` | default | `pytest tests/` | figaroh |
| `lint` | default | `flake8 src/ && mypy src/` | figaroh |
| `format` | default | `black src/ tests/ && isort src/ tests/` | figaroh |
| `docs` | docs | `cd docs && make html` | figaroh |
| `build` | default | `python -m build` | — |
| `clean` | default | `rm -rf dist/ build/ *.egg-info` | — |

每个 task 在其关联环境中执行。`depends-on = ["figaroh"]` 确保项目可编辑安装先于 task 内部命令完成。

## 4. pyproject.toml 精简

- **删除**：`[project.dependencies]`（第43-55行）、`[project.optional-dependencies]`（第57-76行）
- **保留**：`[build-system]`、`[project]` 元数据、`[tool.hatch.build.targets]`、`[project.urls]`
- **添加注释**：`# Dependencies are managed by pixi.toml — see pixi install`

## 5. CI 迁移

`.github/workflows/docs.yml` 变更：

| 步骤 | 当前 | 迁移后 |
|------|------|--------|
| Python setup | `actions/setup-python@v4` | `prefix-dev/setup-pixi@v0`（含缓存） |
| 系统依赖 | `apt-get install pkg-config coinor-libipopt-dev libblas-dev liblapack-dev` | 删除（pixi 通过 conda-forge 管理） |
| Python 依赖 | `pip install sphinx numpy scipy ...` | 删除（pixi install 自动处理） |
| 文档构建 | `cd docs && make html` | `pixi run docs`（内部含 `depends-on`） |

## 6. 风险与缓解

| # | 风险 | 缓解 |
|---|------|------|
| R1 | ndcurves/numdifftools/picos 在 conda-forge aarch64 缺失 | 走 PyPI（纯 Python 包） |
| R2 | cyipopt 与 numpy/scipy 版本冲突 | pixi SAT solver 自动检测；必要时人工降级 |
| R3 | pixi solve 在 aarch64 超时 | 首次 5-10min 正常；lock 永久缓存 |
| R4 | CI setup-pixi 下载慢 | action 自带缓存 |
| R5 | 旧开发者不熟悉 pixi | README 添加 3 行 quickstart |

### 回滚策略

```bash
pixi clean cache
git checkout HEAD~1          # 恢复旧环境文件
conda env create -f environment.yml
```

## 7. 测试策略

```
集成验证  ← pixi run test / lint / format / docs / build
环境验证  ← pixi install (3环境) + import figaroh
静态检查  ← pixi validate / pyproject.toml / YAML syntax
```

跨平台验证通过检查 `pixi.lock` 中是否包含 `linux-64` 和 `osx-arm64` 平台的解析条目间接完成——当前 Tegra 单机无法直接运行其他平台。

## 8. 文件变更清单

| 文件 | 操作 |
|------|------|
| `pixi.toml` | **新增** |
| `pixi.lock` | **新增**（自动生成） |
| `pyproject.toml` | **修改**（精简） |
| `environment.yml` | **删除** |
| `.github/workflows/docs.yml` | **修改** |
| `.gitignore` | **修改** |
| `README.md` | **修改**（添加 pixi quickstart） |
