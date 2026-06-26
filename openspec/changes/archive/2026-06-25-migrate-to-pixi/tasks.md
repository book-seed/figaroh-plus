## 1. 前期调研

- [x] 1.1 验证所有依赖在 conda-forge 三平台（linux-aarch64、linux-64、osx-arm64）的可用性，记录不可用包及 fallback 方案
- [x] 1.2 确认 `pin` 包在 conda-forge/PyPI 中的实际名称，避免名称冲突

## 2. pixi.toml 配置

- [x] 2.1 初始化 pixi 项目：`pixi init`，设置三平台 `platforms = ["linux-aarch64", "linux-64", "osx-arm64"]`
- [x] 2.2 添加 conda 依赖：`python`（>=3.8）、`cyipopt`（conda-forge channel）
- [x] 2.3 添加 PyPI 依赖：`numpy`、`scipy`、`pin`、`matplotlib`、`pyyaml`、`meshcat`、`pandas`、`rospkg`、`picos`、`numdifftools`、`ndcurves`
- [x] 2.4 添加 dev PyPI 依赖：`pytest`(>=6.0)、`pytest-cov`、`black`、`isort`、`flake8`、`mypy`
- [x] 2.5 添加 docs PyPI 依赖：`sphinx`、`sphinx-rtd-theme`、`sphinx-autodoc-typehints`
- [x] 2.6 定义 `default`、`docs`、`test` 三个 feature/environment
- [x] 2.7 定义 7 个 tasks：`test`、`lint`、`format`、`docs`、`build`、`shell`、`clean`

## 3. pixi.lock 生成

- [x] 3.1 运行 `pixi install` 在 linux-aarch64（当前平台）生成 lock
- [x] 3.2 验证 `pixi.lock` 包含三平台解析条目

## 4. pyproject.toml 精简

- [x] 4.1 删除 `[project.dependencies]` 节
- [x] 4.2 删除 `[project.optional-dependencies]` 节（dev、docs、examples）
- [x] 4.3 保留 `[build-system]` 和 `[project]` 元数据（name、version、authors 等），添加注释说明依赖由 pixi.toml 管理

## 5. 旧文件清理

- [x] 5.1 删除 `environment.yml`
- [x] 5.2 更新 `.gitignore`，添加 `.pixi/` 目录排除

## 6. CI 工作流更新

- [x] 6.1 更新 `.github/workflows/docs.yml`，使用 pixi 安装依赖和构建文档
- [x] 6.2 验证更新后的 workflow 语法正确（至少本地检查）

## 7. 验证

- [x] 7.1 运行 `pixi run shell` 确认开发环境可用，`import figaroh` 成功
- [x] 7.2 运行 `pixi run test` 确认 pytest 通过
- [x] 7.3 运行 `pixi run lint` 确认代码检查正常
- [x] 7.4 运行 `pixi run docs` 确认文档可构建
- [x] 7.5 运行 `pixi run build` 确认 wheel 可构建
- [x] 7.6 更新 `README.md` 开发环境说明，添加 pixi quickstart

<!-- review: standard mode, final review APPROVED (3 Minor accepted):
1. osx-64 excluded: intentional — Tier 2 only osx-arm64 per design; Intel Mac not primary target
2. pre-commit removed from pixi.toml: intentional — pre-commit managed via .pre-commit-config.yaml, installed separately
3. pip install -e . missing deps: accepted — README already warns Method 2 skips conda deps; pixi is primary path
-->
