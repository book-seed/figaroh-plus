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
