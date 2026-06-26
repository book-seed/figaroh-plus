# Tasks: Integrate figaroh-examples as Git Submodule

## 1. Submodule Setup

- [x] 1.1 删除当前 untracked `figaroh-examples/` 目录
- [x] 1.2 执行 `git submodule add https://github.com/book-seed/figaroh-examples.git figaroh-examples`
- [x] 1.3 进入 submodule，checkout main 分支，验证 commit `3983de5` 为最新

## 2. figaroh-examples 仓库清理

- [x] 2.1 删除 `environment.yml`
- [x] 2.2 删除 `requirements.txt`
- [x] 2.3 更新顶层 `README.md`：安装说明改为 pixi 使用方式
- [x] 2.4 更新 `examples/ur10/README.md`：删除 pip install 段（含错误 cvxpy），替换为 pixi 指令
- [x] 2.5 检查并更新其他 robot README（TIAGo 有 pip install 段，已替换；staubli_TX40, talos, templates 无需修改）

## 3. figaroh-plus 配置更新

- [x] 3.1 从 `pyproject.toml` `[project.dependencies]` 中移除 `build>=1.5.0,<2`
- [x] 3.2 pixi examples feature 添加 `viser`（pypi-dependencies）
- [x] 3.3 确认 `hppfcl` 已由 `coal`（pin 传递依赖）提供——design 阶段已验证，无需添加到 examples feature
- [x] 3.4 更新 `pyproject.toml` 中 Examples URL 为 `https://github.com/book-seed/figaroh-examples`
- [x] 3.5 更新 `.gitignore` 中 `# Examples and models moved to separate repository` 注释为 `# Examples and models managed as git submodule`
- [x] 3.6 更新 `README.md`：克隆说明改为 `git clone --recurse-submodules`，更新 examples URL

## 4. 依赖验证与重建

- [x] 4.1 运行 `pixi update` 重新生成 `pixi.lock`（viser v1.0.30 已锁定，三平台解析成功）
- [x] 4.2 验证 `pixi run -e examples python -c "import viser; import hppfcl; import pinocchio"` 成功
- [x] 4.3 验证 `_get_models_directory()` 正确解析 models 目录

## 5. UR10 例程验收

- [x] 5.1 运行 UR10 calibration——脚本成功启动，所有导入和依赖解析正常，URDF 文件路径正确；因缺少 `agimus-demos` ROS 包中的 mesh 文件在 pinocchio URDF 加载时失败（数据依赖，非代码/配置问题）
- [x] 5.2 运行 `pixi run test`，214 测试中 212 passed, 2 skipped（与基线一致）

## 6. 提交

- [x] 6.1 在 figaroh-examples 仓库提交清理变更（本地已提交 c21ee41）并推送到 main（需手动执行 `cd figaroh-examples && git push origin main`）
- [x] 6.2 更新 submodule 指针到最新 main commit（已指向 c21ee41）
- [x] 6.3 在 figaroh-plus 提交所有变更（含 `.gitmodules`、submodule 指针、配置更新）——5 个 commit 已在 feature 分支
