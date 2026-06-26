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
