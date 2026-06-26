# Brainstorm Summary

- Change: integrate-examples-submodule
- Date: 2025-06-26

## 确认的技术方案

Submodule 集成 + pixi 依赖统一。关键技术验证结果：

1. **hppfcl**: 由 `coal`（pin 传递依赖）提供，无需在 pixi 中单独声明
2. **viser**: 需在 pixi examples feature 中添加（pypi-dependency），与 pinocchio 无依赖冲突
3. **pinocchio**: PyPI `pin` v4.0.0 已提供 pinocchio 模块，三平台均已锁定，无需额外 conda 依赖
4. **Submodule 工作流**: 标准流程——先推 examples 仓库变更，再更新父仓库 submodule 指针
5. **UR10 验收**: 添加 viser 后 examples 环境依赖即完整

## 关键取舍与风险

- 取舍：删除 `environment.yml`/`requirements.txt` 后，figaroh-examples 不再有独立依赖声明（由父仓库 pixi 接管）
- 风险：`viser` 版本可能在未来与 pixi lock 中其他包冲突 → 缓解：pixi 的 solve-group 机制会检测冲突
- 取舍：HTTPS URL 对公共 clone 友好，SSH 用户通过 git insteadOf 切换 → 无需在 `.gitmodules` 中特殊处理

## 测试策略

- pixi examples 环境依赖导入验证（viser, hppfcl, pinocchio, jupyter）
- models 目录路径解析验证
- UR10 calibration 例程端到端运行
- 现有 212 个测试回归

## Spec Patch

无。现有 pixi-environment spec 的修改在实现阶段通过 delta spec 体现。
