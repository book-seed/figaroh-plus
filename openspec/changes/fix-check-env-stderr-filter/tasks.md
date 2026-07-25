# Tasks: fix-check-env-stderr-filter

- [x] 在 test_check_env.py stderr 过滤中追加排除 CasADi C++ warning 行
- [x] 修复 check_casadi_env.py 标题输出对齐 docstring (FIGAROH 环境验证脚本)
- [x] 重构 check_casadi_env.py 输出为分节编号 ([1] 基础依赖 / [2] IPOPT 线性求解器 / [3] CasADi OpenMP)
- [x] 运行 test_check_env 全部用例确认通过 (6 passed)
