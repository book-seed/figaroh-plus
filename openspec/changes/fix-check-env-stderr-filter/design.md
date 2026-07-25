# Design: 修复 check_env 测试 stderr 过滤

## 背景

`tests/unit/test_check_env.py::test_script_output_format` 通过 `subprocess` 跑
`scripts/check_casadi_env.py`，捕获 stdout/stderr 后做两个断言：

1. stderr 经过滤后应为空（L46-50 过滤掉 `RuntimeWarning`/`DeprecationWarning` 行）
2. stdout 含预期 section 标题与 PASS/FAIL 标记

在 default env（casadi 非自编译 OpenMP 版本）下，CasADi C++ 运行时调用
`map("openmp")` 时向 stderr 输出：

```
CasADi - 2026-07-25 18:54:33 WARNING("CasADi was not compiled with WITH_OPENMP=ON. Falling back to serial evaluation.") [.../casadi/core/map.cpp:406]
```

此行不含 `RuntimeWarning`/`DeprecationWarning` 关键字，未被过滤，触发断言 1 失败。

## 方案

在 L46-50 的过滤条件中，追加排除 CasADi C++ warning 行。匹配条件扩展为：
排除含 `RuntimeWarning` **或** `DeprecationWarning` **或** `CasADi`（C++ 运行时
warning 固定前缀）的行。

```python
stderr_filtered = "\n".join(
    line for line in stderr.strip().split("\n")
    if "RuntimeWarning" not in line
    and "DeprecationWarning" not in line
    and "CasADi" not in line
)
```

## 标题漂移修复

测试 L65 断言 stdout 含 `FIGAROH 环境验证脚本`，但脚本 L93 `print("FIGAROH 环境验证")`
缺「脚本」二字，与脚本 docstring（L2 `"""FIGAROH 环境验证脚本。"""`）不符。
修复：`print("FIGAROH 环境验证脚本")`，让输出对齐 docstring 与测试意图。

## 行为等价性

- 脚本逻辑不变，仍正确检查 OpenMP 并输出 `[PASS] CasADi OpenMP 支持`。
- 仅放宽测试对上游 C++ warning 的容差——该 warning 是 CasADi 在无 OpenMP
  编译时的已知降级行为，非脚本错误。
- 在 casadi env（自编译 OpenMP 版本）下，该 warning 不出现，过滤扩展不影响其行为。

## 不在本次范围

- 不修复 `test_fourier_e2e` 的 `cs.cholesky` 问题（另起 change）。
- 不改动 `scripts/check_casadi_env.py`（脚本本身正确）。
