# Task 1 Report: Backend 包和抽象基类

## 1. 状态

**DONE**

## 2. 提交哈希

- `5052b17` feat(backend): add Backend ABC and create_backend factory

## 3. 测试结果

### RED（测试失败摘要）

```
ERROR tests/unit/test_backend.py
ModuleNotFoundError: No module named 'figaroh.backend.base'
```

预期失败：模块尚未创建，特征缺失。

### GREEN（通过摘要）

```
tests/unit/test_backend.py::TestBackendABC::test_backend_abc_cannot_be_instantiated PASSED
tests/unit/test_backend.py::TestBackendABC::test_concrete_backend_must_implement_abstract_methods PASSED
tests/unit/test_backend.py::TestBackendABC::test_concrete_backend_with_all_methods PASSED
tests/unit/test_backend.py::TestCreateBackendFactory::test_create_backend_numerical PASSED
tests/unit/test_backend.py::TestCreateBackendFactory::test_create_backend_default_is_numerical PASSED
tests/unit/test_backend.py::TestCreateBackendFactory::test_create_backend_passthrough PASSED
tests/unit/test_backend.py::TestCreateBackendFactory::test_create_backend_invalid_string PASSED
tests/unit/test_backend.py::TestCreateBackendFactory::test_create_backend_casadi_without_deps PASSED
8 passed in 1.43s
```

全量回归测试：**220 passed, 2 skipped**（零回归）

### 测试命令

```bash
cd /home/tyche/Documents/identification/figaroh-plus
pixi run python -m pytest tests/unit/test_backend.py -v
```

## 4. 变更文件

- `src/figaroh/backend/__init__.py` (创建) -- 包初始化，导出 Backend、BackendType、NumericalBackend、CasadiBackend（可选）、create_backend
- `src/figaroh/backend/base.py` (创建) -- Backend ABC（build_regressor、gradient、jacobian、create_solver、name）+ create_backend 工厂函数 + BackendType 类型别名
- `src/figaroh/backend/numerical.py` (创建) -- NumericalBackend 存根（实现 Backend ABC，所有方法抛出 NotImplementedError）
- `tests/unit/test_backend.py` (创建) -- ABC 契约测试（3 个）+ 工厂函数测试（5 个）

## 5. 顾虑或问题

### 5.1 额外文件：numerical.py

任务要求的"文件范围"仅列出 3 个文件（`__init__.py`、`base.py`、`test_backend.py`），但任务提供的 `__init__.py` 代码中 `from .numerical import NumericalBackend` 和测试中的 `from figaroh.backend.numerical import NumericalBackend` 均依赖 `numerical.py` 存在。因此额外创建了 `src/figaroh/backend/numerical.py` 作为存根实现。

该存根的所有抽象方法当前抛出 `NotImplementedError`，将在后续任务（Task 2: NumericalBackend）中完整实现。
