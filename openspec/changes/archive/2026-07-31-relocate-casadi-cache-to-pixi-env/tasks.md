## 1. 缓存路径实现

- [x] 1.1 改 `src/figaroh/backend/casadi.py` 的 `_cache_dir()`：从 `figaroh.__file__` 沿目录树向上查找 `pyproject.toml` 定位工程根，缓存写到 `<project>/.cache/figaroh/casadi/`；找不到 `pyproject.toml` 时 fallback `~/.cache/figaroh/casadi/`。
- [x] 1.2 `.gitignore` 新增 `.cache/` 条目。

## 2. 测试

- [x] 2.1 加测试：工程环境缓存路径落在 `.cache/figaroh/casadi/`、非工程环境 fallback `~/.cache/figaroh/casadi/`、指纹 + `_CACHE_VERSION` 失效机制仍生效。

## 3. spec delta

- [x] 3.1 写 `symbolic-casadi-pipeline` delta spec：MODIFIED "Symbolic model construction" scenario 的缓存路径 THEN 子句（工程根 `.cache/figaroh/casadi/` + XDG fallback），指纹 + 版本号失效语义不变。
