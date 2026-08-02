---
comet_change: relocate-casadi-cache-to-pixi-env
role: technical-design
canonical_spec: openspec
---

## Context

当前 `_cache_dir()` 硬编码 `~/.figaroh/casadi_cache/`。本工程是 pixi monorepo，figaroh 以 editable 方式运行。已查证事实：

- `.gitignore` 第 22 行忽略 `.pixi/`，但 `.cache/` 当前未忽略 → 缓存放 `.cache/` 需新增 gitignore 条目。
- `figaroh.__file__` 位于 `src/figaroh/__init__.py`，沿目录树向上查找 `pyproject.toml` 可定位工程根目录（纯 Python，不依赖 git 二进制）。
- 非工程环境（系统 pip install、无 `pyproject.toml` 祖先）fallback 保持可用。

## Goals / Non-Goals

**Goals:**

- 缓存放在工程根目录下 `.cache/figaroh/casadi/`，与工程绑定、不进版本控制。
- 不污染家目录。
- 非工程环境 fallback XDG 兼容路径 `~/.cache/figaroh/casadi/`。

**Non-Goals:**

- 不改指纹 + `_CACHE_VERSION` 失效机制。
- 不改 spec 缓存语义（仍"缓存到磁盘、按指纹失效"），只改路径定位。

## Decisions

- **工程根定位**：从 `figaroh.__file__` 沿目录树向上查找 `pyproject.toml`，所在目录即为工程根。纯 Python 实现，不依赖 git 二进制；在有 `PYTHONPATH`/`sys.path` 正确设置的环境下可靠。
- **缓存位置**：`<project>/.cache/figaroh/casadi/`。`.cache/` 前缀合 XDG 习惯，`figaroh/casadi/` 为未来其他缓存类型留空间（如 `figaroh/symbolic/`）。
- **fallback**：找不到 `pyproject.toml`（非工程安装，如系统 pip install）→ `~/.cache/figaroh/casadi/`（XDG 兼容，非家目录裸写）。
- **gitignore**：新增 `.cache/` 条目到工程 `.gitignore`。
- **现有家目录缓存处理**：路径变后旧 `~/.figaroh/casadi_cache/` 不再命中，首次运行重建到新路径。`_CACHE_VERSION` 不变（缓存格式不变），但路径变了自然触发重建。

## Risks / Trade-offs

- [缓存随工程不随 env] → 同一工程不同 env 共享缓存。可接受：casadi 符号函数由 URDF 惯性参数决定，不随 env 变；若不同 env 的 casadi 版本不兼容，`_CACHE_VERSION` + 指纹机制覆盖。
- [`.cache/` 需手动 gitignore] → 一次性在 `.gitignore` 加一行，风险极低。
- [现有家目录缓存失效] → 路径变后旧缓存不再命中，首次运行重建。可手动清理 `~/.figaroh/casadi_cache/`。
