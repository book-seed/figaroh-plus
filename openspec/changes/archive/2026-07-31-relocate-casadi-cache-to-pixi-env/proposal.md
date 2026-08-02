## Why

CasadiBackend 的符号模型缓存目前写到 `~/.figaroh/casadi_cache/`（用户家目录）。在本工程（pixi monorepo，figaroh 以 editable 方式运行）的使用场景下，这带来两个问题：缓存与工程脱钩——缓存放家目录，不同工程、不同 env 的 casadi 版本可能不同却共享同一家目录缓存，存在跨版本复用旧符号函数的隐患；缓存"隐藏"在家目录，不跟工程走。

将缓存重定位到工程根目录下 `.cache/figaroh/casadi/`：缓存与工程绑定；不污染家目录；且 `.cache/` 通过 `.gitignore` 忽略，缓存自动不进版本控制。非工程安装（无 `pyproject.toml`）时 fallback 到 XDG 兼容路径 `~/.cache/figaroh/casadi/`。

## What Changes

- `_cache_dir()` 从硬编码 `~/.figaroh/casadi_cache/` 改为：从 `figaroh.__file__` 沿目录树向上查找 `pyproject.toml` 定位工程根，缓存写到 `<project>/.cache/figaroh/casadi/`；找不到 `pyproject.toml` 时 fallback `~/.cache/figaroh/casadi/`（XDG 兼容）。
- 缓存指纹（`_cache_key`）与版本号（`_CACHE_VERSION`）失效机制不变，仍按 URDF 惯性参数 + 代码逻辑版本失效。
- delta spec：`symbolic-casadi-pipeline` 的缓存路径验收场景（"cached to disk at `~/.figaroh/casadi_cache/`"）MODIFIED 为新路径策略。
- `.gitignore`：新增 `.cache/` 条目。

## Capabilities

### New Capabilities

（无）

### Modified Capabilities

- `symbolic-casadi-pipeline`: 缓存路径定位策略变更——从固定家目录 `~/.figaroh/casadi_cache/` 改为工程根 `.cache/figaroh/casadi/` + 非工程环境 XDG fallback。原 "Symbolic model construction" scenario 的 THEN 子句 "cached to disk at `~/.figaroh/casadi_cache/`" 需 MODIFIED。缓存指纹 + 版本号失效语义不变。

## Impact

- `src/figaroh/backend/casadi.py`：`_cache_dir()`（line 142-146）改路径定位逻辑。
- `.gitignore`：新增 `.cache/` 条目。
- `openspec/specs/symbolic-casadi-pipeline/spec.md`：缓存路径验收场景 MODIFIED（通过 delta spec）。
- 现有家目录缓存（`~/.figaroh/casadi_cache/`）在新版本下不再被命中（路径变），首次运行重建到工程内；旧家目录缓存可手动清理。
