# Comet Design Handoff

- Change: relocate-casadi-cache-to-pixi-env
- Phase: design
- Mode: compact
- Context hash: 754f8c385aeabb562b992917b8e464c9ef2598ad211e588e6706720f8612611d

Generated-by: comet-handoff.sh

OpenSpec remains the canonical capability spec. This handoff is a deterministic, source-traceable context pack, not an agent-authored summary.

## openspec/changes/relocate-casadi-cache-to-pixi-env/proposal.md

- Source: openspec/changes/relocate-casadi-cache-to-pixi-env/proposal.md
- Lines: 1-29
- SHA256: 8a2efc8a7057a52a891e357d13ddc9a295372ca029ad544c3586a47251e4f4da

```md
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
```

## openspec/changes/relocate-casadi-cache-to-pixi-env/design.md

- Source: openspec/changes/relocate-casadi-cache-to-pixi-env/design.md
- Lines: 1-40
- SHA256: 8caf9080d290130cbe8919a835a209c4c7ddefe4d0bac1788e0efbe21e6f65db

```md
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
```

## openspec/changes/relocate-casadi-cache-to-pixi-env/tasks.md

- Source: openspec/changes/relocate-casadi-cache-to-pixi-env/tasks.md
- Lines: 1-12
- SHA256: 14b89e8943a1d36fa8422a8a0529738ede3e3dc488941459ba87f003b8a73c85

```md
## 1. 缓存路径实现

- [ ] 1.1 改 `src/figaroh/backend/casadi.py` 的 `_cache_dir()`：从 `figaroh.__file__` 沿目录树向上查找 `pyproject.toml` 定位工程根，缓存写到 `<project>/.cache/figaroh/casadi/`；找不到 `pyproject.toml` 时 fallback `~/.cache/figaroh/casadi/`。
- [ ] 1.2 `.gitignore` 新增 `.cache/` 条目。

## 2. 测试

- [ ] 2.1 加测试：工程环境缓存路径落在 `.cache/figaroh/casadi/`、非工程环境 fallback `~/.cache/figaroh/casadi/`、指纹 + `_CACHE_VERSION` 失效机制仍生效。

## 3. spec delta

- [ ] 3.1 写 `symbolic-casadi-pipeline` delta spec：MODIFIED "Symbolic model construction" scenario 的缓存路径 THEN 子句（工程根 `.cache/figaroh/casadi/` + XDG fallback），指纹 + 版本号失效语义不变。
```

