---
change: relocate-casadi-cache-to-pixi-env
design-doc: openspec/changes/relocate-casadi-cache-to-pixi-env/design.md
base-ref: 1984006feb753a1a8d1c4d6459808f99dfcb3209
---

# CasADi 缓存路径迁移至工程 .cache 目录 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 CasADi 符号模型缓存从家目录 `~/.figaroh/casadi_cache/` 迁移到工程根目录 `.cache/figaroh/casadi/`，非工程环境 fallback 至 XDG 兼容路径 `~/.cache/figaroh/casadi/`。

**Architecture:** 修改 `CasadiBackend._cache_dir()` 静态方法，从 `figaroh.__file__` 沿目录树向上查找 `pyproject.toml` 定位工程根目录。找到则缓存至 `<project>/.cache/figaroh/casadi/`；找不到则 fallback 至 `~/.cache/figaroh/casadi/`。同时在 `.gitignore` 新增 `.cache/` 条目。缓存指纹和 `_CACHE_VERSION` 失效机制保持不变。

**Tech Stack:** Python 3, CasADi (via lazy import), pytest + unittest.mock

## Global Constraints

- 不改指纹 + `_CACHE_VERSION` 失效机制（`_cache_key` 和 `_CACHE_VERSION` 不变）
- 不改 spec 缓存语义（仍"缓存到磁盘、按指纹失效"），只改路径定位
- 工程根定位不依赖 git 二进制，纯 Python 实现
- 非工程环境 fallback 路径必须可用

---

## 文件结构

| 文件 | 操作 | 职责 |
|------|------|------|
| `src/figaroh/backend/casadi.py:141-146` | 修改 | `_cache_dir()` 静态方法 — 当前硬编码 `~/.figaroh/casadi_cache/`，需改为工程感知路径 |
| `.gitignore` | 修改 | 新增 `.cache/` 条目，防止缓存目录进入版本控制 |
| `tests/unit/test_backend.py` | 修改 | 新增 `TestCacheDir` 测试类，覆盖工程路径和 fallback 路径 |
| `openspec/changes/relocate-casadi-cache-to-pixi-env/specs/symbolic-casadi-pipeline/spec.md` | 新建 | Delta spec：MODIFIED "Symbolic model construction" scenario 的缓存路径 THEN 子句 |

---

### Task 1: 修改 `_cache_dir()` 实现工程感知缓存路径

**Files:**
- Modify: `src/figaroh/backend/casadi.py:141-146`
- Modify: `tests/unit/test_backend.py`（追加测试类）

**Interfaces:**
- Consumes: `figaroh.__file__`（模块文件路径，用于向上查找 `pyproject.toml`）
- Produces: `CasadiBackend._cache_dir() -> str`（返回类型不变，返回值变为工程感知路径）

- [x] **Step 1: 编写测试 — 工程环境缓存路径落在 `.cache/figaroh/casadi/`**

在 `tests/unit/test_backend.py` 末尾追加以下测试类：

```python
class TestCacheDir:
    """Test CasadiBackend._cache_dir() cache path resolution."""

    def test_cache_dir_in_project_env(self, tmp_path):
        """When pyproject.toml is found, cache goes under .cache/figaroh/casadi/."""
        from figaroh.backend.casadi import CasadiBackend
        import figaroh

        # Create a fake pyproject.toml in tmp_path
        pyproject = tmp_path / "pyproject.toml"
        pyproject.write_text("[project]\nname = 'test'")

        # Simulate figaroh.__file__ being inside the project
        fake_init = tmp_path / "src" / "figaroh" / "__init__.py"
        fake_init.parent.mkdir(parents=True)
        fake_init.write_text("")

        with patch.object(figaroh, "__file__", str(fake_init)):
            cache_dir = CasadiBackend._cache_dir()

        expected = str(tmp_path / ".cache" / "figaroh" / "casadi")
        assert cache_dir == expected
        assert os.path.isdir(cache_dir)

    def test_cache_dir_fallback_no_pyproject_toml(self, tmp_path):
        """When no pyproject.toml found, fallback to ~/.cache/figaroh/casadi/."""
        from figaroh.backend.casadi import CasadiBackend
        import figaroh

        # tmp_path has no pyproject.toml
        fake_init = tmp_path / "isolated" / "figaroh" / "__init__.py"
        fake_init.parent.mkdir(parents=True)
        fake_init.write_text("")

        with patch.object(figaroh, "__file__", str(fake_init)):
            cache_dir = CasadiBackend._cache_dir()

        expected = os.path.join(os.path.expanduser("~"), ".cache", "figaroh", "casadi")
        assert cache_dir == expected

    def test_cache_version_still_in_cache_key(self):
        """_CACHE_VERSION is unchanged and still part of cache filename logic."""
        from figaroh.backend.casadi import CasadiBackend
        assert CasadiBackend._CACHE_VERSION == "v2"
```

需要在文件顶部已有 import 之后追加 `import os`（检查是否已存在；`test_backend.py` 未 import `os`，需追加）。

需要追加的 import 行（在现有 import 区域末尾）：

```python
import os
```

- [x] **Step 2: 运行测试验证失败（_cache_dir 尚未实现新逻辑）**

```bash
cd /home/tyche/Documents/figaroh-plus && python -m pytest tests/unit/test_backend.py::TestCacheDir -v
```

**预期:** `test_cache_dir_in_project_env` 和 `test_cache_dir_fallback_no_pyproject_toml` FAIL — 当前 `_cache_dir()` 仍返回 `~/.figaroh/casadi_cache/`

- [x] **Step 3: 实现 `_cache_dir()` 工程感知逻辑**

修改 `src/figaroh/backend/casadi.py` 第 141-146 行，将：

```python
    @staticmethod
    def _cache_dir() -> str:
        """Return the cache directory, creating it if necessary."""
        d = os.path.join(os.path.expanduser("~"), ".figaroh", "casadi_cache")
        os.makedirs(d, exist_ok=True)
        return d
```

替换为：

```python
    @staticmethod
    def _cache_dir() -> str:
        """Return the cache directory, creating it if necessary.

        Resolution order:
        1. If running inside a pixi project (pyproject.toml found by walking
           up from ``figaroh.__file__``), cache goes to
           ``<project>/.cache/figaroh/casadi/``.
        2. Otherwise fallback to XDG-compatible ``~/.cache/figaroh/casadi/``.
        """
        import figaroh

        # Walk up from figaroh's location looking for pyproject.toml
        current = os.path.dirname(os.path.abspath(figaroh.__file__))
        while True:
            if os.path.isfile(os.path.join(current, "pyproject.toml")):
                # Project root found — cache inside project
                d = os.path.join(current, ".cache", "figaroh", "casadi")
                break
            parent = os.path.dirname(current)
            if parent == current:
                # Filesystem root reached — no project, use XDG fallback
                d = os.path.join(
                    os.path.expanduser("~"), ".cache", "figaroh", "casadi"
                )
                break
            current = parent

        os.makedirs(d, exist_ok=True)
        return d
```

- [x] **Step 4: 运行测试验证通过**

```bash
cd /home/tyche/Documents/figaroh-plus && python -m pytest tests/unit/test_backend.py::TestCacheDir -v
```

**预期:** 全部 3 个测试 PASS

- [x] **Step 5: 确保已有测试不受影响**

```bash
cd /home/tyche/Documents/figaroh-plus && python -m pytest tests/unit/test_backend.py -v
```

**预期:** 全部已有测试 + 新增测试均 PASS

- [x] **Step 6: Commit**

```bash
git add src/figaroh/backend/casadi.py tests/unit/test_backend.py
git commit -m "feat: relocate casadi cache to project .cache/ with XDG fallback

_cache_dir() now walks up from figaroh.__file__ to locate pyproject.toml.
When found, cache goes to <project>/.cache/figaroh/casadi/; otherwise
falls back to ~/.cache/figaroh/casadi/. Fingerprint and _CACHE_VERSION
mechanisms unchanged.

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

### Task 2: 在 .gitignore 中忽略 `.cache/`

**Files:**
- Modify: `.gitignore`

**Interfaces:**
- 无代码接口依赖

- [x] **Step 1: 在 .gitignore 末尾追加 `.cache/` 条目**

修改 `.gitignore`，在文件末尾追加一行：

```
# CasADi symbolic model cache (project-local, see relocate-casadi-cache-to-pixi-env)
.cache/
```

具体编辑：在 `.gitignore` 最后一行之后新增两行。

当前 `.gitignore` 末尾为：

```
# pixi environment directory
.pixi/
```

在其后追加：

```
# pixi environment directory
.pixi/

# CasADi symbolic model cache (project-local)
.cache/
```

- [x] **Step 2: 验证 gitignore 生效**

```bash
cd /home/tyche/Documents/figaroh-plus && mkdir -p .cache/figaroh/casadi && touch .cache/figaroh/casadi/test.casadi && git status --short .cache/ 2>&1
```

**预期:** 无输出（`.cache/` 被 gitignore，git 不追踪）

- [x] **Step 3: Commit**

```bash
git add .gitignore
git commit -m "chore: add .cache/ to gitignore for project-local casadi cache

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

### Task 3: 编写 delta spec

**Files:**
- Create: `openspec/changes/relocate-casadi-cache-to-pixi-env/specs/symbolic-casadi-pipeline/spec.md`

**Interfaces:**
- 无代码接口依赖 — 纯文档变更

- [x] **Step 1: 创建 delta spec 文件**

创建目录并写入文件：

```bash
mkdir -p /home/tyche/Documents/figaroh-plus/openspec/changes/relocate-casadi-cache-to-pixi-env/specs/symbolic-casadi-pipeline
```

文件内容（`openspec/changes/relocate-casadi-cache-to-pixi-env/specs/symbolic-casadi-pipeline/spec.md`）：

```markdown
# symbolic-casadi-pipeline — Delta Spec

## MODIFIED Requirement: Symbolic CasADi model from URDF

### MODIFIED Scenario: Symbolic model construction

- **WHEN** a robot with a valid URDF is loaded
- **THEN** `cpin.Model(robot.model)` SHALL produce a CasADi symbolic model
- **AND** `cmodel.createData()` SHALL allocate symbolic data structures
- **AND** both SHALL be cached to disk: under `<project>/.cache/figaroh/casadi/` when running inside a pixi project (pyproject.toml found by walking up from `figaroh.__file__`), falling back to `~/.cache/figaroh/casadi/` (XDG-compatible) when no project root is found
- **AND** the cache fingerprint and `_CACHE_VERSION` invalidation mechanism SHALL remain unchanged

### Rationale

Current spec (line 15 of main spec) reads:

> `~/.figaroh/casadi_cache/`

This is replaced with the project-local + XDG fallback paths described above. The
cache fingerprint (SHA256 over full inertial parameters + joint structure) and
`_CACHE_VERSION` tag behavior are preserved — only the base directory resolution
changes.
```

- [x] **Step 2: Commit**

```bash
git add openspec/changes/relocate-casadi-cache-to-pixi-env/specs/
git commit -m "spec: delta spec for casadi cache path relocation

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

## 自审清单

**1. Spec 覆盖：**
- "Symbolic model construction" 的缓存路径 THEN 子句 → Task 3 delta spec MODIFIED
- 工程根 `.cache/figaroh/casadi/` + XDG fallback → Task 1 实现 + 测试覆盖
- `.gitignore` 新增 `.cache/` → Task 2
- 指纹 + 版本号失效语义不变 → Task 1.3 测试 `test_cache_version_still_in_cache_key` 覆盖

**2. 占位符扫描：** 无 TBD/TODO/placeholder — 每个步骤都有完整代码和命令。

**3. 类型一致性：**
- `_cache_dir() -> str` — Task 1 实现和测试一致
- `_CACHE_VERSION = "v2"` — Task 1.3 测试验证不变
- `_cache_key(robot) -> str` — 引用 task 中接口标注一致（Task 1 不修改此方法）
