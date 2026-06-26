---
comet_change: integrate-examples-submodule
role: technical-design
canonical_spec: openspec
archived-with: 2026-06-26-integrate-examples-submodule
status: final
---

# Technical Design: Integrate figaroh-examples as Git Submodule

## Context

`figaroh-examples` is a standalone repository (315MB, including 111MB of URDF model files) that provides example scripts, robot models, and a web interface for FIGAROH. It was previously separated from the main `figaroh-plus` repository (18MB). The examples directory currently exists as an untracked copy in the working tree.

The main codebase already has multiple fallback methods in `load_robot.py` to locate the examples/models directory, including a relative path lookup that is compatible with a submodule at the repository root.

## Architecture

```
figaroh-plus (18MB)
├── .gitmodules                  ← NEW: submodule declaration
├── pyproject.toml               ← MODIFIED: deps + URL
├── pixi.lock                    ← REGENERATED
├── src/figaroh/                 (unchanged)
├── figaroh-examples/            ← git submodule (pointer, not content)
│   ├── models/                  (111MB, loaded by load_robot.py)
│   ├── examples/                (9MB, robot scripts)
│   └── web-interface/           (<1MB, viser-based UI)
│
│ Dependency chain:
│   figaroh (editable) → pin (pinocchio) → coal → hppfcl ✓
│   examples feature    → viser (NEW)
│                      → jupyter, notebook, ipywidgets (existing)
```

## Key Decisions

### D1: Dependency Chain — No Redundant Declarations

Verified dependency chain in pixi examples environment:

```
examples feature (pypi-dependencies):
  jupyter, notebook, ipywidgets  (existing)
  viser                          (NEW — currently missing)

Transitively provided (no declaration needed):
  pinocchio  ← pin (PyPI, v4.0.0, 3 platforms locked)
  hppfcl     ← coal v3.0.3 (transitive from pin)
  numpy, scipy, matplotlib, pandas, picos, meshcat, ... ← figaroh dependencies
```

Rationale: Examples feature only declares examples-specific packages. All core dependencies transit through `figaroh` editable install.

### D2: Submodule URL and Protocol

- URL: `https://github.com/book-seed/figaroh-examples.git`
- Branch: `main`
- Protocol: HTTPS (default). SSH users configure via `git config --global url."git@github.com:".insteadOf https://github.com/`

### D3: Submodule Change Workflow

```
1. Make changes inside figaroh-examples/
2. cd figaroh-examples && git commit && git push origin main
3. cd .. && git add figaroh-examples  (updates pointer)
4. git commit -m "chore: bump examples submodule"
```

### D4: Removed Dependencies

| Package | Reason |
|---------|--------|
| `build>=1.5.0,<2` (from project.dependencies) | Zero imports in source; build tool belongs in build-system |
| `cvxpy` (from UR10 README) | Zero imports across entire codebase; documentation drift |
| `environment.yml` (figaroh-examples) | pixi supersedes |
| `requirements.txt` (figaroh-examples) | pixi supersedes |

## Implementation Notes

### `load_robot.py` Compatibility

No changes needed. The fallback chain Method 3 resolves:
```
src/figaroh/tools/load_robot.py
  → ../../  (project root)
  → figaroh-examples/  (submodule)
  → models/  (URDF model packages)
```

### viser Integration

`viser` is used by 10 files in `web-interface/` for browser-based 3D visualization. It is a standalone package with no shared dependencies with pinocchio or figaroh. Adding it to the examples feature's pypi-dependencies is sufficient.

### pixi.lock Regeneration

After modifying `pyproject.toml` dependencies, `pixi update` regenerates the lock file for all 3 target platforms (linux-64, linux-aarch64, osx-arm64).

## Risks and Mitigations

| Risk | Mitigation |
|------|-----------|
| Users forget `--recurse-submodules` | README prominently documents this; error messages in load_robot.py already guide users |
| Submodule pointer becomes stale | Periodic `git submodule update --remote`; bump on examples releases |
| viser version conflicts in pixi solve | pixi solve-group ensures consistent resolution; if conflict arises, pin version |
| Deleted environment.yml breaks standalone users | Standalone users should clone examples repo directly; pixi is the recommended path for submodule users |

## Verification

1. `git clone --recurse-submodules` produces complete examples directory
2. `pixi run -e examples python -c "import viser, hppfcl, pinocchio"` succeeds
3. `load_robot._get_models_directory()` resolves to `figaroh-examples/models/`
4. `pixi run -e examples python examples/ur10/calibration.py` executes successfully
5. `pixi run test` — all 212 existing tests pass
