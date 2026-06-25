# Verification Report: migrate-to-pixi

- Date: 2026-06-25
- Mode: full
- Base ref: 793aff08702102f0cbc6e6c12c220da5ea6af55f

## Summary Scorecard

| Dimension | Status |
|-----------|--------|
| Completeness | **24/24 tasks**, 13 reqs defined |
| Correctness | **13/13 reqs covered**, 19 scenarios addressed |
| Coherence | Design decisions followed |

## Evidence

| Command | Result |
|---------|--------|
| `pixi run test` | **212 passed, 2 skipped, 0 failed** (2.05s) |
| `pixi run build` | **wheel + sdist built successfully** |
| `pixi install` (3 envs) | **default, docs, test all installed** |
| `pixi run lint` | flake8 + mypy ran (pre-existing warnings only) |
| `pixi run docs` | HTML generated at docs/build/html/ |
| `import figaroh` | **0.4.3 imported successfully** |

## Completeness

### Tasks: 24/24 ✅

All tasks marked `[x]` in `openspec/changes/migrate-to-pixi/tasks.md`. Task groups completed:
- 1. Research (2/2): All deps verified on 3 platforms via pixi.lock resolution
- 2. pixi.toml (7/7): Feature layers, environments, tasks all defined
- 3. pixi.lock (2/2): Generated with 3-platform entries
- 4. pyproject.toml (3/3): Dependencies removed, comment added
- 5. Cleanup (2/2): environment.yml deleted, .pixi/ in .gitignore
- 6. CI (2/2): docs.yml migrated to setup-pixi
- 7. Verification (6/6): All verification steps passed

### Specs: 2 capabilities, 13 requirements ✅

- `pixi-environment`: 5 requirements implemented
- `pixi-tasks`: 8 requirements implemented

## Correctness

### Requirement → Implementation Map

| Requirement | Evidence |
|-------------|---------|
| pixi.toml as single source | `pixi.toml` (71 lines, 7 files changed) |
| Multi-platform lock | `pixi.lock` contains linux-aarch64, linux-64, osx-arm64 |
| Multi-environment (default/docs/test) | 3 environments in pixi.toml `[environments]` |
| conda dependency (cyipopt) | `[feature.core.dependencies]` → conda-forge |
| environment.yml removed | `git rm environment.yml` commit fb1598e |
| 7 pixi tasks | `[tasks]` section: shell, test, lint, format, docs, build, clean |
| CI workflow | `.github/workflows/docs.yml` uses `prefix-dev/setup-pixi@v0` |
| pyproject.toml stripped | 0 occurrences of `dependencies` or `optional-dependencies` |

### Scenario Coverage: 19/19 scenarios addressed

All scenarios from both spec files are covered by implementation:
- pixi.toml exists + valid → TOML parse verified
- pyproject.toml stripped → verified
- Lock resolves for all platforms → verified
- Environment reproducible → pixi.lock ensures this
- Default/docs/test environments → all 3 installed
- All 7 tasks functional → test/build/docs verified
- CI integration → setup-pixi action in docs.yml

## Coherence

### Design Decisions Followed ✅

| Decision | Status |
|----------|--------|
| Feature composition (core/dev/docs/examples) | ✅ `[feature.*]` sections in pixi.toml |
| pyproject.toml coexistence | ✅ metadata preserved, deps removed |
| pin/rospkg via PyPI | ✅ `[pypi-dependencies]` |
| cyipopt via conda-forge | ✅ `channel = "conda-forge"` |
| python ==3.12.* | ✅ exact version pin |
| 3 environments | ✅ default, docs, test |
| 7 tasks | ✅ shell, test, lint, format, docs, build, clean |
| CI: setup-pixi action | ✅ prefix-dev/setup-pixi@v0 |

### Code Review ✅

Final review (standard mode): **APPROVED** — 3 Minor findings accepted with documented reasons.

## Issues

### CRITICAL: 0
### WARNING: 0
### SUGGESTION: 1

1. **SUGGESTION**: pixi 0.71+ deprecation warning recommends `[workspace]` instead of `[project]`. Non-blocking, cosmetic change for future pixi versions.

## Final Assessment

**All checks passed. Ready for archive.**
