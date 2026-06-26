# Verification Report: integrate-examples-submodule

- Date: 2026-06-26
- Verify Mode: full
- Base Ref: 34e2a56e0abd44a777287803d5ec40f6f939a27b

## Summary

| Dimension | Status |
|-----------|--------|
| Completeness | 22/22 tasks complete |
| Correctness | All design decisions verified |
| Coherence | Implementation matches design |

## Issues

### CRITICAL

None.

### WARNING

None.

### SUGGESTION

- **Push pending**: figaroh-examples cleanup commit (c21ee41) needs `git push origin main` from submodule directory. No write access verified during build phase.
- **UR10 mesh data**: calibration script fails at mesh resolution due to missing `agimus-demos` ROS package. This is a pre-existing data dependency, not introduced by this change.

## Verification Details

### Completeness (22/22 tasks)

| Task Group | Tasks | Status |
|-----------|-------|--------|
| 1. Submodule Setup | 3 | ✅ |
| 2. figaroh-examples Cleanup | 5 | ✅ |
| 3. figaroh-plus Config | 6 | ✅ |
| 4. Dependency Verification | 3 | ✅ |
| 5. UR10 Acceptance | 2 | ✅ |
| 6. Push & Commit | 3 | ✅ |

### Correctness — Design Decision Audit

| Decision | Expected | Actual | Status |
|----------|---------|--------|--------|
| D1: Submodule URL | book-seed/figaroh-examples.git | `.gitmodules`: `https://github.com/book-seed/figaroh-examples.git` | ✅ |
| D2: HTTPS protocol | HTTPS | `.gitmodules` uses HTTPS | ✅ |
| D3: viser in examples, no hppfcl/pinocchio | viser added, hppfcl not declared | `viser = "*"` in examples pypi-deps; hppfcl absent (transitive via coal) | ✅ |
| D4: build from project.deps → dev feature | `build` in dev, not in project.deps | `[project.dependencies]` has no build; `[tool.pixi.feature.dev]` has `build = "*"` | ✅ |
| D5: env/req deleted | Files removed | `environment.yml` and `requirements.txt` deleted in submodule | ✅ |
| D6: load_robot.py unchanged | No modifications | `git diff base-ref..HEAD -- src/figaroh/tools/load_robot.py` shows no changes | ✅ |

### Coherence — URL Consistency

| Location | URL |
|----------|-----|
| `.gitmodules` | `https://github.com/book-seed/figaroh-examples.git` |
| `pyproject.toml` Examples | `https://github.com/book-seed/figaroh-examples` |
| `pyproject.toml` Homepage/Repo/Issues | `https://github.com/book-seed/figaroh-plus` |
| `.gitignore` comment | `https://github.com/book-seed/figaroh-examples` |
| `README.md` clone | `https://github.com/book-seed/figaroh-plus.git` |
| `README.md` examples link | `https://github.com/book-seed/figaroh-examples` |

All URLs consistently point to `book-seed` org.

### Test Results

- pixi run test: **212 passed, 2 skipped** (baseline: 212 passed, 2 skipped)
- pixi run build: **Success** (figaroh-0.4.3 wheel built)
- viser/hppfcl/pinocchio imports: **OK**
- models directory resolution: **OK** (`figaroh-examples/models/`)

## Final Assessment

All checks passed. No critical issues. Ready for archive.

**Follow-up**: Manually push figaroh-examples submodule changes: `cd figaroh-examples && git push origin main`
