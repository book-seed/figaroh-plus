# ADR: CasADi Optional Backend

## Status
Accepted (2026-07-02)

## Context
FIGAROH's optimal trajectory generation is bottlenecked by IPOPT's gradient
computation, requiring N+1 evaluations of objective_function() per iteration.
Each evaluation runs a serial Python loop over hundreds of samples calling
pin.computeJointTorqueRegressor(). For a 6-DOF robot with 500 samples and 200
IPOPT iterations, this amounts to ~10,000 regressor builds.

## Decision
Introduce a strategy-pattern backend abstraction that lets users choose between:
1. **NumericalBackend** (default): wraps existing code unchanged
2. **CasadiBackend**: uses pinocchio.casadi for symbolic regressor and
   cs.nlpsol('ipopt', nlp) for analytical-derivative IPOPT

## Key Details
- Backend ABC defines: build_regressor(), gradient(), jacobian(), create_solver()
- Solver-level abstraction (not derivative-level) — each backend uses its
  natural IPOPT interface
- Lazy symbolic model construction — users selecting numerical backend pay
  zero overhead
- CasADi is an optional dependency via pip install figaroh[casadi] or
  pixi feature

## Consequences
- Zero regression: numerical backend preserves all existing behavior
- 5-20x speedup when using casadi backend for trajectory optimization
- Two-code-path maintenance burden mitigated by frozen numerical path
