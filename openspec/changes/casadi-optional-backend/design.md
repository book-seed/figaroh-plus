# Design: CasADi Optional Backend

## Context
FIGAROH's computation pipeline currently has a single numerical code path.
CasADi integration via pinocchio.casadi enables analytical AD.

## Goals
1. Backend abstraction dispatching to numerical or casadi
2. CasADi path uses pinocchio.casadi + cs.nlpsol
3. Backend selection via config with numerical as default
4. CasADi is optional dependency

## Decisions
- D1: Solver-level abstraction
- D2: Lazy symbolic model construction
- D3: Symbolic NLP + cs.nlpsol
- D4: Dual-channel backend selection
