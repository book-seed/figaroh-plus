# Proposal: CasADi Optional Backend

## Why
FIGAROH's optimal trajectory generation is slow due to IPOPT numerical differentiation.
CasADi provides analytical AD via pinocchio.casadi, enabling 10-20x speedup.

## What Changes
- New `figaroh.backend` module with Backend ABC, NumericalBackend, CasadiBackend
- Backend selection via YAML config + programmatic parameter
- CasADi declared as optional dependency

## Capabilities
- `casadi-backend`: CasADi-based computation backend

## Impact
No breaking API changes. Backend defaults to `numerical` preserving current behavior.
