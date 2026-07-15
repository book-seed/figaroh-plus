## ADDED Requirements

### Requirement: Symbolic CasADi model from URDF

The system SHALL construct a fully symbolic robot model using `pinocchio.casadi.Model` from the robot's URDF description, along with a corresponding `cpin.Model.createData()` symbolic data structure.

#### Scenario: Symbolic model construction

- **WHEN** a robot with a valid URDF is loaded
- **THEN** `cpin.Model(robot.model)` SHALL produce a CasADi symbolic model
- **AND** `cmodel.createData()` SHALL allocate symbolic data structures
- **AND** both SHALL be cached to disk at `~/.figaroh/casadi_cache/` for subsequent reuse

### Requirement: Cache fingerprint covers full inertial parameters

The system SHALL compute a cache fingerprint that covers ALL inertial parameters and joint structure, using SHA256 hash of: model name, nq, nv, each body's mass (12 decimal places), center-of-mass lever arm coordinates (12dp), inertia tensor diagonal entries (12dp), and each joint's short-name, idx_q, and idx_v. Any modification to the URDF (mass, COM, inertia, joint type, or joint index) SHALL produce a different fingerprint, causing automatic cache invalidation and rebuild.

#### Scenario: URDF parameter change invalidates cache

- **WHEN** the URDF is modified (e.g., a link's mass or COM is changed)
- **THEN** the cache fingerprint SHALL differ from the previous value
- **AND** the cached symbolic functions SHALL be rebuilt from the updated model

#### Scenario: URDF unchanged reuses cache

- **WHEN** the URDF is identical to a previously cached version
- **THEN** the cached symbolic functions SHALL be loaded from disk without rebuild

### Requirement: SX/MX hybrid architecture

The system SHALL use a hybrid SX/MX architecture: inner-layer single-step dynamics functions (regressor and RNEA) SHALL be built and cached as CasADi SX Functions; outer-layer trajectory optimization SHALL use CasADi MX symbols for Fourier coefficients and construct the NLP graph by calling inner SX Functions via `cs.Function.map(N_s, "openmp")`. Gradient computation SHALL use MX reverse-mode automatic differentiation for efficient many-input-to-few-output Jacobians.

#### Scenario: SX inner functions are cached

- **WHEN** `CasadiBackend` constructs symbolic dynamics functions
- **THEN** `regressor_function` SHALL be an SX `cs.Function(q,v,a) → W(nv, n_param)` cached to disk
- **AND** `rnea_function` SHALL be an SX `cs.Function(q,v,a) → τ(nv)` cached to disk
- **AND** both SHALL be accessible via `CasadiBackend` property interfaces

#### Scenario: MX outer layer calls SX via map

- **WHEN** the MX optimization graph is constructed from Fourier coefficient variables
- **THEN** the inner SX Functions SHALL be invoked via `.map(N_s, "openmp")` with column-major MX inputs
- **AND** all trajectory variables (Q, V, A) SHALL be built directly as `(n_act, N_s)` column-major MX expressions to eliminate Transpose nodes

### Requirement: Symbolic regressor matrix construction

The system SHALL construct the full joint torque regressor matrix `W` symbolically via `cpin.computeJointTorqueRegressor(cmodel, cdata, cs_q, cs_v, cs_a)`.

The system SHALL build the base regressor `W_b` by selecting columns from W using the pre-computed base parameter indices `idx_b` (from `BaseParameterComputer`).

The system SHALL append additional parameter columns (friction fv/fs, actuator inertia Ia, joint offset) to the symbolic regressor when enabled in `identif_config`, before base-parameter column selection.

#### Scenario: Regressor with additional parameters

- **WHEN** `identif_config.has_friction: true`
- **THEN** the symbolic regressor SHALL include viscous friction columns `fv · v_j` and static friction columns `fs · tanh(α·v_j)` for each active joint
- **AND** the column position SHALL match the numerical backend's column order

### Requirement: Build-order: symbolic function first, then map

The system SHALL construct the regressor as a CasADi SX symbolic function `W_func(cs_q, cs_v, cs_a)` FIRST, compute its Jacobian via CasADi AD, and THEN apply `cs.Function.map(N_s)` to evaluate over sample points in parallel.

#### Scenario: Jacobian in AD graph

- **WHEN** the constraint Jacobian or regressor Jacobian is required
- **THEN** it SHALL be computed by `cs.jacobian()` on the symbolic expression graph BEFORE `map()` is applied
- **AND** the resulting Jacobian SHALL match finite-difference verification to within `1e-5` relative tolerance

### Requirement: D-optimal objective function with Cholesky factorization

The system SHALL minimize the D-optimality criterion using Cholesky factorization:

```
J = W_b(x)ᵀ · W_b(x) / N_s + λ·I    (SPD guaranteed by λ > 0)
L = cholesky(J)
f(x) = -2 · Σ_i log(L_ii)
```

where `λ` defaults to `1e-6` and SHALL be configurable via `reg_lambda`. The regularization term `λ·I` guarantees `J` is strictly positive definite, ensuring Cholesky factorization always succeeds without triggering IPOPT restoration phase failures.

#### Scenario: Objective computed via Cholesky

- **WHEN** the base regressor `W_b` is constructed from Fourier coefficients `x`
- **THEN** the objective SHALL be computed as `-2 * sum(log(diag(L)))` where `L = cholesky(W_bᵀ·W_b / N_s + λ·I)`
- **AND** the computation SHALL be entirely within the CasADi MX graph

#### Scenario: Regularization prevents singularity

- **WHEN** `W_bᵀ·W_b` is near-singular (e.g., initial iteration with small-amplitude coefficients)
- **THEN** the regularization term `λ·I` SHALL guarantee the argument to Cholesky is strictly positive definite
- **AND** the log-determinant SHALL remain finite and numerically stable

### Requirement: IPOPT solver with HSL acceleration and OpenMP

The system SHALL use CasADi's built-in `cs.nlpsol("ipopt", nlp)` interface with HSL linear solver (`ma57`) preferred, falling back to `mumps` when HSL is unavailable.

The system SHALL use `cs.Function.map(N_s, "openmp")` for parallel evaluation of dynamics functions. OpenMP SHALL be a hard requirement — the environment validation script SHALL verify OpenMP availability and block execution with a remediation message if unavailable. No silent fallback to serial execution is permitted.

#### Scenario: HSL available

- **WHEN** the HSL library (`libhsl.so` or `libcoinhsl.so`) is found in the library path
- **THEN** IPOPT SHALL use `linear_solver: ma57`

#### Scenario: HSL unavailable

- **WHEN** no HSL library is detected
- **THEN** IPOPT SHALL fall back to `linear_solver: mumps` with a logged warning

#### Scenario: OpenMP unavailable

- **WHEN** CasADi `map("openmp")` raises an error
- **THEN** the environment validation script SHALL report the failure with remediation instructions
- **AND** the optimization SHALL NOT proceed

### Requirement: Full symbolic constraints including joint friction

The system SHALL build position, velocity, and torque constraints as CasADi MX expressions. Joint torque constraints SHALL be the sum of rigid-body dynamics and joint friction:

```
τ_total = cpin.rnea(q, v, a) + fv·v + fs·tanh(α_opt·v)
```

where `fv` and `fs` are nominal friction coefficients from `params_std`, and `α_opt` defaults to 10 (configurable via `tanh_alpha_opt`).

#### Scenario: Torque constraint includes friction

- **WHEN** torque constraints are evaluated for a Fourier trajectory
- **THEN** the constraint value SHALL be `τ_rnea + τ_friction`
- **AND** friction SHALL use `tanh(α_opt·v)` with configurable `α_opt` (default 10) for gradient-friendly optimization

#### Scenario: Symbolic constraint evaluation

- **WHEN** Fourier coefficients produce a trajectory via symbolic MX expressions
- **THEN** position constraints `q_lower ≤ q(t) ≤ q_upper` SHALL be constructed as MX inequality expressions
- **AND** velocity constraints `v_lower ≤ v(t) ≤ v_upper` SHALL be constructed as MX inequality expressions
- **AND** torque constraints `τ_lower ≤ RNEA(q,v,a) + friction ≤ τ_upper` SHALL be computed within the CasADi MX graph

### Requirement: CasADi single source of truth

The system SHALL expose pre-built symbolic dynamics functions via `CasadiBackend` property interfaces (`regressor_function` and `rnea_function`), providing a single source of truth for both trajectory optimization and parameter identification. External code SHALL NOT directly access private attributes of `CasadiBackend`.

#### Scenario: Fourier strategy accesses dynamics via property

- **WHEN** `FourierOptimizationStrategy` requires the symbolic regressor or RNEA function
- **THEN** it SHALL obtain them via `context._backend.regressor_function` and `context._backend.rnea_function` properties
- **AND** SHALL NOT access `_W_fun` or other private attributes directly
