#!/usr/bin/env python3
"""UR10 Backend Comparison Benchmark.

Compares NumericalBackend vs CasadiBackend for regressor build, gradient computation,
and full trajectory optimization, measuring time and numerical consistency.
"""
from __future__ import annotations

import time
import sys
import warnings
warnings.filterwarnings("ignore")

import numpy as np
from pathlib import Path

# Path setup
project_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(project_root / "figaroh-examples"))
sys.path.insert(0, str(project_root))

from figaroh.tools.robot import load_robot
from figaroh.tools.regressor import build_regressor_basic, RegressorConfig
from figaroh.backend import create_backend, NumericalBackend, CasadiBackend

# ── 1. Load UR10 Robot ──────────────────────────────────────────────
print("=" * 70)
print("UR10 Backend Comparison Benchmark")
print("=" * 70)

urdf = str(project_root / "figaroh-examples/models/ur_description/urdf/ur10_robot.urdf")
pkg_dir = str(project_root / "figaroh-examples/models")

print(f"\nLoading UR10 from: {urdf}")
robot = load_robot(urdf, package_dirs=pkg_dir, load_by_urdf=True)
nv = robot.model.nv
nq = robot.model.nq
print(f"DOF: nq={nq}, nv={nv}")

# ── 2. Generate Representative Trajectory Data ────────────────────────
print("\n--- Generating Test Data ---")
rng = np.random.default_rng(42)
N = 200  # samples

# Generate smooth-ish random trajectory (sine-based for realism)
t = np.linspace(0, 10.0, N)
q = np.zeros((N, nq))
v = np.zeros((N, nv))
a = np.zeros((N, nv))

for j in range(nq):
    amp = rng.uniform(0.5, 2.0)
    freq = rng.uniform(0.2, 1.5)
    phase = rng.uniform(0, 2 * np.pi)
    q[:, j] = amp * np.sin(freq * t + phase)
    v[:, j] = amp * freq * np.cos(freq * t + phase)
    a[:, j] = -amp * freq**2 * np.sin(freq * t + phase)

print(f"Trajectory: {N} samples, {nq} DOF")

# Build identif_config for the benchmark
identif_config = {
    'has_friction': True,
    'has_actuator_inertia': True,
    'has_joint_offset': False,
    'is_joint_torques': True,
    'is_external_wrench': False,
    'force_torque': None,
    'act_idxv': list(range(nv)),
    'act_idxq': list(range(nq)),
}

# ── 3. Benchmark Regressor Build ─────────────────────────────────────
print("\n" + "=" * 70)
print("REGRESSOR BUILD BENCHMARK")
print("=" * 70)

# 3a. Numerical Backend
num_backend = NumericalBackend(robot=robot)
t0 = time.perf_counter()
W_num = num_backend.build_regressor(q, v, a, identif_config)
t_num_reg = time.perf_counter() - t0
print(f"NumericalBackend:  {t_num_reg:.3f}s → shape {W_num.shape}")

# 3b. CasADi Backend
try:
    casadi_backend = CasadiBackend(robot=robot)
    t0 = time.perf_counter()
    W_cas = casadi_backend.build_regressor(q, v, a, identif_config)
    t_cas_reg = time.perf_counter() - t0
    print(f"CasadiBackend:      {t_cas_reg:.3f}s → shape {W_cas.shape}")

    # Consistency check
    if W_num.shape == W_cas.shape:
        diff = np.max(np.abs(W_num - W_cas))
        print(f"Max difference:     {diff:.2e}")
        speedup_reg = t_num_reg / t_cas_reg if t_cas_reg > 0 else float('inf')
        print(f"Regressor speedup:  {speedup_reg:.1f}x")
    else:
        print(f"WARNING: Shape mismatch! num={W_num.shape}, cas={W_cas.shape}")
        speedup_reg = t_num_reg / t_cas_reg if t_cas_reg > 0 else float('inf')

except ImportError as e:
    print(f"CasadiBackend unavailable: {e}")
    W_cas = None
    t_cas_reg = None

# ── 4. Benchmark Gradient Computation ────────────────────────────────
print("\n" + "=" * 70)
print("GRADIENT BENCHMARK")
print("=" * 70)

# Create a simple test objective (quadratic form of regressor)
def make_objective(W):
    """Return an objective function based on condition number."""
    def obj(x):
        return np.linalg.cond(W)
    return obj

n_vars = (nq - 1) if hasattr(robot.model, 'nq') else nq  # simplified
x0 = rng.normal(size=n_vars)

# 4a. Numerical gradient
obj_fn = make_objective(W_num)
t0 = time.perf_counter()
grad_num = num_backend.gradient(obj_fn, x0)
t_num_grad = time.perf_counter() - t0
print(f"NumericalBackend:  {t_num_grad:.3f}s → grad shape {grad_num.shape}")

# 4b. CasADi gradient
if W_cas is not None:
    try:
        obj_fn2 = make_objective(W_cas)
        t0 = time.perf_counter()
        grad_cas = casadi_backend.gradient(obj_fn2, x0)
        t_cas_grad = time.perf_counter() - t0
        print(f"CasadiBackend:      {t_cas_grad:.3f}s → grad shape {grad_cas.shape}")

        if grad_num.shape == grad_cas.shape:
            diff_grad = np.max(np.abs(grad_num - grad_cas))
            print(f"Max diff (grad):    {diff_grad:.2e}")
            speedup_grad = t_num_grad / t_cas_grad if t_cas_grad > 0 else float('inf')
            print(f"Gradient speedup:   {speedup_grad:.1f}x")
    except Exception as e:
        print(f"CasADi gradient error: {e}")

# ── 5. Regressor Vectorization Benchmark (varying N) ─────────────────
print("\n" + "=" * 70)
print("SCALING BENCHMARK (varying sample count)")
print("=" * 70)

sample_sizes = [50, 100, 200, 500]
print(f"{'N':>6}  {'Num (s)':>10}  {'Cas (s)':>10}  {'Speedup':>8}  {'MaxDiff':>10}")
print("-" * 52)

for n_samp in sample_sizes:
    q_s = q[:n_samp].copy()
    v_s = v[:n_samp].copy()
    a_s = a[:n_samp].copy()

    # Numerical
    t0 = time.perf_counter()
    W_n = num_backend.build_regressor(q_s, v_s, a_s, identif_config)
    t_n = time.perf_counter() - t0

    # CasADi
    if W_cas is not None:
        t0 = time.perf_counter()
        W_c = casadi_backend.build_regressor(q_s, v_s, a_s, identif_config)
        t_c = time.perf_counter() - t0
        diff_s = np.max(np.abs(W_n - W_c)) if W_n.shape == W_c.shape else float('nan')
        sp = t_n / t_c if t_c > 0 else float('inf')
        print(f"{n_samp:>6}  {t_n:>10.3f}  {t_c:>10.3f}  {sp:>7.1f}x  {diff_s:>10.2e}")
    else:
        print(f"{n_samp:>6}  {t_n:>10.3f}  {'N/A':>10}  {'N/A':>8}  {'N/A':>10}")

# ── 6. Summary ────────────────────────────────────────────────────────
print("\n" + "=" * 70)
print("SUMMARY")
print("=" * 70)
print(f"UR10 robot: {nq} DOF, {N} trajectory samples")
print(f"Regressor build — Numerical: {t_num_reg:.3f}s")
if t_cas_reg:
    print(f"Regressor build — CasADi:   {t_cas_reg:.3f}s ({speedup_reg:.1f}x speedup)")
    print(f"Regressor consistency: max |diff| = {diff:.2e}")
print(f"Gradient computation — Numerical: {t_num_grad:.3f}s")
if W_cas is not None:
    try:
        print(f"Gradient computation — CasADi:   {t_cas_grad:.3f}s ({speedup_grad:.1f}x speedup)")
    except:
        pass

print("\n✓ Benchmark complete")
