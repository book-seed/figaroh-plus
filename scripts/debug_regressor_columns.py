#!/usr/bin/env python3
"""Diagnose CasADi vs Python Pinocchio regressor column ordering.

Compares the base regressor W_b built via the CasADi symbolic path
(cpin.computeJointTorqueRegressor) against the Python numerical path
(build_regressor_basic -> reduced -> base) to determine whether column
ordering differences are causing NLP obj != diagnostic d_opt.

Usage:
    pixi run -e casadi python scripts/debug_regressor_columns.py
"""

import numpy as np

# ---- 1. Build a 2-DOF planar robot (programmatic, no URDF) ----------
# Joint 0: revolute around Z, link along X
# Joint 1: revolute around Z, link along X

import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper

model = pin.Model()
model.name = "planar_2dof"
jt0_id = model.addJoint(0, pin.JointModelRZ(), pin.SE3.Identity(), "joint_0")
body0_inertia = pin.Inertia.FromCylinder(1.0, 0.05, 0.5)
body0_placement = pin.SE3(np.eye(3), np.array([0.25, 0.0, 0.0]))
model.appendBodyToJoint(jt0_id, body0_inertia, body0_placement)
jt1_id = model.addJoint(1, pin.JointModelRZ(), body0_placement, "joint_1")
body1_inertia = pin.Inertia.FromCylinder(0.5, 0.03, 0.3)
body1_placement = pin.SE3(np.eye(3), np.array([0.15, 0.0, 0.0]))
model.appendBodyToJoint(jt1_id, body1_inertia, body1_placement)
model.upperPositionLimit = np.array([np.pi, np.pi])
model.lowerPositionLimit = np.array([-np.pi, -np.pi])
model.velocityLimit = np.array([10.0, 10.0])
model.effortLimit = np.array([100.0, 100.0])
robot = RobotWrapper(model)
robot.data = model.createData()
robot.q0 = np.zeros(model.nq)
robot.v0 = np.zeros(model.nv)
nv, nq = model.nv, model.nq
print(f"Robot: {model.name}, nq={nq}, nv={nv}")

# ---- 2. identif_config (no friction/inertia/offset) -----------------
identif_config = {
    "active_joints": ["joint_0", "joint_1"],
    "act_idxq": [0, 1],
    "act_idxv": [0, 1],
    "has_friction": False,
    "has_actuator_inertia": False,
    "has_joint_offset": False,
}

# ---- 3. Joint limits for random sampling ---------------------------
q_lim = 0.9 * np.minimum(np.abs(model.upperPositionLimit), np.abs(model.lowerPositionLimit))
v_lim = 0.5 * model.velocityLimit

# ---- 4. Compute idx_e / idx_b from random trajectory ----------------
from figaroh.tools.regressor import (
    build_regressor_basic, build_regressor_reduced, get_index_eliminate,
)
from figaroh.tools.qrdecomposition import get_baseIndex, build_baseRegressor
from figaroh.identification.identification_tools import get_standard_parameters

np.random.seed(99)
q_base = np.random.uniform(-q_lim, q_lim, (50, nq))
v_base = np.random.uniform(-v_lim, v_lim, (50, nv))
a_base = np.random.uniform(-v_lim, v_lim, (50, nv))
W_base = build_regressor_basic(robot, q_base, v_base, a_base, identif_config)
params_std = get_standard_parameters(robot.model, identif_config)
idx_e_raw, params_r = get_index_eliminate(W_base, params_std, tol_e=0.001)
idx_e = np.array(idx_e_raw, dtype=int)
W_e_base = build_regressor_reduced(W_base, idx_e)
idx_b_raw = get_baseIndex(W_e_base, params_r)
idx_b = np.array(idx_b_raw, dtype=int)
print(f"idx_e ({len(idx_e)}): {idx_e}")
print(f"idx_b ({len(idx_b)}): {idx_b}")

# ---- 5. Generate random trajectory samples --------------------------
np.random.seed(42)
N_samples = 200
q_rand = np.random.uniform(-q_lim, q_lim, (N_samples, nq))
v_rand = np.random.uniform(-v_lim, v_lim, (N_samples, nv))
a_rand = np.random.uniform(-v_lim, v_lim, (N_samples, nv))

# ---- 6. Python path: build_regressor_basic -> reduced -> base -------
W_py = build_regressor_basic(robot, q_rand, v_rand, a_rand, identif_config)
print(f"W_py (full) shape: {W_py.shape}")
W_e_py = build_regressor_reduced(W_py, idx_e)
W_b_py = build_baseRegressor(W_e_py, idx_b)
print(f"W_b_py        shape: {W_b_py.shape}")

# ---- 7. CasADi path: cpin.computeJointTorqueRegressor -> stack ------
import casadi as cs
import pinocchio.casadi as cpin

cmodel = cpin.Model(robot.model)
cdata = cmodel.createData()
cs_q = cs.SX.sym("q", nq)
cs_v = cs.SX.sym("v", nv)
cs_a = cs.SX.sym("a", nv)
W_expr = cpin.computeJointTorqueRegressor(cmodel, cdata, cs_q, cs_v, cs_a)
W_fun = cs.Function("W", [cs_q, cs_v, cs_a], [W_expr])
n_param = W_fun.size_out(0)[1]
print(f"CasADi n_param_total: {n_param}")

W_blocks_cs = []
for i in range(N_samples):
    W_i = np.array(W_fun(q_rand[i], v_rand[i], a_rand[i]))
    W_blocks_cs.append(W_i)
W_full_cs = np.vstack(W_blocks_cs)
print(f"W_cs (full)   shape: {W_full_cs.shape}")

all_cols = list(range(W_full_cs.shape[1]))
keep_cols = [i for i in all_cols if i not in idx_e]
W_e_cs = W_full_cs[:, keep_cols]
W_b_cs = W_e_cs[:, list(idx_b)]
print(f"W_b_cs        shape: {W_b_cs.shape}")

# ---- 8. FIM comparison (row-ordering independent) -------------------
print()
print("=" * 70)
print("Fisher Information Matrix: W_b^T @ W_b / N_samples")
FIM_cs = (W_b_cs.T @ W_b_cs) / N_samples
FIM_py = (W_b_py.T @ W_b_py) / N_samples
fim_diff = float(np.max(np.abs(FIM_cs - FIM_py)))
fim_rel = fim_diff / max(float(np.max(np.abs(FIM_cs))), 1e-12)
print(f"  FIM max abs diff:  {fim_diff:.4e}")
print(f"  FIM max rel diff:  {fim_rel:.4e}")
print(f"  cond(FIM_cs):      {float(np.linalg.cond(FIM_cs)):.2f}")
print(f"  cond(FIM_py):      {float(np.linalg.cond(FIM_py)):.2f}")

reg = 1e-6
sign_cs, ld_cs = np.linalg.slogdet(FIM_cs + reg * np.eye(FIM_cs.shape[0]))
sign_py, ld_py = np.linalg.slogdet(FIM_py + reg * np.eye(FIM_py.shape[0]))
dopt_cs = float(-ld_cs) if sign_cs > 0 else float("inf")
dopt_py = float(-ld_py) if sign_py > 0 else float("inf")
print(f"  d_opt (CasADi):    {dopt_cs:.6f}")
print(f"  d_opt (Python):    {dopt_py:.6f}")
print(f"  d_opt diff:        {abs(dopt_cs - dopt_py):.6e}")

if fim_diff < 1e-8:
    print()
    print(">>> FIMs are IDENTICAL.")
    print(">>> obj != d_opt must have a different root cause.")
else:
    print()
    print(">>> FIMs DIFFER -- this explains obj != d_opt.")
    # Show which FIM entries differ
    diff_mask = np.abs(FIM_cs - FIM_py) > 1e-8
    n_diff = int(np.sum(diff_mask))
    print(f">>> {n_diff} / {FIM_cs.size} entries differ")
    if n_diff > 0 and n_diff < 30:
        rows, cols = np.where(diff_mask)
        for r, c in zip(rows, cols):
            print(f"    [{r},{c}]: cs={FIM_cs[r,c]:.6e}  py={FIM_py[r,c]:.6e}")

print("=" * 70)

# ---- 9. Also compare W_full FIM (before idx_e/b reduction) ----------
print()
print("FIM from W_full (before column reduction):")
FIM_full_cs = (W_full_cs.T @ W_full_cs) / N_samples
FIM_full_py = (W_py.T @ W_py) / N_samples
full_fim_diff = float(np.max(np.abs(FIM_full_cs - FIM_full_py)))
print(f"  W_full FIM diff: {full_fim_diff:.4e}")
if full_fim_diff < 1e-8:
    print("  >>> W_full FIMs MATCH -- idx_e/b reduction is not the issue.")
else:
    print("  >>> W_full FIMs DIFFER -- CasADi & Python regressors diverge at source.")
print("=" * 70)

# ---- 10. Compare CasADi MX vs NumPy Fourier trajectory --------------
from figaroh.utils.fourier_trajectory import FourierTrajectory

n_h = 5
n_act = 2
omega_val = 2.0 * np.pi / 2.0  # T=2s -> omega=pi
T_val = 2.0
N_s = 50

ft = FourierTrajectory(n_harmonics=n_h, n_act=n_act, omega=omega_val)

# Random Fourier coefficients
np.random.seed(123)
n_coeffs = 1 + 2 * n_h  # 11
coeffs_np = np.random.randn(n_act, n_coeffs) * 0.5

# ---- 10a. NumPy evaluation ----
t_np = np.linspace(0, T_val, N_s)
q_np = ft.get_trajectory(t_np, coeffs_np)
v_np = ft.get_velocity(t_np, coeffs_np)
a_np = ft.get_acceleration(t_np, coeffs_np)

# ---- 10b. CasADi MX evaluation ----
t_np_vals = np.linspace(0, T_val, N_s)
t_mx = cs.DM(t_np_vals).T  # (1, Ns) as DM

coeffs_mx = cs.MX.sym("Z", n_act, n_coeffs)
Q_mx, V_mx, A_mx = ft.build_mx_trajectory(t_mx, coeffs_mx)
traj_fn = cs.Function("traj", [coeffs_mx], [Q_mx, V_mx, A_mx])

result_q, result_v, result_a = traj_fn(coeffs_np)
q_cs = np.array(result_q).T  # (N_s, n_act) transposed from (n_act, N_s)
v_cs = np.array(result_v).T
a_cs = np.array(result_a).T

# ---- 10c. Compare ----
print()
print("=" * 70)
print("CasADi MX vs NumPy Fourier Trajectory Comparison")
print(f"  n_harmonics={n_h}, n_act={n_act}, Ns={N_s}, T={T_val:.1f}s, omega={omega_val:.4f}")
print("-" * 70)

for name, np_arr, cs_arr in [
    ("q (pos)", q_np, q_cs),
    ("v (vel)", v_np, v_cs),
    ("a (acc)", a_np, a_cs),
]:
    max_diff = float(np.max(np.abs(np_arr - cs_arr)))
    rel_diff = max_diff / max(float(np.max(np.abs(np_arr))), 1e-12)
    print(f"  {name}: max_diff={max_diff:.4e}, rel_diff={rel_diff:.4e}")

print("-" * 70)
overall_max = max(
    float(np.max(np.abs(q_np - q_cs))),
    float(np.max(np.abs(v_np - v_cs))),
    float(np.max(np.abs(a_np - a_cs))),
)
if overall_max < 1e-10:
    print("  >>> MX and NumPy trajectories are IDENTICAL.")
else:
    print("  >>> MX and NumPy trajectories DIFFER!")
print("=" * 70)

# ---- 11. Full NLP-like pipeline: MX trajectory -> map -> FIM ------
#     Reuse MX trajectory and NumPy trajectory from section 10.
#     Feed the SAME Fourier trajectory into W_map_fun both ways.
print()
print("=" * 70)
print("Full pipeline: MX trajectory vs NumPy trajectory -> regressor -> FIM")
print("-" * 70)

W_map_fun = W_fun.map(N_s, "openmp")  # same as NLP

# ---- 11a. NumPy path: NumPy trajectory -> map -> FIM ----
Q_np = q_np.T  # (n_act, N_s) — note: this is the FOURIER trajectory, not random
V_np = v_np.T
A_np = a_np.T
W_raw_np = np.array(W_map_fun(Q_np, V_np, A_np))
W_blocks_np = np.hsplit(W_raw_np, N_s)
W_full_np = np.vstack(W_blocks_np)
W_e_np = W_full_np[:, [i for i in range(W_full_np.shape[1]) if i not in idx_e]]
W_b_np = W_e_np[:, list(idx_b)]
FIM_np = (W_b_np.T @ W_b_np) / N_s
FIM_reg_np = FIM_np + 1e-6 * np.eye(FIM_np.shape[0])
sign_np, ld_np = np.linalg.slogdet(FIM_reg_np)
dopt_np = float(-ld_np) if sign_np > 0 else float("inf")
print(f"  NumPy path d_opt: {dopt_np:.6f}")

# ---- 11b. MX path: MX trajectory (evaluated) -> map -> FIM ----
W_raw_mx = W_map_fun(Q_mx, V_mx, A_mx)
W_blocks_mx = cs.horzsplit(W_raw_mx, n_param)
W_full_mx = cs.vertcat(*W_blocks_mx)
all_cols_mx = list(range(W_full_mx.shape[1]))
keep_cols_mx = [i for i in all_cols_mx if i not in idx_e]
W_e_mx = W_full_mx[:, keep_cols_mx]
W_b_mx = W_e_mx[:, list(idx_b)]
J_mx = cs.mtimes(W_b_mx.T, W_b_mx) / N_s
J_reg_mx = J_mx + 1e-6 * cs.DM.eye(J_mx.shape[0])
nlp_fn = cs.Function("nlp_test", [coeffs_mx], [J_reg_mx, W_b_mx])
J_reg_val, W_b_val = nlp_fn(coeffs_np)
J_reg_np = np.array(J_reg_val)
sign_mx, ld_mx = np.linalg.slogdet(J_reg_np)
dopt_mx = float(-ld_mx) if sign_mx > 0 else float("inf")
print(f"  MX path d_opt:    {dopt_mx:.6f}")

# ---- 11c. Compare ----
fim_pipeline_diff = float(np.max(np.abs(J_reg_np - FIM_reg_np)))
print(f"  FIM_reg diff (MX vs NumPy): {fim_pipeline_diff:.4e}")
print(f"  d_opt diff:                 {abs(dopt_mx - dopt_np):.6e}")

if fim_pipeline_diff < 1e-8:
    print("  >>> MX and NumPy paths are IDENTICAL.")
    print("  >>> Everything is consistent for this 2-DOF robot.")
else:
    print("  >>> MX and NumPy paths DIVERGE!")
    print("  >>> This is the root cause of obj != d_opt.")
    # Pinpoint: where does the divergence happen?
    W_b_val_np = np.array(W_b_val)
    wb_max_diff = float(np.max(np.abs(W_b_val_np - W_b_np)))
    print(f"  >>> W_b max diff: {wb_max_diff:.4e}")

print("=" * 70)
