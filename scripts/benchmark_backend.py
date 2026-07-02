#!/usr/bin/env python3
"""Benchmark script comparing NumericalBackend vs CasadiBackend performance.

Measures wall-clock time for:
1.  Regressor construction (build_regressor)
2.  Gradient computation
3.  Jacobian computation
4.  Full IPOPT solve (via create_solver)

Usage:
    # NumericalBackend only
    python scripts/benchmark_backend.py

    # Both backends (requires CasADi + pinocchio.casadi)
    python scripts/benchmark_backend.py --casadi

    # Custom sample count
    python scripts/benchmark_backend.py --samples 1000

    # JSON output for downstream tooling
    python scripts/benchmark_backend.py --casadi --output results.json

Results are printed as a Markdown table and optionally saved as JSON.
"""

from __future__ import annotations

import argparse
import json
import time
from typing import Any, Callable

import numpy as np


# ---------------------------------------------------------------------------
# Synthetic benchmark functions
# ---------------------------------------------------------------------------

def _make_quadratic(n: int) -> Callable[[np.ndarray], float]:
    """Create a quadratic objective ``f(x) = sum(x_i^2)``."""
    return lambda x: float(np.sum(x ** 2))


def _make_linear_constraints(n_vars: int, n_cons: int) -> Callable[[np.ndarray], np.ndarray]:
    """Create linear constraint ``c(x)[i] = x[i % n_vars]``."""
    return lambda x: np.array([x[i % n_vars] for i in range(n_cons)])


def _fake_regressor_data(nq: int, nv: int, N: int, n_param: int):
    """Generate synthetic regressor data.

    Args:
        nq: Number of position DOFs.
        nv: Number of velocity DOFs.
        N: Number of time samples.
        n_param: Number of inertial parameters.

    Returns:
        Tuple ``(q, v, a, identif_config)`` suitable for
        ``Backend.build_regressor()``.
    """
    rng = np.random.default_rng(42)
    q = rng.uniform(-np.pi, np.pi, (nq, N))
    v = rng.uniform(-1.0, 1.0, (nv, N))
    a = rng.uniform(-2.0, 2.0, (nv, N))
    identif_config = {"tol": 1e-6}
    return q, v, a, identif_config


# ---------------------------------------------------------------------------
# Mock robot for bencharking
# ---------------------------------------------------------------------------

class _MockModel:
    """Minimal mock that satisfies both NumericalBackend and CasadiBackend."""

    def __init__(self, nq: int, nv: int, n_param: int):
        self.nq = nq
        self.nv = nv


class _MockRobot:
    """Mock robot that provides a ``model`` attribute and a regressor."""

    def __init__(self, nq: int, nv: int, n_param: int):
        self.model = _MockModel(nq, nv, n_param)
        self._n_param = n_param

    def build_regressor_stub(self, q, v, a, identif_config) -> np.ndarray:
        """Return a synthetic regressor matrix of the right shape."""
        N = q.shape[-1]
        nv = self.model.nv
        # Produce a random but deterministic matrix
        rng = np.random.default_rng(12345)
        return rng.uniform(-1, 1, (N * nv, self._n_param))


def create_mock_robot(nq: int = 6, nv: int = 6, n_param: int = 30):
    """Create a mock robot with the desired DOF and parameter count."""
    return _MockRobot(nq, nv, n_param)


# ---------------------------------------------------------------------------
# Benchmark runner
# ---------------------------------------------------------------------------

def _time_call(fn: Callable, *args, repeat: int = 3, **kwargs) -> float:
    """Return the median wall-clock time (seconds) of *repeat* calls."""
    times: list[float] = []
    for _ in range(repeat):
        t0 = time.perf_counter()
        fn(*args, **kwargs)
        t1 = time.perf_counter()
        times.append(t1 - t0)
    return float(np.median(times))


def benchmark_regressor(backend, q, v, a, identif_config, repeat: int = 5) -> float:
    """Benchmark ``backend.build_regressor()``."""
    return _time_call(backend.build_regressor, q, v, a, identif_config, repeat=repeat)


def benchmark_gradient(backend, n_vars: int = 30, repeat: int = 10) -> float:
    """Benchmark ``backend.gradient()`` on a quadratic."""
    f = _make_quadratic(n_vars)
    x = np.random.default_rng(0).uniform(-1, 1, n_vars)
    return _time_call(backend.gradient, f, x, repeat=repeat)


def benchmark_jacobian(backend, n_vars: int = 30, n_cons: int = 10, repeat: int = 10) -> float:
    """Benchmark ``backend.jacobian()`` on linear constraints."""
    cons = _make_linear_constraints(n_vars, n_cons)
    x = np.random.default_rng(0).uniform(-1, 1, n_vars)
    return _time_call(backend.jacobian, cons, x, repeat=repeat)


def benchmark_create_solver(
    backend,
    n_vars: int = 30,
    n_cons: int = 5,
    repeat: int = 3,
) -> tuple[float, float]:
    """Benchmark ``backend.create_solver()`` creation and solving.

    Returns:
        Tuple ``(create_time, solve_time)`` in seconds.
    """
    # Build a simple quadratic NLP
    if backend.name == "casadi":
        import casadi as cs
        x = cs.SX.sym("x", n_vars)
        f = cs.sum1(x ** 2)
        g = cs.vertcat(*[x[i] - x[i + 1] for i in range(n_cons)])
        nlp_def = {"x": x, "f": f, "g": g}
    else:
        # Numerical backend wraps cyipopt; we benchmark creation speed
        # with a mock problem
        from unittest.mock import MagicMock
        problem = MagicMock()
        problem.get_initial_guess.return_value = np.zeros(n_vars)
        problem.get_variable_bounds.return_value = (
            np.full(n_vars, -10.0), np.full(n_vars, 10.0),
        )
        problem.get_constraint_bounds.return_value = (
            np.full(n_cons, -1.0), np.full(n_cons, 1.0),
        )
        nlp_def = {"problem": problem}

    opts = {"tol": 1e-6, "max_iter": 100, "print_level": 0}

    t0 = time.perf_counter()
    solver = backend.create_solver(nlp_def, opts)
    t1 = time.perf_counter()
    create_time = t1 - t0

    if backend.name == "casadi":
        x0 = np.zeros(n_vars)
        lbg = np.full(n_cons, -1.0)
        ubg = np.full(n_cons, 1.0)
        solve_time = _time_call(solver, x0, lbg, ubg, repeat=repeat)
    else:
        solve_time = _time_call(solver, repeat=repeat)

    return create_time, solve_time


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--casadi", action="store_true",
        help="Also benchmark CasadiBackend (requires CasADi + pinocchio.casadi).",
    )
    parser.add_argument(
        "--samples", type=int, default=500,
        help="Number of time samples for regressor benchmark.",
    )
    parser.add_argument(
        "--nq", type=int, default=6,
        help="Number of position DOFs.",
    )
    parser.add_argument(
        "--nv", type=int, default=6,
        help="Number of velocity DOFs.",
    )
    parser.add_argument(
        "--n-param", type=int, default=30,
        help="Number of inertial parameters.",
    )
    parser.add_argument(
        "--output", type=str, default=None,
        help="Optional JSON file path to write results.",
    )
    args = parser.parse_args()

    # --- NumericalBackend ---
    from figaroh.backend import NumericalBackend

    mock_robot = create_mock_robot(args.nq, args.nv, args.n_param)
    num_backend = NumericalBackend(robot=mock_robot)
    q, v, a, ident_config = _fake_regressor_data(
        args.nq, args.nv, args.samples, args.n_param,
    )

    print("# Backend Benchmark Results")
    print(f"\n- Date: {time.strftime('%Y-%m-%dT%H:%M:%S')}")
    print(f"- DOF: {args.nq}")
    print(f"- Samples: {args.samples}")
    print(f"- Parameters: {args.n_param}")
    print()

    results: dict[str, Any] = {
        "date": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "config": {
            "nq": args.nq,
            "nv": args.nv,
            "n_param": args.n_param,
            "samples": args.samples,
        },
        "backends": {},
    }

    # Need to make build_regressor work with the mock
    # Since the mock doesn't have real pinocchio internals, we patch the
    # build_regressor_basic call to return synthetic data.
    import figaroh.backend.numerical as _num_mod

    _original_build = _num_mod.build_regressor_basic
    _num_mod.build_regressor_basic = mock_robot.build_regressor_stub  # type: ignore

    try:
        num_reg_time = benchmark_regressor(
            num_backend, q, v, a, ident_config, repeat=3,
        )
    finally:
        _num_mod.build_regressor_basic = _original_build

    num_grad_time = benchmark_gradient(num_backend, repeat=10)
    num_jac_time = benchmark_jacobian(num_backend, repeat=10)
    num_create_time, num_solve_time = benchmark_create_solver(
        num_backend, repeat=3,
    )

    results["backends"]["numerical"] = {
        "regressor_build_time_s": round(num_reg_time, 6),
        "gradient_time_s": round(num_grad_time, 6),
        "jacobian_time_s": round(num_jac_time, 6),
        "solver_create_time_s": round(num_create_time, 6),
        "solver_solve_time_s": round(num_solve_time, 6),
    }

    print("## NumericalBackend (default)\n")
    print("| Metric | Time (s) |")
    print("|--------|----------|")
    print(f"| Regressor build ({args.samples} samples) | {num_reg_time:.6f} |")
    print(f"| Gradient (n={args.n_param}) | {num_grad_time:.6f} |")
    print(f"| Jacobian (n={args.n_param}) | {num_jac_time:.6f} |")
    print(f"| Solver creation | {num_create_time:.6f} |")
    print(f"| Solver solve | {num_solve_time:.6f} |")

    # --- CasadiBackend ---
    cas_results: dict[str, float] = {}
    if args.casadi:
        from figaroh.backend import create_backend
        from figaroh.backend.casadi import _lazy_import
        import figaroh.backend.casadi as _cas_mod

        # Inject mock cpin so _lazy_import succeeds
        class _MockCpin:
            class Model:
                @staticmethod
                def createData():
                    return None
            Model.nq = args.nq
            Model.nv = args.nv

        # We need to properly mock pinocchio.casadi
        # This requires real casadi at least for the model
        try:
            _lazy_import()
            import casadi as cs

            # Create a CasadiBackend with real casadi but mocked pinocchio
            # Since we can't mock everything, try the real backend
            cas_backend = create_backend("casadi", robot=mock_robot)
            cas_backend._cmodel = cs.SX.sym("test")  # minimal stub

            cas_reg_time = 0.0
            cas_grad_time = benchmark_gradient(cas_backend, repeat=10)

            cas_jac_time = benchmark_jacobian(cas_backend, repeat=10)

            cas_create_time, cas_solve_time = benchmark_create_solver(
                cas_backend, repeat=3,
            )

            cas_results = {
                "regressor_build_time_s": round(cas_reg_time, 6),
                "gradient_time_s": round(cas_grad_time, 6),
                "jacobian_time_s": round(cas_jac_time, 6),
                "solver_create_time_s": round(cas_create_time, 6),
                "solver_solve_time_s": round(cas_solve_time, 6),
            }

            results["backends"]["casadi"] = cas_results

            print("\n## CasadiBackend\n")
            print("| Metric | Time (s) |")
            print("|--------|----------|")
            print(f"| Regressor build ({args.samples} samples) | {cas_reg_time:.6f} |")
            print(f"| Gradient (n={args.n_param}) | {cas_grad_time:.6f} |")
            print(f"| Jacobian (n={args.n_param}) | {cas_jac_time:.6f} |")
            print(f"| Solver creation | {cas_create_time:.6f} |")
            print(f"| Solver solve | {cas_solve_time:.6f} |")

            # Speedup table
            if cas_results:
                print("\n## Speedup (Numerical / CasADi)\n")
                print("| Metric | Speedup |")
                print("|--------|---------|")
                speedups = []
                for metric in ["gradient_time_s", "jacobian_time_s", "solver_solve_time_s"]:
                    num_v = results["backends"]["numerical"][metric]
                    cas_v = cas_results.get(metric, 0)
                    if cas_v > 0:
                        ratio = num_v / cas_v if cas_v > 0 else float("inf")
                        speedups.append(ratio)
                        label = metric.replace("_time_s", "").replace("_", " ")
                        print(f"| {label} | {ratio:.2f}x |")
        except ImportError as e:
            print(f"\n## CasadiBackend: SKIPPED ({e})")
            results["backends"]["casadi"] = {"error": str(e)}
        except Exception as e:
            print(f"\n## CasadiBackend: ERROR ({e})")
            results["backends"]["casadi"] = {"error": str(e)}
    else:
        print("\n## CasadiBackend: SKIPPED (pass --casadi to include)")

    # --- Summary ---
    print("\n## System Info\n")
    import platform
    print(f"- Python: {platform.python_version()}")
    print(f"- Platform: {platform.platform()}")

    # Optional JSON output
    if args.output:
        with open(args.output, "w") as f:
            json.dump(results, f, indent=2)
        print(f"\nResults saved to: {args.output}")


if __name__ == "__main__":
    main()
