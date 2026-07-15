#!/usr/bin/env python3
"""环境验证脚本：检查 CasADi/IPOPT/HSL/pinocchio.casadi 各组件状态。

用法:
    python scripts/check_env.py

返回值:
    0 — 全部组件可用
    1 — 有组件缺失（输出详细修复指引）
"""

import sys
import ctypes.util
from pathlib import Path


def _check(ok: bool, name: str, hint: str = "") -> bool:
    """Print check result and return ok."""
    status = "PASS" if ok else "FAIL"
    print(f"  [{status}] {name}")
    if not ok and hint:
        for line in hint.strip().splitlines():
            print(f"         {line}")
    return ok


def check_casadi() -> bool:
    """Check CasADi is importable and has IPOPT."""
    try:
        import casadi as cs
    except ImportError:
        return _check(False, "casadi 包",
                      "安装: pip install casadi 或 conda install -c conda-forge casadi")

    # Check IPOPT availability via nlpsol
    try:
        x = cs.SX.sym("x")
        solver = cs.nlpsol("tester", "ipopt", {"x": x, "f": x**2})
        ok = True
    except Exception:
        ok = False
    _check(ok, "casadi.nlpsol('ipopt') 可用",
           "IPOPT 未链接到 CasADi，请确认安装了带 IPOPT 的 CasADi 构建。\n"
           "conda-forge 版本默认包含 IPOPT。")

    # Check OpenMP
    try:
        has_omp = cs.has_native("OpenMP")
    except Exception:
        has_omp = False
    _check(has_omp, "CasADi OpenMP 支持",
           "OpenMP 不可用，请确保 CasADi 编译时带 -DWITH_OPENMP=ON。\n"
           "conda-forge 版本默认启用 OpenMP。")
    return ok and has_omp


def check_pinocchio_casadi() -> bool:
    """Check pinocchio.casadi is available."""
    try:
        import pinocchio.casadi as cpin
        return _check(True, "pinocchio.casadi 绑定")
    except ImportError:
        return _check(False, "pinocchio.casadi 绑定",
                      "需要 conda-forge 版本 pinocchio（PyPI pin 包不支持 CasADi）。\n"
                      "安装: conda install -c conda-forge pinocchio")


def check_hsl() -> bool:
    """Check HSL linear solver library (ma57) is available for IPOPT."""
    lib_names = ["libhsl.so", "libhsl.dylib", "libcoinhsl.so", "libcoinhsl.dylib"]
    found = any(ctypes.util.find_library(name) or Path(name).is_file()
                for name in lib_names)
    return _check(found, "HSL 线性求解器库 (ma57)",
                  "HSL 未找到，IPOPT 将回退到 mumps。\n"
                  "安装: 从 https://www.hsl.rl.ac.uk/ipopt/ 获取 libhsl.so")


def check_pinocchio() -> bool:
    """Check base pinocchio package (without casadi bindings)."""
    try:
        import pinocchio
        v = getattr(pinocchio, "__version__", "unknown")
        return _check(True, f"pinocchio (v{v})")
    except ImportError:
        return _check(False, "pinocchio",
                      "安装: conda install -c conda-forge pinocchio")


def main() -> int:
    print("=" * 60)
    print("FIGAROH 环境验证脚本")
    print("=" * 60)

    results = []

    print("\n[1] 基础依赖")
    results.append(("CasADi 包 + IPOPT", check_casadi()))
    results.append(("pinocchio", check_pinocchio()))
    results.append(("pinocchio.casadi 绑定", check_pinocchio_casadi()))

    print("\n[2] IPOPT 线性求解器")
    results.append(("HSL (ma57)", check_hsl()))

    print("\n" + "=" * 60)
    n_pass = sum(1 for _, ok in results if ok)
    n_total = len(results)
    print(f"结果: {n_pass}/{n_total} 通过")

    for name, ok in results:
        if not ok:
            print(f"  [{name}] 未通过 — 请按上方提示修复")

    return 0 if n_pass == n_total else 1


if __name__ == "__main__":
    sys.exit(main())
