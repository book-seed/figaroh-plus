#!/usr/bin/env python3
"""FIGAROH 环境验证脚本。

用法:
    python scripts/check_env.py

返回值:
    0 — 全部组件可用
    1 — 有组件缺失（输出详细修复指引）
"""

import sys


def _check(ok: bool, name: str, hint: str = "") -> bool:
    status = "[PASS]" if ok else "[FAIL]"
    print(f"  {status} {name}")
    if not ok and hint:
        for line in hint.strip().splitlines():
            print(f"         {line}")
    return ok


def check_pinocchio() -> bool:
    try:
        import pinocchio
        v = getattr(pinocchio, "__version__", "unknown")
        return _check(True, f"pinocchio (v{v})")
    except ImportError:
        return _check(
            False, "pinocchio",
            "安装: conda install -c conda-forge pinocchio\n"
            "注意: PyPI 的 pin 包不包含 CasADi 绑定，必须用 conda-forge 版本。",
        )


def check_pinocchio_casadi() -> bool:
    try:
        import pinocchio.casadi  # noqa: F401
        return _check(True, "pinocchio.casadi 绑定")
    except ImportError:
        return _check(
            False, "pinocchio.casadi 绑定",
            "CasADi 符号动力学绑定缺失。\n"
            "pixi install -e casadi && pixi run -e casadi build-casadi-openmp",
        )


def check_casadi_ipopt() -> bool:
    try:
        import casadi as cs
    except ImportError:
        return _check(
            False, "casadi",
            "pixi install -e casadi && pixi run -e casadi build-casadi-openmp",
        )
    try:
        x = cs.SX.sym("x")
        cs.nlpsol("_", "ipopt", {"x": x, "f": x ** 2})
        return _check(True, "casadi.nlpsol('ipopt') 可用")
    except Exception:
        return _check(
            False, "casadi.nlpsol('ipopt') 可用",
            "IPOPT 未链接到 CasADi。重新编译: pixi run -e casadi build-casadi-openmp",
        )


def check_casadi_openmp() -> bool:
    """Check OpenMP by actually trying map('openmp')."""
    try:
        import casadi as cs
        x = cs.SX.sym("x")
        f = cs.Function("_", [x], [x ** 2])
        f.map(2, "openmp")
        return _check(True, "CasADi OpenMP 支持")
    except Exception:
        return _check(
            False, "CasADi OpenMP 支持",
            "CasADi 未编译 OpenMP。重新编译: pixi run -e casadi build-casadi-openmp",
        )


def main() -> int:
    print("=" * 50)
    print("FIGAROH 环境验证")
    print("=" * 50)

    results = []

    print("\n[依赖]")
    results.append(("pinocchio", check_pinocchio()))
    results.append(("pinocchio.casadi", check_pinocchio_casadi()))
    results.append(("CasADi + IPOPT", check_casadi_ipopt()))
    results.append(("CasADi OpenMP", check_casadi_openmp()))

    print("\n" + "=" * 50)
    passed = sum(1 for _, ok in results if ok)
    total = len(results)
    print(f"结果: {passed}/{total} 通过")

    if passed < total:
        print("\n未通过项 — 按上方提示修复后重新运行:")
        for name, ok in results:
            if not ok:
                print(f"  - {name}")

    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
