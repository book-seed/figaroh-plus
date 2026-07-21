#!/usr/bin/env bash
# Build CasADi from source with OpenMP support.
# Idempotent — exits early if already installed.

set -euo pipefail

# Check if CasADi+OpenMP already installed
if python -c "import casadi; casadi.Function('_',[casadi.SX.sym('x')],[0]).map(2,'openmp'); raise SystemExit(0)" 2>/dev/null; then
    echo "CasADi+OpenMP already installed, skipping build."
    exit 0
fi

V=3.7.2
SRC=/tmp/casadi-${V}
BUILD=/tmp/casadi-build

# ── Download source ────────────────────────────────────────
echo "Downloading CasADi ${V}..."
# Try GitHub first, fallback to PyPI
if curl -L --retry 3 -o /tmp/casadi.tar.gz \
    "https://github.com/casadi/casadi/archive/refs/tags/${V}.tar.gz"; then
    echo "GitHub OK, extracting..."
    tar xzf /tmp/casadi.tar.gz -C /tmp/
    rm -f /tmp/casadi.tar.gz
else
    echo "GitHub failed, trying PyPI..."
    python -m pip download --no-binary casadi "casadi==${V}" -d /tmp/cd
    tar xzf "/tmp/cd/casadi-${V}.tar.gz" -C /tmp/
    rm -rf /tmp/cd
fi

# GitHub creates casadi-X.Y.Z, PyPI creates casadi-X.Y.Z — same name
if [ ! -d "${SRC}" ]; then
    # Find actual extracted directory (handle naming variants)
    EXTRACTED=$(ls -d /tmp/casadi-* 2>/dev/null | head -1)
    if [ -n "${EXTRACTED}" ] && [ -d "${EXTRACTED}" ]; then
        mv "${EXTRACTED}" "${SRC}"
    else
        echo "ERROR: cannot find extracted CasADi source in /tmp/"
        exit 1
    fi
fi

# ── Build ───────────────────────────────────────────────────
echo "Building CasADi with OpenMP..."
cmake -S "${SRC}" -B "${BUILD}" \
    -DWITH_OPENMP=ON \
    -DWITH_IPOPT=ON \
    -DCMAKE_INSTALL_PREFIX="${CONDA_PREFIX}" \
    -DCMAKE_PREFIX_PATH="${CONDA_PREFIX}" \
    -DIPOPT_INCLUDE_DIRS="${CONDA_PREFIX}/include/coin-or" \
    -DIPOPT_LIBRARIES="${CONDA_PREFIX}/lib/libipopt.so"

cmake --build "${BUILD}" --parallel "$(nproc)"
cmake --install "${BUILD}"

echo "CasADi+OpenMP build complete."
