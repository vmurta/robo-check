#!/usr/bin/env bash
# Build robo-check's benchmark binaries (BVH, Generate-Tests, rtcd-bench).
#
# Works out of the box on a fresh clone:
#   - CUDA toolchain auto-discovered, in order of preference:
#       1. $CUDA_TOOLKIT_DIR (set to the dir containing bin/nvcc)
#       2. pip-installed nvidia/cu13 or nvidia/cuda_nvcc package (user-space)
#       3. system nvcc on PATH (CUDA_HOME=/usr/local/cuda)
#     (a pip toolchain is needed for GPUs newer than the distro's apt
#     nvidia-cuda-toolkit, e.g. Blackwell sm_120 requires CUDA >= 12.8)
#   - GPU compute capability auto-detected via nvidia-smi (override: GPU_ARCH)
#   - FCL/libccd found in $PREFIX (default ~/usr, as installed by SETUP.sh) or
#     in /usr/local
set -e

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PREFIX="${PREFIX:-$HOME/usr}"

# ---- locate CUDA toolkit ----------------------------------------------------
if [ -n "${CUDA_TOOLKIT_DIR:-}" ]; then
    CUDA_BIN="$CUDA_TOOLKIT_DIR/bin"
    CUDA_LIB="$CUDA_TOOLKIT_DIR/lib"
elif [ -n "${CUDA_HOME:-}" ]; then
    CUDA_BIN="$CUDA_HOME/bin"
    CUDA_LIB="$CUDA_HOME/lib"
else
    # pip-installed toolkit packages live in <python>/site-packages/nvidia/{cu13,cuda_nvcc}
    PY_SITE="$(python3 - <<'EOF'
import sysconfig
print(sysconfig.get_paths()["purelib"])
EOF
)"
    for d in "$PY_SITE/nvidia/cu13" "$PY_SITE/nvidia/cuda_nvcc"; do
        if [ -x "$d/bin/nvcc" ]; then
            CUDA_BIN="$d/bin"; CUDA_LIB="$d/lib"; break
        fi
    done
    if [ -z "${CUDA_BIN:-}" ] && have_nvcc="$(command -v nvcc 2>/dev/null)"; then
        CUDA_BIN="$(dirname "$have_nvcc")"
        CUDA_LIB="$(dirname "$CUDA_BIN")/lib"
    fi
fi
if [ ! -x "${CUDA_BIN:-/nonexistent}/nvcc" ]; then
    echo "build-robocheck.sh: no CUDA toolkit found." >&2
    echo "  Install one with ./SETUP.sh --yes, or:" >&2
    echo "  pip install nvidia-cuda-nvcc nvidia-cuda-runtime nvidia-cuda-cccl nvidia-cuda-nvrtc" >&2
    exit 1
fi
export CUDA_HOME="$(cd "$CUDA_BIN/.." && pwd)"
export PATH="$CUDA_BIN:$PATH"
export LIBRARY_PATH="$CUDA_LIB${LIBRARY_PATH:+:$LIBRARY_PATH}"
echo "[build] CUDA: $(nvcc --version | grep -oE 'release [0-9.]+' | head -1) ($CUDA_HOME)"

# ---- GPU arch ---------------------------------------------------------------
if [ -z "${GPU_ARCH:-}" ]; then
    if command -v nvidia-smi >/dev/null 2>&1; then
        GPU_ARCH="$(nvidia-smi --query-gpu=compute_cap --format=csv,noheader | head -1 | tr -d ' .')"
    else
        GPU_ARCH="90"
        echo "[build] WARNING: nvidia-smi not found, defaulting to sm_$GPU_ARCH"
    fi
fi
# nvcc wants the dotless form: compute_120 / sm_120
GPU_ARCH="$(printf '%s' "$GPU_ARCH" | tr -d ' .')"
ARCH_FLAGS="-gencode arch=compute_${GPU_ARCH},code=sm_${GPU_ARCH}"
echo "[build] GPU arch: sm_${GPU_ARCH}"

# ---- FCL / libccd -----------------------------------------------------------
EXTRA=""
if [ -f "$PREFIX/include/fcl/fcl.h" ]; then
    EXTRA="-I$PREFIX/include -L$PREFIX/lib"
    LD_RPATH="$PREFIX/lib"
elif [ -f "/usr/local/include/fcl/fcl.h" ]; then
    LD_RPATH="/usr/local/lib"
elif [ -f "/usr/include/fcl/fcl.h" ]; then
    LD_RPATH=""
else
    echo "build-robocheck.sh: FCL headers not found (looked in $PREFIX, /usr/local, /usr)." >&2
    echo "  Run ./SETUP.sh --yes to build FCL + libccd." >&2
    exit 1
fi
if [ -d "/usr/include/eigen3" ]; then
    EXTRA="$EXTRA -I/usr/include/eigen3"
fi
echo "[build] FCL: $([ -n "$EXTRA" ] && echo "$PREFIX" || echo "/usr/local")"

CUFLAGS="-lineinfo -Wno-deprecated-declarations --expt-relaxed-constexpr -diag-suppress 20012 -O3 $ARCH_FLAGS $EXTRA"
LDFLAGS="-lfcl -lccd ${EXTRA:+$EXTRA}"
[ -n "$LD_RPATH" ] && LDFLAGS="$LDFLAGS -Xlinker -rpath -Xlinker $LD_RPATH"

make -C "$REPO_DIR" "$@" CUFLAGS="$CUFLAGS" CXXFLAGS="$EXTRA" LDFLAGS="$LDFLAGS"
