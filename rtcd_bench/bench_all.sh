#!/usr/bin/env bash
# Full three-way benchmark: robo-check (GPU BVH, quatSAT) vs cuRobo (GPU
# spheres, links 1..7 only) vs FCL (CPU ground truth).
# Self-contained for a fresh clone: builds robo-check if needed, generates the
# FCL ground-truth labels, runs all frameworks, merges the CSVs.
#
# Thorough mode (default): batch-size sweep for every scene -- 1 pose doubling
# up to NPOSES (1, 2, 4, ..., 4096, NPOSES) -- so launch overhead, occupancy,
# and amortization are all visible in the CSV. Every timed point is the
# AVERAGE of REPEATS timed runs (the harnesses average internally and write
# the average kernel_ms/us_per_pose to the CSV).
#
# Fairness:
#   - robo-check runs quatSAT only (--kernel-mode 64), the production layout
#   - cuRobo runs franka_links17.yml (links 1..7, matches RTCD SKIP_BASE);
#     the stock franka.yml (base + hand + fingers) is NOT run -- those extra
#     spheres are not part of the FCL ground truth, so timing them is unfair
#   - cuRobo timed path is scene-collision only (no self-collision)
#
# Prereqs (installed by ./SETUP.sh --yes):
#   - CUDA toolkit (nvcc) + Eigen + libccd + FCL 0.7
#   - python with: torch, cuda-core[cu13], and cuRobo
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(dirname "$HERE")"
cd "$HERE"
mkdir -p results

NPOSES=${NPOSES:-8192}
SCENES=${SCENES:-"simple shelf dense rtcc"}
REPEATS=${REPEATS:-3}
KERNEL_MODE=${KERNEL_MODE:-64}

# ---- python with cuRobo ------------------------------------------------------
# The cuRobo leg needs a python that can import curobo + torch. Prefer an
# explicit override (PYTHON_CUROBO), else probe: $python3 first, then a
# ~/env-robo-check venv (the common Jetson setup, where the distro python3
# has no cuRobo).
find_curobo_python() {
    for cand in "${PYTHON_CUROBO:-}" python3 "$HOME/env-robo-check/bin/python"; do
        [ -n "$cand" ] || continue
        command -v "$cand" >/dev/null 2>&1 || continue
        if "$cand" -c "import curobo, torch" >/dev/null 2>&1; then
            echo "$cand"
            return 0
        fi
    done
    echo "error: no python with curobo+torch found (set PYTHON_CUROBO=/path/to/python)" >&2
    return 1
}
PYTHON_CUROBO="$(find_curobo_python)"

# ---- per-scene pose pools ----------------------------------------------------
# rtcc ships its own 4096-pose pool; every other scene uses panda8192.bin.
scene_pool() {
    case "$1" in
        rtcc) echo "$REPO/data/rtcd/panda4096_rtcc.bin" ;;
        *)    echo "$REPO/data/rtcd/panda8192.bin" ;;
    esac
}
scene_nposes() {
    case "$1" in
        rtcc) echo 4096 ;;
        *)    echo "$NPOSES" ;;
    esac
}

# ---- build robo-check (auto-discovers the CUDA toolchain) -------------------
if [ ! -x "$REPO/rtcd-bench" ]; then
    echo "===== building robo-check (BVH, Generate-Tests, rtcd-bench) ====="
    ./build-robocheck.sh BVH Generate-Tests rtcd-bench
fi

# ---- 1. robo-check (GPU) sweep + FCL ground truth + per-pose labels ----------
echo "===== robo-check (GPU, quatSAT kmode=$KERNEL_MODE, batch sweep, "
echo "      repeat=$REPEATS averaged) + FCL ground truth ====="
: > results/robocheck.csv
: > results/fcl.csv
for s in $SCENES; do
    n="$(scene_nposes "$s")"
    pool="$(scene_pool "$s")"
    out=$((cd "$REPO" && ./rtcd-bench --scene "$s" --poses "$pool" --nposes "$n" \
        --sweep --repeat "$REPEATS" --kernel-mode "$KERNEL_MODE" \
        --csv "$HERE/results/robocheck.csv" \
        --dump-labels "$HERE/results/labels_${s}.bin") 2>&1)
    echo "$out" | grep -E "Wrote|FP/FN check" | tail -3 || echo "$out" | tail -5
    fcl_us=$(echo "$out" | grep -oP 'FCL ground truth:.*-> \K[0-9.]+(?= us/pose)' | head -1)
    [ -n "$fcl_us" ] && echo "$s,$fcl_us" >> results/fcl.csv
done

# ---- 2. cuRobo (links 1..7, batch sweep, scene-only timing) ------------------
# (the default franka.yml model is intentionally not run: base/hand/finger
# spheres are outside the FCL ground truth)
echo "===== cuRobo (links 1..7, batch sweep, repeat=$REPEATS averaged) ====="
: > results/curobo_links17.csv
for s in $SCENES; do
    n="$(scene_nposes "$s")"
    pool="$(scene_pool "$s")"
    "$PYTHON_CUROBO" bench_curobo.py --scene "$s" --poses "$pool" --nposes "$n" \
        --robot-config franka_links17.yml \
        --labels "results/labels_${s}.bin" --sweep --repeat "$REPEATS" \
        --verify-fcl --csv results/curobo_links17.csv 2>&1 \
      | grep -E "BATCH|FCL VERIFY" || true
done

# stale rows from the old default-model leg (no longer produced)
rm -f results/curobo.csv

"$PYTHON_CUROBO" merge_results.py
echo "===== done ====="
