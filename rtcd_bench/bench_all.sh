#!/usr/bin/env bash
# Full three-way benchmark: robo-check (GPU BVH) vs cuRobo (GPU spheres) vs FCL (CPU).
# Self-contained for a fresh clone: builds robo-check if needed, generates the
# FCL ground-truth labels, runs all frameworks, merges the CSVs.
#
# Prereqs (installed by ./SETUP.sh --yes):
#   - CUDA toolkit (nvcc) + Eigen + libccd + FCL 0.7
#   - python3 with: torch, cuda-core[cu13], and cuRobo (pip install /path/to/curobo)
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(dirname "$HERE")"
cd "$HERE"
mkdir -p results

NPOSES=${NPOSES:-8192}
SCENES=${SCENES:-"simple shelf dense rtcc"}

# ---- build robo-check (auto-discovers the CUDA toolchain) -------------------
if [ ! -x "$REPO/rtcd-bench" ]; then
    echo "===== building robo-check (BVH, Generate-Tests, rtcd-bench) ====="
    ./build-robocheck.sh BVH Generate-Tests rtcd-bench
fi

# ---- 1. robo-check (GPU) + FCL ground truth + per-pose labels ---------------
echo "===== robo-check (GPU) + FCL ground truth ====="
: > results/robocheck.csv
: > results/fcl.csv
for s in $SCENES; do
    out=$((cd "$REPO" && ./rtcd-bench --scene "$s" --nposes "$NPOSES" --repeat 3 \
        --csv "$HERE/results/robocheck.csv" \
        --dump-labels "$HERE/results/labels_${s}.bin") 2>&1)
    echo "$out" | grep -E "Wrote|FP/FN check|BATCH ${NPOSES}" || echo "$out" | tail -5
    fcl_us=$(echo "$out" | grep -oP 'FCL ground truth:.*-> \K[0-9.]+(?= us/pose)' | head -1)
    [ -n "$fcl_us" ] && echo "$s,$fcl_us" >> results/fcl.csv
done

# ---- 2. cuRobo (default franka.yml) -----------------------------------------
echo "===== cuRobo (default model) ====="
: > results/curobo.csv
for s in $SCENES; do
    python3 bench_curobo.py --scene "$s" --nposes "$NPOSES" \
        --labels "results/labels_${s}.bin" --repeat 3 --csv results/curobo.csv 2>&1 \
      | grep BATCH || true
done

# ---- 3. cuRobo (links 1..7 only, matches RTCD SKIP_BASE) --------------------
echo "===== cuRobo (links 1-7) ====="
: > results/curobo_links17.csv
for s in $SCENES; do
    python3 bench_curobo.py --scene "$s" --nposes "$NPOSES" \
        --robot-config franka_links17.yml \
        --labels "results/labels_${s}.bin" --repeat 3 --csv results/curobo_links17.csv 2>&1 \
      | grep BATCH || true
done

python3 merge_results.py
echo "===== done ====="
