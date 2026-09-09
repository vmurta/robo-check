#!/bin/bash
# Runs the full collision-detection benchmark suite in one shot:
#   1. robo-check (GPU articulated BVH) on the RTCD scenes + the MoveIt scene
#   2. RTCD (OptiX BM_IAS / BM_RAY) on shelfSimple / shelf / denseShelf
#   3. MoveIt FCL (rtcc_benchmark, sphere + mesh robots) on the MoveIt shelf scene
#
# Mirrors run-bvh.sh: attempts to lock the GPU clocks (needs root; falls back
# to unlocked with a warning) and restores defaults on exit.
#
# Output: results/benchmarks/ (CSVs + logs) and a summary table on stdout.

set -u

ROBOCHECK_DIR="/home/victor/Projects/robo-check"
RTCD_DIR="/home/victor/Projects/RTCollisionDetection"
ROS_WS="/home/victor/ros_ws"
OUT_DIR="$ROBOCHECK_DIR/results/benchmarks"
mkdir -p "$OUT_DIR"

# Final combined CSV: results/<first 10 chars of the latest commit subject,
# spaces -> hyphens>-results.csv
COMMIT_SUBJECT=$(git -C "$ROBOCHECK_DIR" log -1 --pretty=%s 2>/dev/null | head -n1)
COMMIT_TAG=$(printf '%s' "$COMMIT_SUBJECT" | cut -c1-10 | tr ' ' '-')
if [[ -z "$COMMIT_TAG" ]]; then
    COMMIT_TAG="results"
fi
RESULT_CSV="$ROBOCHECK_DIR/results/${COMMIT_TAG}-results.csv"
echo "[output] final results -> $RESULT_CSV"

# ---------------------------------------------------------------------------
# Lock GPU clocks (best effort, same approach as run-bvh.sh)
# ---------------------------------------------------------------------------
CLOCK_LOCKED=0
if command -v nvidia-smi >/dev/null 2>&1; then
    MAX_SM=$(nvidia-smi --query-gpu=clocks.max.sm --format=csv,noheader,nounits 2>/dev/null | head -1 | tr -d ' ')
    if [[ -n "$MAX_SM" && "$MAX_SM" =~ ^[0-9]+$ ]]; then
        LOCK_SM=$(( MAX_SM * 90 / 100 ))
        if nvidia-smi -lgc "$LOCK_SM,$LOCK_SM" >/dev/null 2>&1; then
            echo "[clocks] Locked SM clock to $LOCK_SM MHz (max boost: $MAX_SM MHz)"
            CLOCK_LOCKED=1
            MAX_MEM=$(nvidia-smi --query-gpu=clocks.max.mem --format=csv,noheader,nounits 2>/dev/null | head -1 | tr -d ' ')
            if [[ -n "$MAX_MEM" && "$MAX_MEM" =~ ^[0-9]+$ ]]; then
                LOCK_MEM=$(( MAX_MEM * 90 / 100 ))
                if nvidia-smi -lmc "$LOCK_MEM,$LOCK_MEM" >/dev/null 2>&1; then
                    echo "[clocks] Locked memory clock to $LOCK_MEM MHz (max: $MAX_MEM MHz)"
                fi
            fi
        else
            echo "[clocks] WARNING: failed to lock SM clock (needs root?); running unlocked"
        fi
    else
        echo "[clocks] WARNING: could not query max clocks; running unlocked"
    fi
else
    echo "[clocks] WARNING: nvidia-smi not found; running unlocked"
fi

cleanup() {
    if [[ "$CLOCK_LOCKED" == 1 ]]; then
        nvidia-smi -rgc >/dev/null 2>&1 || true
        nvidia-smi -rmc >/dev/null 2>&1 || true
        echo "[clocks] Restored default clocks"
    fi
    pkill -x roslaunch >/dev/null 2>&1 || true
    pkill -x rosmaster >/dev/null 2>&1 || true
    pkill -x roscore >/dev/null 2>&1 || true
}
trap cleanup EXIT

ROBOCSV="$OUT_DIR/robocheck.csv"
RTCDCSV="$OUT_DIR/rtcd.csv"
FCLCSV="$OUT_DIR/fcl.csv"
rm -f "$ROBOCSV" "$RTCDCSV" "$FCLCSV"

# ---------------------------------------------------------------------------
# 1. robo-check (GPU articulated BVH, 0 FP/FN validated against FCL)
# ---------------------------------------------------------------------------
echo ""
echo "===== robo-check ====="
cd "$ROBOCHECK_DIR"
make rtcd-bench >/dev/null 2>&1 || { echo "ERROR: robo-check build failed"; exit 1; }

run_robocheck() {
    local scene="$1" poses="$2" nposes="$3"
    echo "--- robo-check: scene=$scene poses=$nposes ---"
    ./rtcd-bench --scene "$scene" --poses "$poses" --nposes "$nposes" --repeat 3 --csv "$ROBOCSV" \
        | tee "$OUT_DIR/robocheck_${scene}.log" \
        | grep -E "FP/FN|MISMATCH|WARNING|FCL ground truth|BATCH"
}

run_robocheck simple  ./data/rtcd/panda8192.bin       8192
run_robocheck shelf   ./data/rtcd/panda8192.bin       8192
run_robocheck dense   ./data/rtcd/panda8192.bin       8192
run_robocheck rtcc    ./data/rtcd/panda4096_rtcc.bin  4096

# ---------------------------------------------------------------------------
# 2. RTCD (OptiX discrete benchmarks, Google Benchmark JSON output)
# ---------------------------------------------------------------------------
echo ""
echo "===== RTCD (OptiX) ====="
cd "$RTCD_DIR"
RTCD_BIN="$RTCD_DIR/build/bin/Benchmarks"
for bench in benchmarkShelfSimple benchmarkShelf benchmarkDenseShelf; do
    if [[ ! -x "$RTCD_BIN/$bench" ]]; then
        echo "WARNING: $RTCD_BIN/$bench not built; skipping"
        continue
    fi
    echo "--- RTCD: $bench ---"
    "$RTCD_BIN/$bench" --benchmark_out="$OUT_DIR/rtcd_${bench}.json" --benchmark_out_format=json \
        >"$OUT_DIR/rtcd_${bench}.log" 2>&1 || true
done

python3 - "$OUT_DIR" "$RTCDCSV" <<'PYEOF'
import json, re, sys, glob
out_dir, csv_path = sys.argv[1], sys.argv[2]
rows = []
for f in sorted(glob.glob(out_dir + "/rtcd_benchmark*.json")):
    scene = f.split("rtcd_benchmark")[-1].replace(".json", "")
    d = json.load(open(f))
    for x in d.get("benchmarks", []):
        m = re.search(r"(BM_IAS|BM_RAY)/(\d+)/", x["name"])
        if m and int(m.group(2)) == 4096:
            n = 4096
            rows.append((scene, m.group(1), n, x["real_time"], x["real_time"] / n))
with open(csv_path, "w") as fh:
    fh.write("scene,method,batch,batch_us,us_per_pose\n")
    for r in rows:
        fh.write(f"{r[0]},{r[1]},{r[2]},{r[3]:.3f},{r[4]:.6f}\n")
PYEOF

# ---------------------------------------------------------------------------
# 3. MoveIt FCL (rtcc_benchmark headless, RoboStack ROS noetic env)
# ---------------------------------------------------------------------------
echo ""
echo "===== MoveIt FCL (rtcc_benchmark) ====="
if [[ ! -x "$HOME/micromamba/micromamba" ]]; then
    echo "WARNING: micromamba not found; skipping FCL benchmark"
else
    set +u  # micromamba's hook and ROS setup.bash are not `set -u` safe
    export MAMBA_ROOT_PREFIX="$HOME/micromamba"
    eval "$($HOME/micromamba/micromamba shell hook -s bash)" >/dev/null 2>&1
    if micromamba activate rtcc >/dev/null 2>&1; then
        source "$ROS_WS/devel/setup.bash" 2>/dev/null || true
        set -u
        roscore >"$OUT_DIR/fcl_roscore.log" 2>&1 &
        ROSCORE_PID=$!
        sleep 8

        run_fcl() {
            local launch="$1" tag="$2"
            echo "--- FCL: $tag ---"
            timeout 900 roslaunch rtcc_benchmark "$launch" >"$OUT_DIR/fcl_${tag}.log" 2>&1 || true
            grep -aE "Time for static" "$OUT_DIR/fcl_${tag}.log" | tail -1
        }
        run_fcl FCLBenchmarkHeadless.launch     sphere
        run_fcl FCLBenchmarkHeadlessMesh.launch mesh

        kill "$ROSCORE_PID" >/dev/null 2>&1 || true

        python3 - "$OUT_DIR" "$FCLCSV" <<'PYEOF'
import re, sys, glob
out_dir, csv_path = sys.argv[1], sys.argv[2]
with open(csv_path, "w") as fh:
    fh.write("robot,scene,total_ms,us_per_pose\n")
    for f in sorted(glob.glob(out_dir + "/fcl_*.log")):
        tag = f.split("fcl_")[-1].replace(".log", "")
        txt = open(f, errors="ignore").read()
        m = re.search(r"Time for static poses collision detection:\s*([0-9.]+)\s*ms", txt)
        if m:
            ms = float(m.group(1))
            fh.write(f"{tag},shelf(191k tris),{ms:.2f},{ms/4096*1000:.2f}\n")
PYEOF
        micromamba deactivate >/dev/null 2>&1 || true
        set -u
    else
        set -u
        echo "WARNING: rtcc env missing; skipping FCL benchmark"
    fi
fi

# ---------------------------------------------------------------------------
# Merge into the final unified CSV: results/<commit-tag>-results.csv
# ---------------------------------------------------------------------------
python3 - "$OUT_DIR" "$RESULT_CSV" <<'PYEOF'
import csv, glob, re, sys

out_dir, result_csv = sys.argv[1], sys.argv[2]
rows = []

# robo-check: raw harness CSV + FP/FN from the logs
robocsv = out_dir + "/robocheck.csv"
try:
    with open(robocsv) as f:
        for r in csv.DictReader(f):
            rows.append({
                "benchmark": "robo-check",
                "scene": r["scene"],
                "method": r["algorithm"],
                "batch": r["batch"],
                "total_ms": f'{float(r["kernel_ms"]):.3f}',
                "us_per_pose": f'{float(r["us_per_pose"]):.4f}',
                "fp": "", "fn": "",
            })
except FileNotFoundError:
    pass

# FP/FN per scene from the robo-check logs (last FP/FN line wins)
for scene in ("simple", "shelf", "dense", "rtcc"):
    log = out_dir + f"/robocheck_{scene}.log"
    try:
        txt = open(log, errors="ignore").read()
    except FileNotFoundError:
        continue
    m = re.findall(r"FP/FN check \(.*?\): .*?FP=(\d+) FN=(\d+)", txt)
    if m:
        fp, fn = m[-1]
        for r in rows:
            if r["benchmark"] == "robo-check" and r["scene"] == scene:
                r["fp"], r["fn"] = fp, fn

# RTCD
try:
    with open(out_dir + "/rtcd.csv") as f:
        for r in csv.DictReader(f):
            rows.append({
                "benchmark": "rtcd",
                "scene": r["scene"],
                "method": r["method"],
                "batch": r["batch"],
                "total_ms": f'{float(r["batch_us"]) / 1000.0:.3f}',
                "us_per_pose": f'{float(r["us_per_pose"]):.4f}',
                "fp": "", "fn": "",
            })
except FileNotFoundError:
    pass

# FCL (MoveIt)
try:
    with open(out_dir + "/fcl.csv") as f:
        for r in csv.DictReader(f):
            rows.append({
                "benchmark": "fcl-moveit",
                "scene": r["scene"],
                "method": r["robot"],
                "batch": "4096",
                "total_ms": f'{float(r["total_ms"]):.3f}',
                "us_per_pose": f'{float(r["us_per_pose"]):.2f}',
                "fp": "", "fn": "",
            })
except FileNotFoundError:
    pass

with open(result_csv, "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=["benchmark", "scene", "method", "batch",
                                      "total_ms", "us_per_pose", "fp", "fn"])
    w.writeheader()
    for r in rows:
        w.writerow(r)
print(f"[output] wrote {len(rows)} rows to {result_csv}")
PYEOF

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
echo ""
echo "===== SUMMARY ====="
cat "$RESULT_CSV" 2>/dev/null || true
echo ""
echo "Logs in $OUT_DIR; combined results in $RESULT_CSV"
