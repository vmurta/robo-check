#!/bin/bash
set -e

# Configuration
ALGO="${1:-edge}"
ROBOT_BASE="data/models"
OBSTACLE="data/models/alpha1.0/obstacle.obj"
ITERATIONS=3
LVS=0.05
MAX_EDGES=10000

# Output CSV file
CSV_FILE="${2:-edge_results.csv}"
echo "model,difficulty,size,iteration,num_edges,num_samples,time_init_ms,time_copy_edges_ms,time_edges_ms,time_copy_back_ms,time_cpu_ms,total_gpu_ms,speedup_pct,false_positives,false_negatives" > "$CSV_FILE"

# ---- Lock GPU clocks for reproducible timings ----
# Lock to 90% of the max boost clock: high enough to be representative,
# low enough that sustained load is unlikely to crash or overheat-throttle.
CLOCK_LOCKED=0
if command -v nvidia-smi >/dev/null 2>&1; then
    MAX_SM=$(nvidia-smi --query-gpu=clocks.max.sm --format=csv,noheader,nounits 2>/dev/null | head -1 | tr -d ' ')
    MAX_MEM=$(nvidia-smi --query-gpu=clocks.max.mem --format=csv,noheader,nounits 2>/dev/null | head -1 | tr -d ' ')
    if [[ -n "$MAX_SM" && "$MAX_SM" =~ ^[0-9]+$ ]]; then
        LOCK_SM=$(( MAX_SM * 90 / 100 ))
        if nvidia-smi -lgc "$LOCK_SM,$LOCK_SM" >/dev/null 2>&1; then
            echo "Locked SM clock to $LOCK_SM MHz (max boost: $MAX_SM MHz)"
            CLOCK_LOCKED=1
        else
            echo "WARNING: failed to lock SM clock (permissions? unsupported GPU?); running unlocked" >&2
        fi
    fi
    if [[ "$CLOCK_LOCKED" == 1 && -n "$MAX_MEM" && "$MAX_MEM" =~ ^[0-9]+$ ]]; then
        LOCK_MEM=$(( MAX_MEM * 90 / 100 ))
        if nvidia-smi -lmc "$LOCK_MEM,$LOCK_MEM" >/dev/null 2>&1; then
            echo "Locked memory clock to $LOCK_MEM MHz (max: $MAX_MEM MHz)"
        else
            echo "WARNING: failed to lock memory clock; running unlocked" >&2
        fi
    fi
fi

# Restore default clocks when the script exits
cleanup_clocks() {
    if [[ "$CLOCK_LOCKED" == 1 ]]; then
        nvidia-smi -rgc >/dev/null 2>&1 || true
    fi
}
trap cleanup_clocks EXIT

# Loop over each model directory
for model_dir in data/configurations/alpha data/configurations/octahedron data/configurations/sphere data/configurations/tetrahedron; do
    model_name=$(basename "$model_dir")
    echo "Processing model: $model_name"
    ROBOT="$ROBOT_BASE/$model_name.obj"
    echo "Using robot model: $ROBOT"
    # Loop over each difficulty/size config file
    for conf_file in "$model_dir"/*.conf; do
        [ -f "$conf_file" ] || continue

        difficulty_size=$(basename "$conf_file" .conf)
        [[ "$difficulty_size" == *"10M" ]] && continue  # skip 10M runs

        # Find matching size from our known sizes
        total=0
        for size_spec in "10" "100" "1k" "10k" "100k" "1M" "10M" "1"; do
            if [[ "$difficulty_size" == *"$size_spec" ]]; then
                if [[ "$size_spec" == "1k" ]]; then
                    total=$(( 1000 ))
                elif [[ "$size_spec" == "10k" ]]; then
                    total=$(( 10000 ))
                elif [[ "$size_spec" == "100k" ]]; then
                    total=$(( 100000 ))
                elif [[ "$size_spec" == "1M" ]]; then
                    total=$(( 1000000 ))
                elif [[ "$size_spec" == "10M" ]]; then
                    total=$(( 10000000 ))
                else
                    total=$(( size_spec ))
                fi
                break
            fi
        done

        # Parse difficulty: free, easy, hard, impossible
        diff="free"
        if [[ "$difficulty_size" == "easy"* ]]; then diff="easy"
        elif [[ "$difficulty_size" == "hard"* ]]; then diff="hard"
        elif [[ "$difficulty_size" == "impossible"* ]]; then diff="impossible"
        fi

        echo "Processing: model=$model_name diff=$diff size_spec=$difficulty_size"

        # Run edge validation ITERATIONS times with taskset
        for iter in $(seq 1 "$ITERATIONS"); do
            output=$(taskset -c 2 ./BVH --algo "$ALGO" --dry-run --cpu-check --lvs "$LVS" --max-edges "$MAX_EDGES" "$ROBOT" "$OBSTACLE" "$conf_file")
            echo $output

            # Extract the timing values from the output
            num_edges=$(echo "$output" | grep -oP 'Edge validation: \K[0-9]+' | head -1)
            num_samples=$(echo "$output" | grep -oP 'edges and \K[0-9]+' | head -1)
            time_init=$(echo "$output" | grep -oP 'Initial allocation and transfer to GPU took \K[0-9]+(\.[0-9]+)?' | head -1)
            time_copy_edges=$(echo "$output" | grep -oP 'Copying edges to GPU took \K[0-9]+(\.[0-9]+)?' | head -1)
            time_edges=$(echo "$output" | grep -oP 'BVH Edges GPU kernel took \K[0-9]+(\.[0-9]+)?' | head -1)
            time_copy_back=$(echo "$output" | grep -oP 'Copying results from GPU took \K[0-9]+(\.[0-9]+)?' | head -1)
            time_cpu=$(echo "$output" | grep -oP 'cpu edge validation execution time: \K[0-9]+(\.[0-9]+)?' | head -1)

            # Extract false positives and false negatives
            false_positives=$(echo "$output" | grep -oP 'were true positives and \K[0-9]+' | head -1)
            false_negatives=$(echo "$output" | grep -oP 'were true negatives and \K[0-9]+' | head -1)

            # Default to 0 if not found
            num_edges=${num_edges:-0}
            num_samples=${num_samples:-0}
            time_init=${time_init:-0}
            time_copy_edges=${time_copy_edges:-0}
            time_edges=${time_edges:-0}
            time_copy_back=${time_copy_back:-0}
            time_cpu=${time_cpu:-0}
            false_positives=${false_positives:-0}
            false_negatives=${false_negatives:-0}

            # Any false positive or negative is a hard failure
            if (( false_positives > 0 || false_negatives > 0 )); then
                echo "ERROR: model=$model_name diff=$diff size=$size_spec FP=$false_positives FN=$false_negatives" >&2
                exit 1
            fi

            # Compute total GPU time and speedup over CPU (percentage change from CPU)
            total_gpu=$(awk "BEGIN{print $time_init+$time_copy_edges+$time_edges+$time_copy_back}")
            speedup_pct=$(awk "BEGIN{ cpu=$time_cpu; gpu=$time_init+$time_copy_edges+$time_edges+$time_copy_back; if(cpu>0) printf \"%.2f\", (cpu-gpu)/cpu*100; else printf \"0\" }")

            # Append to CSV
            echo "${model_name},${diff},${total},${iter},${num_edges},${num_samples},${time_init},${time_copy_edges},${time_edges},${time_copy_back},${time_cpu},${total_gpu},${speedup_pct},${false_positives},${false_negatives}" >> "$CSV_FILE"

            # Small delay to avoid overwhelming the CPU
            sleep 0.1
        done
    done
done

echo "Edge benchmark complete. Results stored in $CSV_FILE"
