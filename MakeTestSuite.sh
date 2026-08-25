#!/bin/bash
set -e

mkdir -p data/configurations/{alpha,octahedron,sphere,tetrahedron}

SIZES=(10 100 1000 10000 100000 1000000 10000000)
SIZE_NAMES=("10" "100" "1k" "10k" "100k" "1M" "10M")
MODELS=("alpha" "octahedron" "sphere" "tetrahedron")
MODEL_PATHS=("data/models/alpha1.0/robot.obj" "data/models/octahedron.obj" "data/models/sphere.obj" "data/models/tetrahedron.obj")

DIFFICULTIES=("free" "easy" "hard" "impossible")
# percentage of configs in collision: free=0%, easy=10%, hard=50%, impossible=100%
COLLISION_PCT=(0 10 50 100)

for m in "${!MODELS[@]}"; do
    model="${MODELS[$m]}"
    model_path="${MODEL_PATHS[$m]}"
    for s in "${!SIZES[@]}"; do
        total="${SIZES[$s]}"
        size_name="${SIZE_NAMES[$s]}"
        for d in "${!DIFFICULTIES[@]}"; do
            diff="${DIFFICULTIES[$d]}"
            pct="${COLLISION_PCT[$d]}"
            num_in_collision=$(( total * pct / 100 ))
            outfile="data/configurations/${model}/${diff}${size_name}.conf"
            echo "Generating ${model} ${diff} ${total} -> ${outfile}"
            ./Generate-Tests "$model_path" "$num_in_collision" "$total" "$outfile"
        done
    done
done

echo "Done."
