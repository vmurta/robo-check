#!/bin/bash
set -e

# Configuration
ALGO="bvh"
ROBOT="data/models/alpha1.0/robot.obj"
OBSTACLE="data/models/alpha1.0/obstacle.obj"
ITERATIONS=3

# Output CSV file
CSV_FILE="bvh_results.csv"
echo "model,difficulty,size,iteration,time_init_ms,time_copy_config_ms,time_bvh_ms,time_copy_back_ms,time_cpu_ms" > "$CSV_FILE"

# Loop over each model directory
for model_dir in data/configurations/alpha data/configurations/octahedron data/configurations/sphere data/configurations/tetrahedron; do
    model_name=$(basename "$model_dir")

    # Loop over each difficulty/size config file
    for conf_file in "$model_dir"/*.conf; do
        [ -f "$conf_file" ] || continue

        difficulty_size=$(basename "$conf_file" .conf)
        [[ "$difficulty_size" == *"10" ]] || continue  # TEMP: only run size-10 tests
        # Parse difficulty and size from filename: e.g., free10, easy1k, hard100k, impossible1M
        # We need to extract difficulty and size parts

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

        # Get the numeric portion if possible for the CSV
        num_str="${difficulty_size//[![:digit:]]/}"

        echo "Processing: model=$model_name diff=$diff size_spec=$difficulty_size"

        # Run BVH ITERATIONS times with taskset
        for iter in $(seq 1 "$ITERATIONS"); do
            # Use taskset to bind to CPU 0 (you can modify the mask as needed)
            # taskset -c 0 binds to the first CPU core
            output=$(taskset -c 0 ./BVH --algo "$ALGO" --dry-run "$ROBOT" "$OBSTACLE" "$conf_file" 2>&1)

            # Extract the five timing values from the output
            time_init=$(echo "$output" | grep -oP 'Initial allocation and transfer to GPU took \K[0-9]+(\.[0-9]+)?' | head -1)
            time_copy_config=$(echo "$output" | grep -oP 'Copying configurations to GPU took \K[0-9]+(\.[0-9]+)?' | head -1)
            time_bvh=$(echo "$output" | grep -oP 'BVH Naive GPU broad phase took \K[0-9]+(\.[0-9]+)?' | head -1)
            time_copy_back=$(echo "$output" | grep -oP 'Copying results from GPU took \K[0-9]+(\.[0-9]+)?' | head -1)
            time_cpu=$(echo "$output" | grep -oP 'CPU collision check took \K[0-9]+(\.[0-9]+)?' | head -1)

            # Default to 0 if not found
            time_init=${time_init:-0}
            time_copy_config=${time_copy_config:-0}
            time_bvh=${time_bvh:-0}
            time_copy_back=${time_copy_back:-0}
            time_cpu=${time_cpu:-0}

            # Append to CSV
            echo "${model_name},${diff},${total},${iter},${time_init},${time_copy_config},${time_bvh},${time_copy_back},${time_cpu}" >> "$CSV_FILE"

            # Small delay to avoid overwhelming the CPU
            sleep 0.1
        done
    done
done

echo "BVH benchmark complete. Results stored in $CSV_FILE"