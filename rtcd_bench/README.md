# rtcd_bench — robo-check vs cuRobo vs FCL

Three-way collision-detection comparison on the RTCD scenes (Franka Panda,
8192-pose pool from `data/rtcd/panda8192.bin`).

- **robo-check** — GPU OBB-BVH + exact triangle-triangle (`rtcd-bench` binary)
- **cuRobo** — GPU sphere model (default `franka.yml` and a links-1..7 variant)
- **FCL** — CPU ground truth (`BVHModel<OBBRSSf>`), also the source of the
  FP/FN labels used to validate the GPU results

## Quick start (fresh clone)

```bash
./SETUP.sh --yes        # CUDA toolkit, Eigen, libccd, FCL, googletest, cuRobo
./rtcd_bench/bench_all.sh
```

`bench_all.sh` builds the robo-check binaries (auto-detecting the CUDA
toolchain and GPU arch), generates FCL ground-truth labels, runs all three
frameworks, and writes `rtcd_bench/results/comparison.csv` plus a summary
table on stdout.

Notes:
- No sudo needed: `SETUP.sh` installs what it can, and `build-robocheck.sh`
  falls back to the pip-installed CUDA toolchain
  (`pip install nvidia-cuda-nvcc nvidia-cuda-runtime nvidia-cuda-cccl nvidia-cuda-nvrtc`)
  when there is no system CUDA toolkit — required for GPUs newer than the
  distro's apt toolkit (e.g. Blackwell sm_120 needs CUDA >= 12.8).
- cuRobo needs a python env with `torch`, `cuda-core[cu13]`, and curobo
  installed (`SETUP.sh` does this, or:
  `pip install torch 'cuda-core[cu13]' && pip install /path/to/curobo`).

## Individual commands

```bash
# robo-check only, with FP/FN validation against FCL
../rtcd-bench --scene shelf --nposes 8192 --repeat 3

# cuRobo only (labels first: ../rtcd-bench ... --dump-labels results/labels_shelf.bin)
python3 bench_curobo.py --scene shelf --nposes 8192 --labels results/labels_shelf.bin --repeat 3
python3 bench_curobo.py --scene shelf --sweep ...        # batch-size sweep
python3 bench_curobo.py --activation-distance 0.0 ...    # raw spheres (no safety margin)
python3 bench_curobo.py --robot-config franka_links17.yml ...  # links 1..7 model

# merge CSVs
python3 merge_results.py
```

## Results (8192 poses, RTX 5070 Ti; FCL ground truth: 0 FP/FN baseline)

cuRobo is run with `collision_activation_distance=0.02` (2 cm sphere
inflation), its intended safety margin, which makes it conservative (FN=0).
Timing is scene-only (FK + scene collision distance; self-collision is not
computed at all) to match robo-check/FCL, which check the world only — an A/B
with the combined scene+self kernel showed no measurable difference.

| scene  | method            | us/pose | +verify | FP  | FN  |
|--------|-------------------|--------:|--------:|----:|----:|
| simple | robo-check (quatSAT) |   3.07  |    -    |  0  |  0  |
| simple | curobo-default    |   0.71  |  16.21  | 569 |  0  |
| simple | curobo-links17    |   0.53  |  13.39  | 359 |  0  |
| simple | fcl-cpu           |  17.02  |    -    |  -  |  -  |
| shelf  | robo-check (quatSAT) |   4.52  |    -    |  0  |  0  |
| shelf  | curobo-default    |   1.59  |  18.48  | 534 |  0  |
| shelf  | curobo-links17    |   1.02  |  16.54  | 345 |  0  |
| shelf  | fcl-cpu           |  20.83  |    -    |  -  |  -  |
| dense  | robo-check (quatSAT) |   6.53  |    -    |  0  |  0  |
| dense  | curobo-default    |   1.72  |  17.69  | 533 |  0  |
| dense  | curobo-links17    |   1.15  |  16.67  | 346 |  0  |
| dense  | fcl-cpu           |  22.28  |    -    |  -  |  -  |
| rtcc   | robo-check (quatSAT) |   5.29  |    -    |  0  |  0  |
| rtcc   | curobo-default    |   1.69  |  18.41  | 533 |  0  |
| rtcc   | curobo-links17    |   1.12  |  17.25  | 346 |  0  |
| rtcc   | fcl-cpu           |  22.96  |    -    |  -  |  -  |

Columns:
- **us/pose** — framework kernel time per pose (full GPU pipeline for
  robo-check; sphere FK + distance query for cuRobo; FCL is full CPU).
- **+verify** — *pipeline metric*: total time per pose when every collision
  the GPU checker reports is double-checked by FCL
  (`rtcd-bench --verify`). For cuRobo the verification dominates: ~36-49 us
  per checked pose, and 359-569 of the reported collisions turn out to be
  **false** (FP column). robo-check needs no verification (0 FP / 0 FN).
- **FP/FN** — vs FCL mesh ground truth over all 8192 poses.

Notes:
- robo-check numbers include the full pipeline (FK + BVH traversal + narrow
  phase) on GPU; cuRobo includes sphere FK + distance query (CUDA graph off,
  eager mode). Timings vary with hardware — rerun for your GPU.
- Both sides are warmed before timing: robo-check's `bvh_articulated` dry-run
  launches a 256-config batch (allocations live outside the timed region),
  and `bench_curobo.py` warms each batch size once (3 extra calls at 256
  poses to cache the kernels, plus one call per timed batch size so lazy
  buffer setup is excluded).
- **Accuracy**: cuRobo's sphere model is designed to over-approximate, but the
  shipped `franka.yml` spheres do not strictly cover the meshes (the FCL
  ground truth uses the larger visual meshes; e.g. link6 protrudes up to 9 cm
  beyond its spheres). At `activation_distance=0.0` (raw spheres) cuRobo shows
  FN ~0.5%. With the 2 cm safety margin it reaches FN=0 at the cost of
  FP ~4-7%. robo-check is exact on both axes (0 FP / 0 FN).
- FCL ground truth comes from `rtcd-bench --dump-labels` (per-pose uint8), so
  the cuRobo FP/FN use the exact same poses and scenes.
- The FCL double-check metric (`+verify`) runs `rtcd-bench --verify <file>`
  over exactly the poses cuRobo reported as colliding; the per-check cost is
  dominated by FCL's triangle-level collision test.
