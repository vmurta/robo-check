# TODO

> Status: results are **not** publication-ready yet. This file tracks the
> comparison against related work and the gaps that need to be closed first.
> remove profiling code


## Comparison with related work

| | This project | Sui et al. 2024 (arXiv:2409.09918) | cuRobo | FCL |
|---|---|---|---|---|
| Narrow phase | OBB BVH + triangle-triangle (SAT) | RT cores (OptiX) ray-triangle | point-to-SDF (spheres) | CPU BVH |
| Broad phase | OBB + stream compaction (atomic append) | OBB + stream compaction | spheres | BVH |
| GPU hardware | any CUDA GPU | NVIDIA RTX + OptiX only | any CUDA GPU | CPU |
| Watertight mesh required | no (arbitrary OBJ) | yes | no (spheres) | no |
| Precision | double (narrow phase) | single (documented 0.07% FN) | single | float/double |
| Surface contact / coplanar | detected (conservative) | not detected (penetrating only) | n/a | detected |
| CCD | no | yes (swept sphere curves) | yes (conservative advancement) | limited |
| Dense-scene speed | slower | fastest | competitive | slowest |
| Simplicity | high maintenance (own BVH/traversal) | simple (HW abstracts BVH) | moderate | moderate |

## Before this is publication-ready

- [ ] Decide the defensible claim to center the paper on (exactness? portability?
      contact-exact? hardware-agnostic?).
- [ ] Get benchmark results that are stable and reproducible (the current
      `bvh_results.csv` harness measures FP/FN vs FCL, but numbers need
      re-validation and error bars).
- [ ] Add a proper competitor baseline (cuRobo, and if an RTX GPU is
      available, the OptiX RT-DCD/RT-CCD approach).
- [ ] Measure and document the accuracy angle explicitly (FP/FN rates,
      coplanar/contact cases, non-watertight-mesh handling) where this
      project is claimed to be stronger.
- [ ] Decide whether to add CCD (currently unsupported) or explicitly scope
      out of it.
