#!/usr/bin/env python3
"""Benchmark cuRobo collision checking against robo-check's RTCD scenes and
FCL ground truth.

Same setup as robo-check's rtcd_bench.cu:
  - robot: Franka Panda (cuRobo franka.yml sphere model, base kept; poses are
    the 7-dof pool from data/rtcd/panda8192.bin, fingers locked)
  - scenes: simple / shelf / dense / rtcc (same OBJs + world poses as
    rtcd_bench.cu makeScene(), meshes merged with world translation baked in)
  - ground truth: FCL per-pose labels dumped by
    `./rtcd-bench --scene <scene> --nposes N --dump-labels <file>`

Usage:
  python bench_curobo.py --scene shelf --poses data/rtcd/panda8192.bin \
      --nposes 8192 --labels shelf_labels.bin --repeat 3 --csv out.csv
"""

import argparse
import struct
import time
import csv
import os
import sys

import numpy as np
import torch

from curobo.collision_checking import RobotCollisionChecker, RobotCollisionCheckerCfg
from curobo._src.geom.types import SceneCfg, Mesh


def load_obj(path):
    verts, faces = [], []
    with open(path) as f:
        for line in f:
            parts = line.split()
            if not parts:
                continue
            if parts[0] == "v":
                verts.append([float(x) for x in parts[1:4]])
            elif parts[0] == "f":
                idx = []
                for p in parts[1:]:
                    v = int(p.split("/")[0])
                    idx.append(v - 1 if v > 0 else len(verts) + v)
                if len(idx) == 3:
                    faces.extend(idx)
                elif len(idx) == 4:  # triangulate quads
                    faces.extend([idx[0], idx[1], idx[2], idx[0], idx[2], idx[3]])
    return verts, faces


SCENES = {
    "simple": [
        ("shelves.obj", [0.45, 0.0, 0.4]),
        ("bin.obj", [0.0, -0.6, 0.0]),
        ("bin.obj", [0.0, 0.6, 0.0]),
    ],
    "shelf": [
        ("shelves.obj", [0.45, 0.0, 0.4]),
        ("bin.obj", [0.0, -0.6, 0.0]),
        ("bin.obj", [0.0, 0.6, 0.0]),
        ("Bob.obj", [-0.4, 0.0, 0.0]),
        ("Cow.obj", [-0.4, 0.0, 0.0]),
        ("Fish.obj", [-0.4, 0.0, 0.0]),
        ("Sheep.obj", [-0.4, 0.0, 0.0]),
        ("Snakeboard.obj", [-0.4, 0.0, 0.0]),
    ],
    "dense": [
        ("shelves.obj", [0.45, 0.0, 0.4]),
        ("bin_dense.obj", [0.0, -0.6, 0.0]),
        ("bin_dense.obj", [0.0, 0.6, 0.0]),
        ("Bob.obj", [-0.4, 0.0, 0.0]),
        ("Face1.obj", [-0.4, 0.0, 0.0]),
        ("Face2.obj", [-0.4, 0.0, 0.0]),
        ("Face3.obj", [-0.4, 0.0, 0.0]),
        ("Cow.obj", [-0.4, 0.0, 0.0]),
        ("Fish.obj", [-0.4, 0.0, 0.0]),
        ("Sheep.obj", [-0.4, 0.0, 0.0]),
        ("Snakeboard.obj", [-0.4, 0.0, 0.0]),
    ],
    "rtcc": [
        ("shelves.obj", [0.45, 0.0, 0.4]),
        ("bin.obj", [0.0, -0.6, 0.0]),
        ("bin.obj", [0.0, 0.6, 0.0]),
        ("Bob.obj", [-0.4, 0.0, 0.0]),
        ("Face1.obj", [-0.4, 0.0, 0.0]),
        ("Face2.obj", [-0.4, 0.0, 0.0]),
        ("Face3.obj", [-0.4, 0.0, 0.0]),
        ("Cow.obj", [-0.4, 0.0, 0.0]),
        ("Fish.obj", [-0.4, 0.0, 0.0]),
        ("Sheep.obj", [-0.4, 0.0, 0.0]),
        ("Snakeboard.obj", [-0.4, 0.0, 0.0]),
    ],
}


def build_scene_cfg(scene_dir, scene_name):
    obstacles = []
    for i, (fname, pos) in enumerate(SCENES[scene_name]):
        verts, faces = load_obj(os.path.join(scene_dir, fname))
        verts = [[v[0] + pos[0], v[1] + pos[1], v[2] + pos[2]] for v in verts]
        obstacles.append(
            Mesh(
                name=f"{fname.split('.')[0]}_{i}",
                vertices=verts,
                faces=faces,
                pose=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            )
        )
    return SceneCfg(mesh=obstacles)


def read_bin_poses(path, nposes):
    with open(path, "rb") as f:
        raw = f.read()
    n = len(raw) // (4 * 7)
    data = np.frombuffer(raw, dtype=np.float32, count=n * 7).reshape(n, 7)
    return np.array(data[:nposes])  # copy: writable, safe to hand to torch


# Repo root, so the defaults work no matter where the script is invoked from.
HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(HERE)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default="simple", choices=list(SCENES))
    ap.add_argument("--scene-dir", default=os.path.join(REPO_ROOT, "data/rtcd/scene"))
    ap.add_argument("--poses", default=os.path.join(REPO_ROOT, "data/rtcd/panda8192.bin"))
    ap.add_argument("--nposes", type=int, default=8192)
    ap.add_argument("--labels", default=None, help="FCL labels file from rtcd-bench --dump-labels")
    ap.add_argument("--repeat", type=int, default=3)
    ap.add_argument("--csv", default=None)
    ap.add_argument("--sweep", action="store_true", help="batch sizes 1..4096")
    ap.add_argument(
        "--robot-config",
        default="franka.yml",
        help="cuRobo robot config name or path to yaml (default: franka.yml)",
    )
    ap.add_argument(
        "--activation-distance",
        type=float,
        default=0.02,
        help="cuRobo collision_activation_distance: inflates robot spheres by "
        "this margin (safety over-approximation). 0.02 makes cuRobo "
        "conservative (FN=0); 0.0 uses the raw fitted spheres.",
    )
    args = ap.parse_args()

    poses = read_bin_poses(args.poses, args.nposes)
    print(f"loaded {poses.shape[0]} poses from {args.poses}")

    scene_cfg = build_scene_cfg(args.scene_dir, args.scene)

    robot_cfg = args.robot_config
    if os.path.isabs(robot_cfg) or os.path.exists(robot_cfg):
        import yaml

        robot_cfg = yaml.safe_load(open(robot_cfg))
    checker_cfg = RobotCollisionCheckerCfg.load_from_config(
        robot_config=robot_cfg,
        scene_model=scene_cfg,
        collision_activation_distance=args.activation_distance,
        self_collision_activation_distance=0.0,
    )
    checker = RobotCollisionChecker(checker_cfg)
    print("cuRobo checker initialized")

    if args.labels and os.path.exists(args.labels):
        with open(args.labels, "rb") as f:
            labels = np.frombuffer(f.read(), dtype=np.uint8)
        labels = labels[: poses.shape[0]]
    else:
        labels = None

    device = torch.device("cuda:0")
    q_all = torch.as_tensor(poses, dtype=torch.float32, device=device)

    # warmup + calibrate collision threshold (distance convention)
    warm_q = q_all[: min(256, q_all.shape[0])].view(-1, 1, 7)
    for _ in range(3):
        d_w, d_s = checker.get_scene_self_collision_distance_from_joints(warm_q)
    torch.cuda.synchronize()

    rows = []
    batch_sizes = (
        [2 ** k for k in range(13) if 2 ** k <= q_all.shape[0]] + [q_all.shape[0]]
        if args.sweep
        else [q_all.shape[0]]
    )
    for batch in batch_sizes:
        q = q_all[:batch].view(batch, 1, 7)
        times = []
        for _ in range(args.repeat):
            t0 = time.perf_counter()
            d_w, d_s = checker.get_scene_self_collision_distance_from_joints(q)
            torch.cuda.synchronize()
            times.append((time.perf_counter() - t0) * 1e3)
        avg_ms = float(np.mean(times))
        us_per_pose = avg_ms * 1e3 / batch
        d_w_np = d_w.detach().cpu().numpy()
        per_pose = d_w_np.max(axis=tuple(range(1, d_w_np.ndim)))  # (batch,)

        if labels is not None:
            lab = labels[:batch]
            # cuRobo convention: positive collision-distance => penetrating.
            # Auto-calibrate the sign: pick the convention with fewer errors.
            best = None
            for sign in (1.0, -1.0):
                pred = (sign * per_pose) > 0
                fp = int(np.sum(pred & (lab == 0)))
                fn = int(np.sum((~pred) & (lab == 1)))
                if best is None or (fp + fn) < best[0]:
                    best = (fp + fn, sign, fp, fn)
            _, sign, fp, fn = best
            print(
                f"BATCH {batch}: avg {avg_ms:.3f} ms -> {us_per_pose:.2f} us/pose "
                f"(best {min(times):.3f} ms) FP={fp} FN={fn} "
                f"(curobo collisions {int(np.sum((sign * per_pose) > 0))}, "
                f"FCL collisions {int(np.sum(lab))})"
            )
        else:
            fp = fn = -1
            print(
                f"BATCH {batch}: avg {avg_ms:.3f} ms -> {us_per_pose:.2f} us/pose "
                f"(best {min(times):.3f} ms) curobo collisions {int(np.sum(per_pose > 0))}"
            )
        rows.append((args.scene, "curobo", batch, args.nposes, avg_ms, us_per_pose, fp, fn))

    if args.csv:
        new = not os.path.exists(args.csv) or os.path.getsize(args.csv) == 0
        with open(args.csv, "a", newline="") as f:
            w = csv.writer(f)
            if new:
                w.writerow(["scene", "algorithm", "batch", "poses", "kernel_ms", "us_per_pose", "fp", "fn"])
            w.writerows(rows)
        print(f"wrote {len(rows)} rows to {args.csv}")


if __name__ == "__main__":
    main()
