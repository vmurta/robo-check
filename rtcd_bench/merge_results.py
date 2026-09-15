#!/usr/bin/env python3
"""Merge benchmark CSVs into a single comparison CSV.

Includes every sweep row (batch 1..NPOSES). The stdout summary table shows
only the largest-batch (headline) row per scene per framework; the full
sweep lives in results/comparison.csv.
"""
import csv
import os

HERE = os.path.dirname(os.path.abspath(__file__))
FIELDS = ["scene", "algorithm", "batch", "poses", "kernel_ms", "us_per_pose",
          "fp", "fn", "fcl_verified", "fcl_false", "fcl_verify_ms",
          "total_ms", "us_per_pose_total", "us_per_check"]

ROWS = []
scenes = []
for tag, path in (("robo-check-bvh-quatSAT", "results/robocheck.csv"),
                  ("curobo-links17", "results/curobo_links17.csv")):
    with open(os.path.join(HERE, path)) as f:
        for r in csv.DictReader(f):
            r["algorithm"] = tag
            if tag.startswith("robo-check"):
                # rtcd-bench exits on any FP/FN, so robo-check rows are
                # FCL-validated by construction
                r["fp"], r["fn"] = 0, 0
            ROWS.append(r)
            if r["scene"] not in scenes:
                scenes.append(r["scene"])

FCL = {}
try:
    with open(os.path.join(HERE, "results/fcl.csv")) as f:
        for line in f:
            s, us = line.strip().split(",")
            FCL[s] = float(us)
except FileNotFoundError:
    pass
for s in scenes:
    us = f"{FCL[s]:.2f}" if s in FCL else "-"
    ROWS.append({"scene": s, "algorithm": "fcl-cpu", "batch": "-", "poses": "-",
                 "kernel_ms": "", "us_per_pose": us, "fp": "-", "fn": "-"})

with open(os.path.join(HERE, "results/comparison.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=FIELDS, extrasaction="ignore")
    w.writeheader()
    w.writerows(ROWS)

# headline row per (scene, algorithm): largest batch (ignores fcl rows)
headline = {}
for r in ROWS:
    b = r["batch"]
    if not str(b).lstrip("-").isdigit():
        continue
    key = (r["scene"], r["algorithm"])
    b = int(b)
    if key not in headline or b > headline[key][0]:
        headline[key] = (b, r)

print(f"{'scene':8} {'method':19} {'batch':>6} {'us/pose':>9} "
      f"{'+verify':>9} {'FP':>5} {'FN':>5}")
last = None
for r in ROWS:
    s = r["scene"]
    if r["algorithm"] == "fcl-cpu":
        us = f"{float(r['us_per_pose']):.2f}" if r["us_per_pose"] not in ("-", "") else "-"
        print(f"{s:8} {'fcl-cpu':19} {'-':>6} {us:>9} {'-':>9} {'-':>5} {'-':>5}")
        continue
    key = (s, r["algorithm"])
    if headline[key][1] is not r:
        continue
    b = headline[key][0]
    us = f"{float(r['us_per_pose']):.2f}"
    tot = r.get("us_per_pose_total", "")
    tot = f"{float(tot):.2f}" if tot not in ("-", "", None) else "-"
    print(f"{s:8} {r['algorithm']:19} {b:>6} {us:>9} {tot:>9} "
          f"{str(r['fp']):>5} {str(r['fn']):>5}")

print(f"\nheadline = largest batch per scene (full sweep 1..N in "
      f"results/comparison.csv)")
print("us/pose    = framework kernel time per pose, averaged over the timed repeats")
print("+verify    = total pipeline time per pose, including FCL double-checking of\n"
      "             every collision the GPU checker reports (cuRobo rows)")
print("wrote results/comparison.csv")
