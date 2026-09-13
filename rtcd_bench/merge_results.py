#!/usr/bin/env python3
"""Merge benchmark CSVs into a single comparison table."""
import csv
import os
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))

ROWS = []
scenes = []
with open(os.path.join(HERE, "results/robocheck.csv")) as f:
    for r in csv.DictReader(f):
        ROWS.append(dict(r, fp=0, fn=0))
        if r["scene"] not in scenes:
            scenes.append(r["scene"])

# curobo: keep the largest batch run per scene
for tag, path in (("curobo-default", "results/curobo.csv"), ("curobo-links17", "results/curobo_links17.csv")):
    best = {}
    try:
        f = open(os.path.join(HERE, path))
    except FileNotFoundError:
        continue
    with f:
        for r in csv.DictReader(f):
            s, b = r["scene"], int(r["batch"])
            if s not in best or b > int(best[s]["batch"]):
                best[s] = r
    for s, r in best.items():
        ROWS.append(dict(r, algorithm=tag))

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
    w = csv.DictWriter(f, fieldnames=["scene", "algorithm", "batch", "poses", "kernel_ms", "us_per_pose", "fp", "fn"])
    w.writeheader()
    w.writerows(ROWS)

print(f"{'scene':8} {'method':17} {'us/pose':>9} {'FP':>5} {'FN':>5}")
last = None
for r in ROWS:
    s = r["scene"]
    if s != last:
        if last:
            print()
        last = s
    us = f"{float(r['us_per_pose']):.2f}" if r["us_per_pose"] not in ("-", "") else "-"
    print(f"{s:8} {r['algorithm']:17} {us:>9} {str(r['fp']):>5} {str(r['fn']):>5}")
print("\nwrote results/comparison.csv")
