#!/usr/bin/env python3
"""coverage_report.py stl reachability.csv visited.csv
Option 1: a triangle with executed=1 counts as fully covered.
Probe size is NOT used. Write that limit next to any reported number."""
import argparse, hashlib, struct, sys
import numpy as np

def read_stl(path):
    b = open(path, 'rb').read()
    n = struct.unpack('<I', b[80:84])[0]
    d = np.frombuffer(b, dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]),
                      count=n, offset=84)
    return d['v'].astype(float), hashlib.sha256(b).hexdigest()

def read_csv(path):
    head, rows = [], []
    for line in open(path):
        line = line.strip()
        if not line:
            continue
        (head if line.startswith('#') else rows).append(line)
    return head, [r.split(',') for r in rows[1:]]

ap = argparse.ArgumentParser()
ap.add_argument('stl'); ap.add_argument('reach'); ap.add_argument('visited')
a = ap.parse_args()

tris, sha = read_stl(a.stl)
n = len(tris)
head, rr = read_csv(a.reach)
print("ASSUMPTIONS:", *head, "coverage_rule=visited triangle counts as fully covered (probe size not used)", sep="\n  ")
if not head or f"sha256={sha}" not in head[0]:
    sys.exit("ERROR: STL hash differs from the one used for reachability. Rerun reachability.")
if len(rr) != n:
    sys.exit(f"ERROR: {len(rr)} reachability rows but STL has {n} triangles.")

reach = np.zeros(n, bool)
for i, r in rr:
    reach[int(i)] = (r == '1')
vis = np.zeros(n, bool)
_, vv = read_csv(a.visited)
for i, _, ex in vv:
    if ex == '1':
        vis[int(i)] = True

area = 0.5 * np.linalg.norm(np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]), axis=1)
tot = area.sum()
reach_area = area[reach].sum()
cov_all = area[vis].sum()
cov_reach = area[vis & reach].sum()
missed = np.where(reach & ~vis)[0]
outside = np.where(vis & ~reach)[0]

print(f"\nTotal area          {tot:10.0f} mm2 ({n} triangles)")
print(f"Reachable area      {reach_area:10.0f} mm2 ({100*reach_area/tot:.1f}% of total)")
print(f"Covered area        {cov_all:10.0f} mm2")
if reach_area > 0:
    print(f"HEADLINE covered / reachable: {100*cov_reach/reach_area:.1f}%")
print(f"For comparison covered / total: {100*cov_all/tot:.1f}%")
print(f"Reachable but not visited: {len(missed)} triangles, {area[missed].sum():.0f} mm2 (planner misses)")
print(f"Unreachable: {(~reach).sum()} triangles, {area[~reach].sum():.0f} mm2 (move base or part)")
if len(outside):
    print(f"WARNING: {len(outside)} triangles were visited but marked unreachable. "
          f"Reachability and trace_cube disagree (check lift-off and Y transform).")
np.savetxt("uncovered_reachable.csv", missed, fmt="%d", header="triangle_id")

