
import re
import math

filename = "/home/shauryachanana/tcp_trace_recovery.yaml"

points = []
current = {}
inside_points = False

number = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"

with open(filename) as f:
    for line in f:
        s = line.strip()

        if s == "points:":
            inside_points = True
            continue

        if inside_points and (
            s.startswith("colors:")
            or s.startswith("texture_resource:")
        ):
            break

        if not inside_points:
            continue

        m = re.match(rf"(?:-\s*)?x:\s*({number})", s)
        if m:
            current["x"] = float(m.group(1))
            continue

        m = re.match(rf"y:\s*({number})", s)
        if m:
            current["y"] = float(m.group(1))
            continue

        m = re.match(rf"z:\s*({number})", s)
        if m:
            current["z"] = float(m.group(1))

            if all(k in current for k in ("x", "y", "z")):
                points.append(
                    (current["x"], current["y"], current["z"])
                )
                current = {}

distance = 0.0

for a, b in zip(points, points[1:]):
    distance += math.dist(a, b)

print("Recovered TCP points:", len(points))
print("Recovered TCP path length: %.3f m" % distance)

if points:
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    zs = [p[2] for p in points]

    print("X range: %.3f to %.3f m" % (min(xs), max(xs)))
    print("Y range: %.3f to %.3f m" % (min(ys), max(ys)))
    print("Z range: %.3f to %.3f m" % (min(zs), max(zs)))
