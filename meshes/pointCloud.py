import numpy as np

# cube dimensions in meters
size = 0.5  # 50cm
points_per_face = 2500  # 4 faces × 2500 = 10000 points total

def face_points(n, size):
    xy = np.random.uniform(0, size, (n, 2))
    return xy

points = []

# bottom face (z=0)
xy = face_points(points_per_face, size)
points += [[x, y, 0] for x, y in xy]

# top face (z=0.5)
xy = face_points(points_per_face, size)
points += [[x, y, size] for x, y in xy]

# front face (y=0)
xz = face_points(points_per_face, size)
points += [[x, 0, z] for x, z in xz]

# back face (y=0.5)
xz = face_points(points_per_face, size)
points += [[x, size, z] for x, z in xz]

# left face (x=0)
yz = face_points(points_per_face, size)
points += [[0, y, z] for y, z in yz]

# right face (x=0.5)
yz = face_points(points_per_face, size)
points += [[size, y, z] for y, z in yz]

# save as .pcd
points = np.array(points)
with open("cube_50cm.pcd", "w") as f:
    f.write("# .PCD v0.7\n")
    f.write("VERSION 0.7\n")
    f.write("FIELDS x y z\n")
    f.write("SIZE 4 4 4\n")
    f.write("TYPE F F F\n")
    f.write("COUNT 1 1 1\n")
    f.write(f"WIDTH {len(points)}\n")
    f.write("HEIGHT 1\n")
    f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
    f.write(f"POINTS {len(points)}\n")
    f.write("DATA ascii\n")
    for p in points:
        f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")

print(f"Done! {len(points)} points saved to cube_50cm.pcd")
print("Done!")