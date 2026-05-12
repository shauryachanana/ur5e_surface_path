# UR5e Surface Tracing using ROS2 Jazzy and Gazebo Harmonic
This package, for now is able to spawn a UR5e with a cube in front of it using the ur5e_cube_moveit.launch.py launch file.
The cube is made in tinkercad and can be replaced by any .STL model in "mesh" folder.
After running launch file, use ros2 run ur5e_surface_path trace_cube --ros-args -p use_sim_time:=true in a new terminal to run tracing.

## Tracing logic
After moving to an initial (hardcoded position), the script stores coordinates of the current position of the TCP. Then it extracts the triangles of which the 3D mesh is composed and calculate centre coordinates in x y z for each of them. Then we compare all triangles' positions to a positiion of a TCP and store the closest triangle for later use. After that, by calculating the nomal vectors of each triangle we are able to understand which triangles belong to the same side as the closest one. Finally, the robot arm will move from the centre of the closes triangle, to all triangles that are on the same side and return to an origin pisition (hardcoded).

## Problems
- Orientation logic (turn the TCP in an opposite direction of a normal vector of a triangle) is not functonal yet. One its fixed we should be able to trace every surface and be perpendicular to it.
- Function getCurrentPose() is not functional so we have to use a handwritten analogue for it, which may have problems in itself.
- Tracing works good, but it failed to return to an origin once for me, maybe it was just a bug.
- Tracing is not perfect for easy shapes (like a cube) as they have very small amount of triangles and robot will not cover the whole surface.

## Prerequisites
- Before building, please create empty folders config/ worlds/ rviz/
- ROS2 Jazzy
- Gazebo Harmonic
- RViz2
- Binary/Source install of Universial Robots ROS2 Description (Binary recommended): https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_description/doc/index.html
- Binary/Source install of Universal Robots ROS2 GZ Simulation (Binary recommended): https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_simulation_gz/ur_simulation_gz/doc/index.html

## Authors
- Shaurya Chanana
- Arnab Roy
- Ivan Kuzmichov
