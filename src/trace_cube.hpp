#ifndef TRACE_CUBE_HPP
#define TRACE_CUBE_HPP

#pragma once

#include <vector>
#include <math.h>

#include <fstream> // reading files from disk
#include <chrono> //for selay of 2s
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <geometric_shapes/shapes.h> //defines what a mesh object is
#include <geometric_shapes/mesh_operations.h> //loads .stl files
#include <geometric_shapes/shape_operations.h> //mesh operations

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

//for point clouds:
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#define DEBUGGER
#define TEMPORARY
// #define TRIANGLES
#define POINTCLOUDS

extern std::shared_ptr<rclcpp::Node> node;

extern std::string mesh_path;
extern shapes::Mesh* mesh;

using moveit::planning_interface::MoveGroupInterface;
extern std::unique_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_interface;

struct Triangle{
    //corners
    float x0, y0, z0; //corner/vertex 0
    float x1, y1, z1; //corner/vertex 1
    float x2, y2, z2; //corner/vertex 2

    float centre_x, centre_y, centre_z; //centre of the triangle

    //normals
    float normal_x, normal_y, normal_z;
};

void triangleExtraction(std::vector<Triangle> &vectorOfTriangles, std::vector<float> &ZofTriang);
void calculateTriangleNormal(Triangle &triangle, int index);
void init();
void goHome();
void getTCPpose(double* currentTCP);
int getClosestTriangle(std::vector<Triangle> &vectorOfTriangles, double* currentTCP);
void getClosestPoint(pcl::KdTreeFLANN<pcl::PointXYZ>& kdTree, std::vector<int>& pointIdxKNNSearch, std::vector<float>& pointKNNSquaredDistance);

#endif