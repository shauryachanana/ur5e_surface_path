#ifndef TRACE_CUBE_HPP
#define TRACE_CUBE_HPP

#pragma once

#include <vector>
#include <math.h>
#include <algorithm>

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

#include <unordered_map>
#include <utility>
#include <functional>

#define DEBUGGER
#define TEMPORARY
// #define TRIANGLES
#define POINTCLOUDS

extern std::shared_ptr<rclcpp::Node> node;

//to load mesh
extern std::string mesh_path;
extern shapes::Mesh* mesh;

//for movement planning
using moveit::planning_interface::MoveGroupInterface;
extern std::unique_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_interface;

//will be used to detect edges
//we store 2 indeces of the triange corners (edge of a triangle) as a key and then assign it triangle's index
struct PairHash {
    std::size_t operator()(const std::pair<int,int>& p) const {
        std::size_t h1 = std::hash<int>{}(p.first);
        std::size_t h2 = std::hash<int>{}(p.second);
        return h1 ^ (h2 * 2654435761ULL);
    }
};

extern std::unordered_map<std::pair<int,int>, int, PairHash> edgeDetection;
// extern std::unordered_map<std::pair<int,int>, int> edgeDetection;

struct Edge {
    int v1, v2 = 0;
    float center[3] = {0, 0, 0};

    void normalizeEdge(int firstCorner, int secondCorner){
        if(secondCorner > firstCorner){
            v1 = secondCorner;
            v2 = firstCorner;
        } else{
            v1 = firstCorner;
            v2 = secondCorner;
        }
        return;
    }
};

struct Triangle{
    //corners
    int x[3] = {0, 0, 0};
    int y[3] = {0, 0, 0};
    int z[3] = {0, 0, 0};

    //centre of the triangle
    float centre_x, centre_y, centre_z = 0;
    //normal of the triangle
    float normal_x, normal_y, normal_z = 0;

    //three pair of xyz = edges
    Edge triangleEdges[3];
    //neighbours
    int myNeighbours[3] = {-1, -1, -1};
    //vars to not trace triangle multiple times
    int unreachableCounter = 0;
    bool traced = false;

    int myIndex = 0;

    void calculateTriangleNormal(int index){
        //its an array of x1y1z1x2y2z2x3y3z3...
        normal_x = mesh->triangle_normals[index*3 + 0];
        normal_y = mesh->triangle_normals[index*3 + 1];
        normal_z = mesh->triangle_normals[index*3 + 2];
    }

    void findNeighbours(std::vector<Triangle>& vectorOfTriangles, std::pair<int, int> key, int  arrayIndex){
        auto value = edgeDetection.find(key);
        if(value == edgeDetection.end()){
            myNeighbours[arrayIndex] = -1;
            edgeDetection[key] = myIndex;
        }else if(value->second == myIndex){
            //just a temporary solution to not store its own index as the neighbour
            myIndex = myIndex;
        }else{
            myNeighbours[arrayIndex] = value->second;
            for(int j = 0; j<3; j++){
                if(vectorOfTriangles[value->second].myNeighbours[j] == -1){
                    vectorOfTriangles[value->second].myNeighbours[j] = myIndex;
                    break;
                }
            }
        }
        return;
    }
};

//TODO: add a different algorythm to adding "traced" to triangles
//      add edge detection using element of Tringle struct
//      shoud clean Tringle struct

void traceThreeNeighbours(
    std::vector<Triangle> &vectorOfTriangles, 
    int previousTriangle, 
    Triangle& triangleToTrace, 
    Edge& edgeToPrevTriangle
);
void triangleExtraction(
    std::vector<Triangle> &vectorOfTriangles, 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud
);
void getClosestPoint(
    pcl::KdTreeFLANN<pcl::PointXYZ>& kdTree, 
    std::vector<int>& pointIdxKNNSearch, 
    std::vector<float>& pointKNNSquaredDistance
);
void init();
void goHome();
void getTCPpose(double* currentTCP);
int getClosestTriangle(std::vector<Triangle> &vectorOfTriangles, double* currentTCP);
bool moveToPoint(geometry_msgs::msg::Pose target_pose);

#endif