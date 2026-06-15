#ifndef TRACE_CUBE_HPP
#define TRACE_CUBE_HPP

#pragma once

#include <vector>
#include <stack>
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
#include <numeric>

#define DEBUGGER
#define TEMPORARY
// #define TRIANGLES
// #define POINTCLOUDS

extern std::shared_ptr<rclcpp::Node> node;

//to load mesh
extern std::string mesh_path;
extern shapes::Mesh* mesh;

//for movement planning
using moveit::planning_interface::MoveGroupInterface;
extern std::unique_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_interface;

extern std::vector<bool> traced;

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

enum class AttemptToReach{
    TRIANGLE_REACHED,
    TRIANGLE_FAILED,
    CENTER_TO_EDGE_FAILED,
    EDGE_TO_CENTER_FAILED,
    EMPTY_VECTOR
};

enum class movementDirection{
    FORWARD,
    BACKWARDS
};

enum class waypointType{
    TRIANGLE,
    EDGE
};

struct Waypoint {
    geometry_msgs::msg::Pose pose;
    waypointType typeOfWaypoint;
    int triangleIndex;
};

extern std::stack<Waypoint> pathHistory;

struct Edge {
    int v1, v2 = 0;
    float centreOfEdge[3] = {0, 0, 0};

    void normalizeEdge(const int firstCorner, const int secondCorner){
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
    float centreOfTriangle[3] = {0,0,0};
    //normal of the triangle
    float normal_x, normal_y, normal_z = 0;

    //three pair of xyz = edges
    Edge triangleEdges[3];
    //neighbours
    std::vector<int> myNeighbours = std::vector<int>(3, -1);
    //vars to not trace triangle multiple times
    int unreachableCounter = 0;
    bool traced = false;

    int myIndex = 0;

    int getValidNeighbours(std::vector<bool> &tracedTriangles, const std::vector<Triangle>& vectorOfTriangles){
        int numbOfNeigh = 0;
        tracedTriangles[0] = tracedTriangles[0];
        for(int i = 0; i<3; i++){
            if((myNeighbours[i] != -1) && (vectorOfTriangles[myNeighbours[i]].traced == false) && (vectorOfTriangles[myNeighbours[i]].unreachableCounter < 3)){
                numbOfNeigh++;
            }
        }
        return numbOfNeigh;
    }

    void calculateTriangleNormal(const int index){
        //its an array of x1y1z1x2y2z2x3y3z3...
        normal_x = mesh->triangle_normals[index*3 + 0];
        normal_y = mesh->triangle_normals[index*3 + 1];
        normal_z = mesh->triangle_normals[index*3 + 2];
    }

    void findNeighbours(std::vector<Triangle>& vectorOfTriangles, const std::pair<int, int> key, const int  arrayIndex){
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

AttemptToReach traceNeighbour(
    Triangle& previousTriangle, 
    Triangle& triangleToTrace, 
    Edge& edgeToPrevTriangle
);
#ifdef POINTCLOUDS
void triangleExtraction(
    std::vector<Triangle> &vectorOfTriangles, 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud
);
#endif
void triangleExtraction(
    std::vector<Triangle> &vectorOfTriangles
);
void getClosestPoint(
    pcl::KdTreeFLANN<pcl::PointXYZ>& kdTree, 
    std::vector<int>& pointIdxKNNSearch, 
    std::vector<float>& pointKNNSquaredDistance
);
int startOperation(
    std::vector<Triangle> vectorOfTriangles, 
    std::vector<bool> &traced, 
    Triangle triangleToTrace
);
void init();
void goHome();
void getTCPpose(double* currentTCP);
void getTCPorientation(double* TCPorientation);
AttemptToReach attemptToReachNextClosest(std::vector<Triangle> vectorOfDesidedTriangles, int &closestTriangleIndex);
geometry_msgs::msg::Pose targetPose(const Triangle &triangle);
int getClosestTriangle(std::vector<Triangle> &vectorOfTriangles, double* currentTCP);
float distanceToTCP(Triangle &triangle, double* currentTCP);
bool moveToPoint(geometry_msgs::msg::Pose target_pose, int triangleIndex, movementDirection movementDir = movementDirection::FORWARD, waypointType waypoint = waypointType::TRIANGLE);
// std::vector<int> triangleWithLeastNeighbours(std::vector<Triangle> &vectorOfTriangles, std::vector<bool> &traced, Triangle triangleToTrace);
std::pair<std::vector<int>, std::vector<int>> triangleWithLeastNeighbours(std::vector<Triangle> &vectorOfTriangles, std::vector<bool> &traced, Triangle triangleToTrace);

#endif