#include "trace_cube.hpp"

void getClosestPoint(pcl::KdTreeFLANN<pcl::PointXYZ>& kdTree, std::vector<int>& pointIdxKNNSearch, std::vector<float>& pointKNNSquaredDistance){
    pcl::PointXYZ searchPoint;
    double currentTCP[3] = {};
    getTCPpose(currentTCP);

    //set a point around which we want to start a search
    searchPoint.x = (float)currentTCP[0];
    searchPoint.y = (float)currentTCP[1];
    searchPoint.z = (float)currentTCP[2];

    kdTree.nearestKSearch(searchPoint, 3, pointIdxKNNSearch, pointKNNSquaredDistance);
    return;
}

