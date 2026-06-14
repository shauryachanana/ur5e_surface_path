#include "trace_cube.hpp"

// std::unordered_map<std::pair<int,int>, int> edgeDetection;
std::unordered_map<std::pair<int,int>, int, PairHash> edgeDetection;

#ifdef POINTCLOUDS
void triangleExtraction(std::vector<Triangle> &vectorOfTriangles, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
#endif
void triangleExtraction(std::vector<Triangle> &vectorOfTriangles){
    //reserve memory for the triangles
    vectorOfTriangles.reserve(mesh->triangle_count);
    #ifdef POINTCLOUDS
    cloud->points.reserve(mesh->triangle_count);
    #endif

    //store trangles to a vector
    //will upload a scetch on how the calculation of index is done
    for(size_t i = 0; i< mesh->triangle_count; i++){

        //create a triangle
        Triangle triangle;
        //create an ibj to store coords in it
        #ifdef POINTCLOUDS
        pcl::PointXYZ center;
        #endif

        /*triangles are described in versices:
        triangle1 = vertix1 vertix2 vertix3
        so we calculate index of vertices of each triangle here:
        */
        int firstVertexOfTriangle = mesh->triangles[i*3 + 0];
        int secondVertexOfTriangle = mesh->triangles[i*3 + 1];
        int thirdVertexOfTriangle = mesh->triangles[i*3 + 2];

        int verteces[3] = {firstVertexOfTriangle, secondVertexOfTriangle, thirdVertexOfTriangle};

        /*
        we store all the edges in a map
        if the edge is the same in the future, the map will be rewritten (if 1 had the same edge as 2, 2 will be stored in a map)
        then we call a funtion to assign every truengle it's neighbours
        triangle 1 with list of Edges stored in it, will ask map using an Edge as a key(3 times), will see that 2 has the same Edge,
        triangle 1 will have "2" stored in its neighbours list and triangle 2 will store "1" automatically

        later when we start an operation all the triangles will have 3 neighbpurs and will be able to detect the closest triangles
        using this map (give ma  the edge, see the neighour, check if reachable/traced and then trace)
        */

        /*now we need to get actual xyz of each point*/
        //first vertex
        for(int j = 0; j<3; j++){
            triangle.x[j] = mesh->vertices[verteces[j] * 3 + 0];
            triangle.y[j] = mesh->vertices[verteces[j] * 3 + 1];
            triangle.z[j] = mesh->vertices[verteces[j] * 3 + 2];
        }

        #ifdef POINTCLOUDS
        //centre of triangle FOR A POINT CLOUD
        center.x = triangle.centre_x = (triangle.x[0] + triangle.x[1] + triangle.x[2]) / 3.0f;
        center.y = triangle.centre_y = (triangle.y[0] + triangle.y[1] + triangle.y[2]) / 3.0f;
        center.z = triangle.centre_z = (triangle.z[0] + triangle.z[1] + triangle.z[2]) / 3.0f; 
        #endif

        triangle.centreOfTriangle[0] = (triangle.x[0] + triangle.x[1] + triangle.x[2]) / 3.0f;
        triangle.centreOfTriangle[1] = (triangle.y[0] + triangle.y[1] + triangle.y[2]) / 3.0f;
        triangle.centreOfTriangle[2] = (triangle.z[0] + triangle.z[1] + triangle.z[2]) / 3.0f; 

        triangle.myIndex = i;

        //give each triangle its set of edges
        //so an index that has xyz of a vertex in it
        Edge e1;
        e1.normalizeEdge(firstVertexOfTriangle, secondVertexOfTriangle);
        Edge e2;
        e2.normalizeEdge(secondVertexOfTriangle, thirdVertexOfTriangle);
        Edge e3;
        e3.normalizeEdge(firstVertexOfTriangle, thirdVertexOfTriangle);

        //store triangle edges in triangles
        triangle.triangleEdges[0] = e1;
        triangle.triangleEdges[1] = e2;
        triangle.triangleEdges[2] = e3;

        //centre of an edge line (in case center-to-center is not availiable)
        for (int j = 0; j < 3; j++) {
            int next = (j + 1) % 3;
            triangle.triangleEdges[j].centreOfEdge[0] = (triangle.x[j] + triangle.x[next]) / 2.0f;
            triangle.triangleEdges[j].centreOfEdge[1] = (triangle.y[j] + triangle.y[next]) / 2.0f;
            triangle.triangleEdges[j].centreOfEdge[2] = (triangle.z[j] + triangle.z[next]) / 2.0f;

            auto key = std::make_pair(triangle.triangleEdges[j].v1, triangle.triangleEdges[j].v2);
            triangle.findNeighbours(vectorOfTriangles, key, j);
        }

        //store the orientation
        triangle.calculateTriangleNormal(i);

        //add this triangle to our vector
        vectorOfTriangles.push_back(triangle);

        #ifdef POINTCLOUDS
        //store the centre to a point cloud for nearest neighbour usage
        cloud->push_back(center);
        #endif
    }
    return;
}

int getClosestTriangle(std::vector<Triangle> &vectorOfTriangles, double* currentTCP){
    int closestTriangle = -1;
    float minDist = 100000;
    for(unsigned int i = 0; i < vectorOfTriangles.size(); i++){
        float distanceToTriangle = distanceToTCP(vectorOfTriangles[i], currentTCP);
        if(distanceToTriangle < minDist){
            minDist = distanceToTriangle;
            closestTriangle = i;
        }
    }
    return closestTriangle;
}

float distanceToTCP(Triangle &triangle, double* currentTCP){
    float distanceToTCP = sqrt(
        pow((triangle.centreOfTriangle[0] * 0.001f) - currentTCP[0], 2) + 
        pow((triangle.centreOfTriangle[1]* 0.001f) - currentTCP[1], 2) + 
        pow((triangle.centreOfTriangle[2] * 0.001f) - currentTCP[2], 2)
    );
    return distanceToTCP;
}