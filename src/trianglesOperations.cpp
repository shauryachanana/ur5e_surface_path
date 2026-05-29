#include "trace_cube.hpp"

void calculateTriangleNormal(Triangle& triangle, int index){
    //its an array of x1y1z1x2y2z2x3y3z3...
    triangle.normal_x = mesh->triangle_normals[index*3 + 0];
    triangle.normal_y = mesh->triangle_normals[index*3 + 1];
    triangle.normal_z = mesh->triangle_normals[index*3 + 2];
}

void triangleExtraction(std::vector<Triangle> &vectorOfTriangles, std::vector<float> &ZofTriang){

    //store trangles to a vector
    //will upload a scetch on how the calculation of index is done
    for(unsigned int i = 0; i< mesh->triangle_count; i++){
        /*triangles are described in versices:
        triangle1 = vertix1 vertix2 vertix3
        so we calculate index of vertices of each triangle here:
        */
        int firstVertexOfTriangle = mesh->triangles[i*3 + 0];
        int secondVertexOfTriangle = mesh->triangles[i*3 + 1];
        int thirdVertexOfTriangle = mesh->triangles[i*3 + 2];

        //create a triangle
        Triangle triangle;

        /*now we need to get actual xyz of each point*/
        //first vertex
        triangle.x0 = mesh->vertices[firstVertexOfTriangle * 3 + 0];
        triangle.y0 = mesh->vertices[firstVertexOfTriangle * 3 + 1];
        triangle.z0 = mesh->vertices[firstVertexOfTriangle * 3 + 2];

        //second vertex
        triangle.x1 = mesh->vertices[secondVertexOfTriangle * 3 + 0];
        triangle.y1 = mesh->vertices[secondVertexOfTriangle * 3 + 1];
        triangle.z1 = mesh->vertices[secondVertexOfTriangle * 3 + 2];

        //third vertex
        triangle.x2 = mesh->vertices[thirdVertexOfTriangle * 3 + 0];
        triangle.y2 = mesh->vertices[thirdVertexOfTriangle * 3 + 1];
        triangle.z2 = mesh->vertices[thirdVertexOfTriangle * 3 + 2];

        //centre of triangle
        triangle.centre_x = (triangle.x0 + triangle.x1 + triangle.x2) / 3.0f;
        triangle.centre_y = (triangle.y0 + triangle.y1 + triangle.y2) / 3.0f;
        triangle.centre_z = (triangle.z0 + triangle.z1 + triangle.z2) / 3.0f;

        //store the orientation
        calculateTriangleNormal(triangle, i);
        
        //add z to a vector
        ZofTriang.push_back(triangle.centre_z);

        //add this triangle to our vector
        vectorOfTriangles.push_back(triangle);
    }

    // int highestTriangIndex = *max_element(ZofTriang.begin(), ZofTriang.end());

    // return highestTriangIndex;
    return;
}

int getClosestTriangle(std::vector<Triangle> &vectorOfTriangles, double* currentTCP){
    int closestTriangle = -1;
    float minDist = 100000;
    for(unsigned int i = 0; i < mesh->triangle_count; i++){
        float distanceToTriangle = sqrt(
            pow((vectorOfTriangles[i].centre_x * 0.001f) - currentTCP[0], 2) + 
            pow((vectorOfTriangles[i].centre_y * 0.001f) - currentTCP[1], 2) + 
            pow((vectorOfTriangles[i].centre_z * 0.001f) - currentTCP[2], 2)
        );
        if(distanceToTriangle < minDist){
            minDist = distanceToTriangle;
            closestTriangle = i;
        }
    }
    return closestTriangle;
}