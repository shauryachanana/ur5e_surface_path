#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

#include <fstream>    // reading files from disk
#include <vector>     // dynamic list/array
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <geometric_shapes/shapes.h> //defines what a mesh object is
#include <geometric_shapes/mesh_operations.h> //loads .stl files
#include <geometric_shapes/shape_operations.h> //mesh operations

struct Triangle{
    //corners
    float x0, y0, z0; //corner/vertex 0
    float x1, y1, z1; //corner/vertex 1
    float x2, y2, z2; //corner/vertex 2

    float centre_x, centre_y, centre_z; //centre of the triangle

    //normals
    float normal_x, normal_y, normal_z;
};

void calculateTriangleNormal(Triangle& triangle){
    // two edges of the triangle
    float edge1x = triangle.x1 - triangle.x0;
    float edge1y = triangle.y1 - triangle.y0;
    float edge1z = triangle.z1 - triangle.z0;

    float edge2x = triangle.x2 - triangle.x0;
    float edge2y = triangle.y2 - triangle.y0;
    float edge2z = triangle.z2 - triangle.z0;

    // cross product = normal vector
    float nx = edge1y * edge2z - edge1z * edge2y;
    float ny = edge1z * edge2x - edge1x * edge2z;
    float nz = edge1x * edge2y - edge1y * edge2x;

    // normalize
    float length = sqrt(
        pow(nx, 2) + pow(ny, 2) + pow(nz, 2)
    );

    triangle.normal_x = nx / length;
    triangle.normal_y = ny / length;
    triangle.normal_z = nz / length;
}

int main(int argc, char** argv){
    rclcpp::init(argc, argv);

    auto const node = std::make_shared<rclcpp::Node>(
        "trace_cube",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    auto const logger = rclcpp::get_logger("trace_cube");

    using moveit::planning_interface::MoveGroupInterface;

    //create an object and pass our node and the name of the structure you want to control (founf in "Planning" tab)
    auto gripper_group_interface = MoveGroupInterface(node, "ur_manipulator");

    //set id from "Context" tab sed desired planning library (open motion plannin library)
    gripper_group_interface.setPlanningPipelineId("ompl");
    //decides which motion algorithm to use
    gripper_group_interface.setPlannerId("RRTConnectkConfigDefault");

    //~the longer thr better
    gripper_group_interface.setPlanningTime(5.0);
    //from 0 to 1
    gripper_group_interface.setMaxVelocityScalingFactor(1.0);
    //0 to 1, 0 for const velocity 
    gripper_group_interface.setMaxAccelerationScalingFactor(0.0);
    
    RCLCPP_INFO(logger, "Planning pipeline: %s", gripper_group_interface.getPlanningPipelineId().c_str());
    RCLCPP_INFO(logger, "Planner ID: %s", gripper_group_interface.getPlannerId().c_str());
    RCLCPP_INFO(logger, "Planning time: %.2f", gripper_group_interface.getPlanningTime());

    //the initial position (right before the tracing)
    std::vector<double> preferred_joints = {
        -101.0 * M_PI / 180.0,
        -115.0 * M_PI / 180.0,
        -40.0 * M_PI / 180.0,
        -205.0 * M_PI / 180.0,
        -101.0 * M_PI / 180.0,
        -180.0 * M_PI / 180.0
    };

    //sets the joint val as a target but doesnt move yet
    gripper_group_interface.setJointValueTarget(preferred_joints);
    
    moveit::planning_interface::MoveGroupInterface::Plan home_plan;
    //msg type to set a pose
    geometry_msgs::msg::Pose target_pose;
  
    //for the obect created before, call plan which saves the trajectory to reach preferred_joints to home_plan address
    //not a straight line, arbitrary trajectory
    auto ok = static_cast<bool>(gripper_group_interface.plan(home_plan));

    //after conversion to bool we see if position is reachable
    if (!ok) {
        RCLCPP_ERROR(logger, "Phase 1 planning failed!");
        rclcpp::shutdown();
        return 1;
    }

    //if it is reachable, we can and do move to a target joint position
    gripper_group_interface.execute(home_plan);

    //create a vector that contains the set of "waypoints"
    std::vector<geometry_msgs::msg::Pose> target_poses;
    
    /*------------PROCESS STL FILE------------*/

    //use info from a cad file
    shapes::Mesh* mesh = shapes::createMeshFromResource(
        "file:///home/iku/ur5e_ws/src/ur5e_surface_path/meshes/tinkercadCube.stl"
    );

    //create a vector to store all the triangles
    //array is bad bc we dont have an amount of triangles (could use triangle_count but why)
    std::vector<Triangle> vectorOfTriangles;

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
        calculateTriangleNormal(triangle);
        
        //add this triangle to our vector
        vectorOfTriangles.push_back(triangle);
    }

    RCLCPP_WARN(logger, "total traing: %zu", vectorOfTriangles.size());

    //take a position of tcp
    geometry_msgs::msg::PoseStamped tcp_pose = gripper_group_interface.getCurrentPose();
    double currentTCP[3] = {tcp_pose.pose.position.x, tcp_pose.pose.position.y, tcp_pose.pose.position.z};

    //find closest triangle to  a tcp
    int closestTriangle = -1;
    float misDist = 10000;
    for(unsigned int i = 0; i < mesh->triangle_count; i++){
        float distanceToTriangle = sqrt(
            pow(vectorOfTriangles[i].centre_x - currentTCP[0], 2) + 
            pow(vectorOfTriangles[i].centre_y - currentTCP[1], 2) + 
            pow(vectorOfTriangles[i].centre_z - currentTCP[2], 2)
        );
        if(distanceToTriangle < misDist){
            misDist = distanceToTriangle;
            closestTriangle = i;
        }
    }

    //this is out closest triangle to a tcp:
    Triangle closest = vectorOfTriangles[closestTriangle];

    //now find all triangles with ~same orientation
    float normalOfClosestX = closest.normal_x;
    float normalOfClosestY = closest.normal_y;
    float normalOfClosestZ = closest.normal_z;
    
    std::vector<Triangle> sameSideTriangles;

    //small tolarance bc float is stupid
    float tolerance = 0.01f;

    //now chwck if the normal vectors are alligned
    for(unsigned int i = 0; i < vectorOfTriangles.size(); i++){
        float newTriangleNormalX = vectorOfTriangles[i].normal_x - normalOfClosestX;
        float newTriangleNormalY = vectorOfTriangles[i].normal_y - normalOfClosestY;
        float newTriangleNormalZ = vectorOfTriangles[i].normal_z - normalOfClosestZ;

        //if yes then they are on the same side of the cube
        if(std::abs(newTriangleNormalX) < tolerance &&
            std::abs(newTriangleNormalY) < tolerance &&
            std::abs(newTriangleNormalZ) < tolerance){
                sameSideTriangles.push_back(vectorOfTriangles[i]);
        }
    }
    RCLCPP_WARN(logger, "same side: %zu", sameSideTriangles.size());
    /*----------------------------------------*/

    moveit_msgs::msg::RobotTrajectory trajectory;

    /*----------START THE OPERATION----------*/
    for(unsigned int i = 0; i <= sameSideTriangles.size(); i++){
        // if((sameSideTriangles[i].centre_z * 0.001f) > 0.0001f){
            RCLCPP_WARN(logger, "start computation number %d", i);
            //multiply by 0.001f so they are "mm"
            target_pose.position.x = sameSideTriangles[i].centre_x * 0.001f;
            target_pose.position.y = 0.7f + sameSideTriangles[i].centre_y * 0.001f;
            target_pose.position.z = 0.2f + sameSideTriangles[i].centre_z * 0.001f;

            RCLCPP_WARN(logger, "x %2f", target_pose.position.x);
            RCLCPP_WARN(logger, "y %2f", target_pose.position.y);
            RCLCPP_WARN(logger, "z %2f", target_pose.position.z);

            /*calculate the orientation (so its always perpendicular)*/

            target_pose.orientation.x = - 1 / sqrt(2);
            target_pose.orientation.y = 0.0;
            target_pose.orientation.z = 0.0;
            target_pose.orientation.w = 1 / sqrt(2);

            // tf2::Vector3 normal(sameSideTriangles[i].normal_x, sameSideTriangles[i].normal_y, sameSideTriangles[i].normal_z);
            // //TCP should point TO the surface, while normals point OUT -> minus
            // tf2::Vector3 approach = -normal;
            // //need to give tf2 a reference (here we use Z axis)
            // tf2::Vector3 zAxis(0, 0, 1);
            // //find ritation axis (im not sure how)
            // tf2::Vector3 rotationAxis = zAxis.cross(approach);
            // if(rotationAxis.length() < tolerance){
            //     target_pose.orientation.x = 0.0;
            //     target_pose.orientation.y = 0.0;
            //     target_pose.orientation.z = 0.0;
            //     target_pose.orientation.w = 1.0;
            // } else {
            //     rotationAxis.normalize();
            //     float angle = acos(zAxis.dot(approach));
            //     tf2::Quaternion q(rotationAxis, angle);
            //     q.normalize();
            //     target_pose.orientation.x = q.x();
            //     target_pose.orientation.y = q.y();
            //     target_pose.orientation.z = q.z();
            //     target_pose.orientation.w = q.w();
            // }

            //add this pose to our vector
            target_poses.push_back(target_pose);

            //cartesian allows linear movement, no curves
            //computeCartesianPath(vector of the poses, step at every mm, trajectory, avoid colisions?)
            //no movement
            double fraction = gripper_group_interface.computeCartesianPath(target_poses, 0.01, trajectory, true);

            RCLCPP_WARN(logger, "Cartesian Path coverage: %.2f%%", fraction * 100.00);

            //Full Cartesian path achieved
            if (fraction >= 0.9) {
                moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
                //so our new trajectory which was calculated in computeCartesianPath is now stored in new cartesian_plan
                cartesian_plan.trajectory = trajectory;
                //now we can and do 
                gripper_group_interface.execute(cartesian_plan);
            }
            else {
                RCLCPP_ERROR(logger, "Cartesian path only %.1f%% complete — collision likely blocked it!", fraction * 100.0);
                return 1;
            }

            //remove the pose that was just executed, now we have empty array again
            target_poses.pop_back();
        // }else{
            // RCLCPP_WARN(logger, "too low");
        // }
    }
    /*---------------------------------------*/

    preferred_joints = {
        0.0,
        - M_PI / 2.0,
        0.0,
        0.0,
        0.0,
        0.0
    };

    gripper_group_interface.setJointValueTarget(preferred_joints);
    
    ok = static_cast<bool>(gripper_group_interface.plan(home_plan));

    if(!ok){
        RCLCPP_ERROR(logger, "Planning to home failed");
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(logger, "Cube Face Tracing complete. Returning to home position now...");
    gripper_group_interface.execute(home_plan);

    rclcpp::shutdown();
    return 0;
}