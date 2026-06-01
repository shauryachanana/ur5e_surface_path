#include "trace_cube.hpp"

//global node
std::shared_ptr<rclcpp::Node> node;
std::unique_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_interface;

//load a file
std::string mesh_path = "file://" + ament_index_cpp::get_package_share_directory("ur5e_surface_path") + "/meshes/50cmCube.stl";
shapes::Mesh* mesh = shapes::createMeshFromResource(mesh_path);

int main(int argc, char** argv){
    rclcpp::init(argc, argv);

    // initialize them here after rclcpp::init()
    node = std::make_shared<rclcpp::Node>(
        "trace_cube",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    gripper_group_interface = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
        node, "ur_manipulator"
    );

    init();

    goHome();

    auto logger = rclcpp::get_logger("main");

    #ifdef DEBUGGER
    RCLCPP_WARN(logger, "home");
    #endif
    
    /*===================FIND THE POINT TO START===================*/

    #ifdef POINTCLOUDS
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Load PCD file to cloud pbject above
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(mesh_path, *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file\n");
        return;
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;

    //create kdTree
    kdTree.setInputCloud(cloud);

    //in this vector we will get an index of the nearest neighbour
    std::vector<int> pointIdxKNNSearch(1);
    //in this vector we will get a distance to the nearest neighbour
    std::vector<float> pointKNNSquaredDistance(1);

    //find the neighbour
    getClosestPoint(kdTree, pointIdxKNNSearch, pointKNNSquaredDistance);

    #endif

    #ifdef TRIANGLES
    //create a vector to store all the triangles
    //array is bad bc we dont have an amount of triangles (could use triangle_count but why)
    std::vector<Triangle> vectorOfTriangles;
    //vector for all z's
    std::vector<float> ZofTriang;

    #ifdef DEBUGGER
    RCLCPP_WARN(logger, "triang extract now:");
    #endif
    triangleExtraction(vectorOfTriangles, ZofTriang);
    
    #ifdef TEMPORARY
    #ifdef DEBUGGER
    RCLCPP_WARN(logger, "current now:");
    #endif
    double currentTCP[3] = {};
    getTCPpose(currentTCP);
    #endif

    #ifndef TEMPORARY
    geometry_msgs::msg::PoseStamped tcp_pose = gripper_group_interface.getCurrentPose();
    double currentTCP[3] = {tcp_pose.pose.position.x, tcp_pose.pose.position.y, tcp_pose.pose.position.z};
    #endif

    #ifdef DEBUGGER
    RCLCPP_WARN(logger, "TCP x: %f", currentTCP[0]);
    RCLCPP_WARN(logger, "TCP y: %f", currentTCP[1]);
    RCLCPP_WARN(logger, "TCP z: %f", currentTCP[2]);
    #endif

    //this is out closest triangle to a tcp:
    Triangle closest = vectorOfTriangles[getClosestTriangle(vectorOfTriangles, currentTCP)];

    #ifdef DEBUGGER
    RCLCPP_WARN(logger, "closest triangle x: %2f", closest.centre_x);
    RCLCPP_WARN(logger, "closest triangle y: %2f", closest.centre_y);
    RCLCPP_WARN(logger, "closest triangle z: %2f", closest.centre_z);
    #endif

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

    #ifdef DEBUGGER
    RCLCPP_WARN(logger, "same side: %zu", sameSideTriangles.size());
    #endif

    #endif
    /*=============================================================*/

    moveit_msgs::msg::RobotTrajectory trajectory;

    //create a vector that contains the set of "waypoints"
    std::vector<geometry_msgs::msg::Pose> target_poses;

    //msg type to set a pose
    geometry_msgs::msg::Pose target_pose;

    /*----------START THE OPERATION----------*/

    #ifdef POINTCLOUDS
    for(unsigned int i = 0; i < 1000; i++){
        getClosestPoint(kdTree, pointIdxKNNSearch, pointKNNSquaredDistance);
        target_pose.position.x = (*cloud)[ pointIdxKNNSearch[0] ].x;
        target_pose.position.y = (*cloud)[ pointIdxKNNSearch[0] ].y + 1.00f;
        target_pose.position.z = (*cloud)[ pointIdxKNNSearch[0] ].z;
        i++;
    }
    #endif

    #ifdef TRIANGLES
    for(unsigned int i = 0; i < sameSideTriangles.size(); i++){
        if((sameSideTriangles[i].centre_z * 0.001f) > 0.0001f){
            //multiply by 0.001f so they are "mm"
            target_pose.position.x = sameSideTriangles[i].centre_x * 0.001f;
            target_pose.position.y = 1.0f - sameSideTriangles[i].centre_y * 0.001f - 0.1f;
            target_pose.position.z = sameSideTriangles[i].centre_z * 0.001f;

            #ifdef DEBUGGER
            RCLCPP_WARN(logger, "x of target %d : %2f", i, target_pose.position.x);
            RCLCPP_WARN(logger, "y of target %d : %2f", i, target_pose.position.y);
            RCLCPP_WARN(logger, "z of target %d : %2f", i, target_pose.position.z);
            #endif

            /*ROTATES THE TCP PERPENDICULARLY TO THE SURFACE*/
            tf2::Vector3 normal(
                sameSideTriangles[i].normal_x,
                sameSideTriangles[i].normal_y,
                sameSideTriangles[i].normal_z
            );

            #ifdef DEBUGGER
            RCLCPP_WARN(logger, "x of normal %d : %2f", i, normal.x());
            RCLCPP_WARN(logger, "y of normal %d : %2f", i, normal.y());
            RCLCPP_WARN(logger, "z of normal %d : %2f", i, normal.z());
            #endif

            normal.normalize();

            tf2::Vector3 z_axis = normal;  // Z into the surface
            z_axis.normalize();

            tf2::Vector3 world_up(0.0, 0.0, -1.0);
            if (std::abs(z_axis.dot(world_up)) > 0.99) {
                world_up = tf2::Vector3(0.0, 1.0, 0.0);
            }

            tf2::Vector3 x_axis = world_up.cross(z_axis);
            x_axis.normalize();

            tf2::Vector3 y_axis = z_axis.cross(x_axis);
            y_axis.normalize();

            tf2::Matrix3x3 rot(
                x_axis.x(), y_axis.x(), z_axis.x(),
                x_axis.y(), y_axis.y(), z_axis.y(),
                x_axis.z(), y_axis.z(), z_axis.z()
            );

            tf2::Quaternion q;
            rot.getRotation(q);
            q.normalize();

            target_pose.orientation.x = q.x();
            target_pose.orientation.y = q.y();
            target_pose.orientation.z = q.z();
            target_pose.orientation.w = q.w();

            #ifdef DEBUGGER
            RCLCPP_WARN(logger, "x of quart : %2f", q.x());
            RCLCPP_WARN(logger, "y of quart : %2f", q.y());
            RCLCPP_WARN(logger, "z of quart : %2f", q.z());
            RCLCPP_WARN(logger, "w of quart : %2f", q.w());
            #endif

            //add this pose to our vector
            target_poses.push_back(target_pose);

            //cartesian allows linear movement, no curves
            //computeCartesianPath(vector of the poses, step at every mm, trajectory, avoid colisions?)
            //no movement
            double fraction = gripper_group_interface->computeCartesianPath(target_poses, 0.01, trajectory, true);

            #ifdef DEBUGGER
                RCLCPP_WARN(logger, "Cartesian Path coverage: %.2f%%", fraction * 100.00);
            #endif

            //Full Cartesian path achieved
            if (fraction >= 0.9) {
                moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
                //so our new trajectory which was calculated in computeCartesianPath is now stored in new cartesian_plan
                cartesian_plan.trajectory = trajectory;
                //now we can and do 
                gripper_group_interface->execute(cartesian_plan);
            }
            else {
                RCLCPP_ERROR(logger, "Cartesian path only %.1f%% complete — collision likely blocked it!", fraction * 100.0);
                return 1;
            }

            //remove the pose that was just executed, now we have empty array again
            target_poses.pop_back();
        }else{
            RCLCPP_WARN(logger, "too low");
        }
    }
    #endif
    /*---------------------------------------*/

    goHome();

    gripper_group_interface.reset();
    node.reset();

    rclcpp::shutdown();
    return 0;
}