#include "trace_cube.hpp"

//global node
std::shared_ptr<rclcpp::Node> node;
std::unique_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_interface;

//load a file
std::string mesh_path = "file://" + ament_index_cpp::get_package_share_directory("ur5e_surface_path") + "/meshes/50cmCube.stl";
//now we have info on all the triangles
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
    
    /*===================FIND THE POINT TO START===================*/

    //create a point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    std::vector<Triangle> vectorOfTriangles;

    //now we have all the triangle centres and theor normals stored
    //centers go to the point cloud obj for easy nearest neighbour calculation
    //vectorOfTriangles is used to store the normal vectors of the triangles to orient the TCP
    triangleExtraction(vectorOfTriangles, cloud);

    pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;

    //create kdTree
    kdTree.setInputCloud(cloud);

    //in this vector we will get an index of the nearest neighbour
    std::vector<int> pointIdxKNNSearch(1);
    //in this vector we will get a distance to the nearest neighbour
    std::vector<float> pointKNNSquaredDistance(1);

    /*=============================================================*/

    moveit_msgs::msg::RobotTrajectory trajectory;

    //create a vector that contains the set of "waypoints"
    std::vector<geometry_msgs::msg::Pose> target_poses;

    //msg type to set a pose
    geometry_msgs::msg::Pose target_pose;

    int currentTriangleIndex = 0;

    /*=================GET TO THE CLOSEST TRIANGLE=================*/
    //go  to the closest triangle
    getClosestPoint(kdTree, pointIdxKNNSearch, pointKNNSquaredDistance);

    target_pose.position.x = vectorOfTriangles[ pointIdxKNNSearch[0] ].centre_x * 0.001f;
    target_pose.position.y = vectorOfTriangles[ pointIdxKNNSearch[0] ].centre_y * 0.001f + 1.00f;
    target_pose.position.z = vectorOfTriangles[ pointIdxKNNSearch[0] ].centre_z * 0.001f;

    #ifdef DEBUGGER
    for(int k = 0; k<3; k++){
    RCLCPP_WARN(logger, "x : %f", vectorOfTriangles[ pointIdxKNNSearch[k] ].centre_x * 0.001f);
    RCLCPP_WARN(logger, "y : %f", vectorOfTriangles[ pointIdxKNNSearch[k] ].centre_y * 0.001f);
    RCLCPP_WARN(logger, "z : %f", vectorOfTriangles[ pointIdxKNNSearch[k] ].centre_z * 0.001f);
    }
    #endif

    tf2::Vector3 normal(
        vectorOfTriangles[ pointIdxKNNSearch[0] ].normal_x,
        vectorOfTriangles[ pointIdxKNNSearch[0] ].normal_y,
        vectorOfTriangles[ pointIdxKNNSearch[0] ].normal_z
    );

    currentTriangleIndex = pointIdxKNNSearch[0];

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
    RCLCPP_WARN(logger, "x of norm : %2f", normal[0]);
    RCLCPP_WARN(logger, "y of norm : %2f", normal[1]);
    RCLCPP_WARN(logger, "z of norm : %2f", normal[2]);

    RCLCPP_WARN(logger, "x of quart : %2f", q.x());
    RCLCPP_WARN(logger, "y of quart : %2f", q.y());
    RCLCPP_WARN(logger, "z of quart : %2f", q.z());
    RCLCPP_WARN(logger, "w of quart : %2f", q.w());
    #endif

    if(!moveToPoint(target_pose)){
        #ifdef DEBUGGER
        RCLCPP_ERROR(logger, "first triangle failed!");
        #endif
    }else{
        #ifdef DEBUGGER
        RCLCPP_WARN(logger, "moving to the first triangle");
        #endif
        return 0;
    }

    /*=============================================================*/


    /*=====================START THE OPERATION=====================*/

    for(int i = 0; i < 3; i++){
        int nextTriangleIndex = vectorOfTriangles[currentTriangleIndex].myNeighbours[i];

        if((vectorOfTriangles[nextTriangleIndex].myNeighbours[i] != -1) 
            && //is there is a neighbour
            (vectorOfTriangles[nextTriangleIndex].unreachableCounter < 3) 
            && //and it is not unreachable
            (!vectorOfTriangles[nextTriangleIndex].traced)){ //and it was not traced

            traceThreeNeighbours(
                vectorOfTriangles, 
                currentTriangleIndex, 
                vectorOfTriangles[nextTriangleIndex], 
                vectorOfTriangles[currentTriangleIndex].triangleEdges[i]
            );
        }
    }

    /*=============================================================*/

    goHome();

    gripper_group_interface.reset();
    node.reset();

    rclcpp::shutdown();
    return 0;
}