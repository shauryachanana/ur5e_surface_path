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

    std::vector<Triangle> vectorOfTriangles;

    triangleExtraction(vectorOfTriangles);
    std::vector<bool> traced = std::vector<bool>(vectorOfTriangles.size(), false);

    #ifdef POINTCLOUDS
    //create a point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    //now we have all the triangle centres and theor normals stored
    //centers go to the point cloud obj for easy nearest neighbour calculation
    //vectorOfTriangles is used to store the normal vectors of the triangles to orient the TCP
    triangleExtraction(vectorOfTriangles, cloud);
    std::vector<bool> traced = std::vector<int>(vectorOfTriangles.size(), false);

    pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;

    //create kdTree
    kdTree.setInputCloud(cloud);

    //in this vector we will get an index of the nearest neighbour
    std::vector<int> pointIdxKNNSearch(1);
    //in this vector we will get a distance to the nearest neighbour
    std::vector<float> pointKNNSquaredDistance(1);
    #endif

    /*=============================================================*/

    moveit_msgs::msg::RobotTrajectory trajectory;

    //create a vector that contains the set of "waypoints"
    std::vector<geometry_msgs::msg::Pose> target_poses;

    //msg type to set a pose
    geometry_msgs::msg::Pose target_pose;

    /*========FIND TRIANGLE WITH THE LEAST AMOUNT OF EDGES=========*/

    //two arrays to store min neighbour triangles
    std::vector<Triangle> singleNeighbpurTrangles;
    std::vector<Triangle> doubleNeighbourTriangles;
    int chosenVector = 0;

    for(unsigned int i = 0; i < vectorOfTriangles.size(); i++){
        if(vectorOfTriangles[i].getValidNeighbours(traced, vectorOfTriangles) == 1){
            //use one if a truiangle only has one neighbour (prefferable)
            singleNeighbpurTrangles.push_back(vectorOfTriangles[i]);
        }else if(vectorOfTriangles[i].getValidNeighbours(traced, vectorOfTriangles) == 2){
            //for 2 neighbours
            doubleNeighbourTriangles.push_back(vectorOfTriangles[i]);
        }
        //if there are no < 2 neighbours triangles we use vectorOfTriangles
    }
    
    //depending on how full the arrays are use them to determine the initial one
    if(singleNeighbpurTrangles.size() != 0){
        //if there are triangles with a single neighbour
        chosenVector = 0;
    }else if(doubleNeighbourTriangles.size() != 0){
        //if there are no triangles with a single neighbour but with two neighbours
        chosenVector = 1;
    }else{
        //if there are only triangles with 3 neighbpurs:
        chosenVector = 2;
    }

    /*=============================================================*/

    /*=================GET TO THE CLOSEST TRIANGLE=================*/

    #ifndef POINTCLOUDS
    
    AttemptToReach initialTriangle = AttemptToReach::EMPTY_VECTOR;
    int closestTriangleIndex = 0;
    while((initialTriangle != AttemptToReach::TRIANGLE_REACHED) && (chosenVector < 3)){
        switch(chosenVector){
            case 0:
            #ifdef DEBUGGER
            RCLCPP_WARN(logger, "attempt in singleNeighbpurTrangles");
            #endif
            initialTriangle = attemptToReachNextClosest(singleNeighbpurTrangles, closestTriangleIndex);
            chosenVector++;
            break;

            case 1:
            #ifdef DEBUGGER
            RCLCPP_WARN(logger, "attempt in doubleNeighbourTriangles");
            #endif
            initialTriangle = attemptToReachNextClosest(doubleNeighbourTriangles, closestTriangleIndex);
            chosenVector++;
            break;

            case 2:
            #ifdef DEBUGGER
            RCLCPP_WARN(logger, "attempt in vectorOfTriangles");
            #endif
            initialTriangle = attemptToReachNextClosest(vectorOfTriangles, closestTriangleIndex);
            chosenVector++;
            break;
        }
    }
    if((initialTriangle != AttemptToReach::TRIANGLE_REACHED) && (chosenVector > 3)){
        #ifdef DEBUGGER
        RCLCPP_WARN(logger, "there are no reachable triangles!");
        #endif
    }else if(initialTriangle == AttemptToReach::TRIANGLE_REACHED){
        vectorOfTriangles[closestTriangleIndex].traced = true;
        traced[closestTriangleIndex] = true;
        #ifdef DEBUGGER
        RCLCPP_WARN(logger, "reached initial triangle, %d!", closestTriangleIndex);
        #endif
    }

    #endif

    #ifdef POINTCLOUDS
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

    #endif

    /*=============================================================*/


    /*=====================START THE OPERATION=====================*/

    int neighboursCount  = INT_MAX;
    int bestCandidateIndex = -1;
    double currentTCP[3] = {0, 0, 0};
    getTCPpose(currentTCP);
    for(std::size_t i = 0; i < vectorOfTriangles[closestTriangleIndex].myNeighbours.size(); i++){
        int neighbourIndex = vectorOfTriangles[closestTriangleIndex].myNeighbours[i];
        if((neighbourIndex != -1) && (!vectorOfTriangles[neighbourIndex].traced)){
            //check for the amount of availiable neighbours
            int count = vectorOfTriangles[neighbourIndex].getValidNeighbours(traced, vectorOfTriangles);
            if(count < neighboursCount){
                neighboursCount = count;
                bestCandidateIndex = neighbourIndex;
            }else if(count == neighboursCount){
                float distance01 = distanceToTCP(vectorOfTriangles[neighbourIndex], currentTCP);
                float distance02 = distanceToTCP(vectorOfTriangles[bestCandidateIndex], currentTCP);

                if(distance01 < distance02){
                    bestCandidateIndex = neighbourIndex;
                }
            }
        }
    }
    #ifdef DEBUGGER
    RCLCPP_WARN(logger, "index of next triangle: %d", bestCandidateIndex);
    #endif

    /*=============================================================*/

    goHome();

    gripper_group_interface.reset();
    node.reset();

    rclcpp::shutdown();
    return 0;
}