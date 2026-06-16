#include "trace_cube.hpp"

//global node
std::shared_ptr<rclcpp::Node> node;
std::unique_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_interface;

//load a file
std::string mesh_path = "file://" + ament_index_cpp::get_package_share_directory("ur5e_surface_path") + "/meshes/50cmCube.stl";
//now we have info on all the triangles
shapes::Mesh* mesh = shapes::createMeshFromResource(mesh_path);

extern std::stack<Waypoint> pathHistory;
std::vector<bool> traced;

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
    traced = std::vector<bool>(vectorOfTriangles.size(), false);

    #ifndef DEBUGGER
    for(int i = 0; i < (int)vectorOfTriangles.size(); i++){
        RCLCPP_WARN(logger, "neighbours of %d : %d, %d, %d", i, vectorOfTriangles[i].myNeighbours[0],vectorOfTriangles[i].myNeighbours[1],vectorOfTriangles[i].myNeighbours[2]);
    }
    #endif
    #ifdef POINTCLOUDS
    //create a point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
w   s   
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

    for(std::size_t i = 0; i < vectorOfTriangles.size(); i++){
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

    /*=============================================================*/


    /*=====================START THE OPERATION=====================*/

    int nextOne = startOperation(vectorOfTriangles, traced, vectorOfTriangles[closestTriangleIndex]);
    while(nextOne != -1){
        nextOne = startOperation(vectorOfTriangles, traced, vectorOfTriangles[nextOne]);
    }

    /*=============================================================*/

    goHome();

    gripper_group_interface.reset();
    node.reset();

    rclcpp::shutdown();
    return 0;
}