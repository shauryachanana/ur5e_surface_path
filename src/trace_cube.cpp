#include "trace_cube.hpp"

//global node
std::shared_ptr<rclcpp::Node> node;
std::unique_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_interface;

//load a file
std::string mesh_path = "file://" + ament_index_cpp::get_package_share_directory("ur5e_surface_path") + "/meshes/50cmCube.stl";
//now we have info on all the triangles
shapes::Mesh* mesh = shapes::createMeshFromResource(mesh_path);

extern std::stack<Waypoint> pathHistory;
extern std::stack<Waypoint> stackOfReachableWaypoints;
std::vector<bool> traced;

int main(int argc, char** argv){
    rclcpp::init(argc, argv);

    // initialize them here after rclcpp::init()
    node = std::make_shared<rclcpp::Node>(
        "trace_cube",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // 1. Create a background thread executor explicitly dedicated to handling ROS messages
    auto spinner = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    spinner->add_node(node);
    
    // 2. Start spinning in the background immediately.
    // This handles /joint_states concurrently while main() continues executing down.
    std::thread spinner_thread([spinner]() { spinner->spin(); });

    // 3. Wait a moment for the background spinner to capture initial clock and joint frames
    rclcpp::sleep_for(std::chrono::seconds(2));

    gripper_group_interface = std::make_unique<moveit::planning_interface::MoveGroupInterface>(node, "ur_manipulator");

    init();

    goHome();

    auto logger = rclcpp::get_logger("main");
    
    /*===================FIND THE POINT TO START===================*/

    std::vector<Triangle> vectorOfTriangles;

    triangleExtraction(vectorOfTriangles);
    traced = std::vector<bool>(vectorOfTriangles.size(), false);

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

    #ifdef DEBUGGER
    RCLCPP_WARN(logger, "START OPERATION");
    #endif
    int nextOne = startOperation(vectorOfTriangles, traced, vectorOfTriangles[closestTriangleIndex]);
    int a = 0;
    while(nextOne != -1 && a<15){
        nextOne = startOperation(vectorOfTriangles, traced, vectorOfTriangles[nextOne]);
    }

    /*=============================================================*/

    /*===================PLAN COMPLETE — REVIEW & EXECUTE===========*/

    //planning is done and the physical robot has not moved at all yet.
    //drop the virtual chained start state so any further real planning
    //(the execution replay below, and goHome()) starts from the robot's
    //actual, unmoved current position instead of the last virtual pose.
    gripper_group_interface->setStartStateToCurrentState();
    #ifdef DEBUGGER
    RCLCPP_WARN(logger, "setStartStateToCurrentState passed");
    #endif

    //flatten the planning history (a stack) back into the order it was
    //produced in, and pull out the vector<Triangle> that represents the
    //complete, fixed sequence of triangles the robot will visit
    std::vector<Waypoint> orderedWaypoints = extractOrderedPath(stackOfReachableWaypoints);
    #ifdef DEBUGGER
    RCLCPP_WARN(logger, "extractOrderedPath passed");
    #endif
    plannedPath.clear();
    for(const auto &wp : orderedWaypoints){
        plannedPath.push_back(vectorOfTriangles[wp.triangleIndex]);
        #ifdef DEBUGGER
        RCLCPP_WARN(logger, "added waypoint");
        #endif
    }

    float coveragePercent = computeCoveragePercent(vectorOfTriangles, plannedPath);

    if(confirmPathExecution(coveragePercent)){
        //execute the already-planned, already-verified trajectories in
        //order; no re-planning or re-deciding of the sequence happens here
        #ifdef DEBUGGER
        RCLCPP_WARN(logger, "execution start");
        #endif
        executePlannedPath(orderedWaypoints);
    }else{
        RCLCPP_WARN(logger, "path execution cancelled by user");
    }

    /*=============================================================*/

    goHome();

    gripper_group_interface.reset();

    if (rclcpp::ok()) {
        rclcpp::shutdown();
        if (spinner_thread.joinable()) {
            spinner_thread.join();
        }
    }

    node.reset();

    return 0;
    
}