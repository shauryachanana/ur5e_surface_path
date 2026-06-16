#include "trace_cube.hpp"

std::stack<Waypoint> pathHistory;
extern std::vector<bool> traced;

void init(){
    //set id from "Context" tab sed desired planning library (open motion plannin library)
    gripper_group_interface->setPlanningPipelineId("ompl");
    //decides which motion algorithm to use
    gripper_group_interface->setPlannerId("RRTConnectkConfigDefault");

    //~the longer thr better
    gripper_group_interface->setPlanningTime(5.0);
    //from 0 to 1
    gripper_group_interface->setMaxVelocityScalingFactor(1.0);
    //0 to 1, 0 for const velocity 
    gripper_group_interface->setMaxAccelerationScalingFactor(0.0);
}

void goHome(){
    auto logger = rclcpp::get_logger("goGome");
    RCLCPP_WARN(logger, "going home");

    //the initial position (right before the tracing)
    std::vector<double> preferred_joints = {
        -101.0 * M_PI / 180.0,
        -115.0 * M_PI / 180.0,
        -25.0 * M_PI / 180.0,
        -205.0 * M_PI / 180.0,
        -101.0 * M_PI / 180.0,
        -180.0 * M_PI / 180.0
    };

    //sets the joint val as a target but doesnt move yet
    gripper_group_interface->setJointValueTarget(preferred_joints);
    
    moveit::planning_interface::MoveGroupInterface::Plan home_plan;
  
    //for the obect created before, call plan which saves the trajectory to reach preferred_joints to home_plan address
    //not a straight line, arbitrary trajectory
    auto ok = static_cast<bool>(gripper_group_interface->plan(home_plan));

    //if it is reachable, we can and do move to a target joint position
    if(ok){
        gripper_group_interface->execute(home_plan);
        RCLCPP_WARN(logger, "initial position rached");
    } else{
        RCLCPP_ERROR(logger, "initial positioning failed");
    }
}

void getTCPpose(double* currentTCP){
    auto logger = rclcpp::get_logger("currentTCP");
    //take a position of tcp
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    rclcpp::sleep_for(std::chrono::seconds(1));

    geometry_msgs::msg::TransformStamped transform;
    transform = tf_buffer->lookupTransform(
        "base_link",
        "tool0",
        tf2::TimePointZero
    );

    currentTCP[0] = transform.transform.translation.x;
    currentTCP[1] = transform.transform.translation.y;
    currentTCP[2] = transform.transform.translation.z;

    #ifdef DEBUGGER
    RCLCPP_WARN(logger, "TCP: %f", currentTCP[0]);
    RCLCPP_WARN(logger, "TCP: %f", currentTCP[1]);
    RCLCPP_WARN(logger, "TCP: %f", currentTCP[2]);
    #endif
}

void getTCPorientation(double* TCPorientation){
    auto logger = rclcpp::get_logger("getTCPorientation");
    //take a position of tcp
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    rclcpp::sleep_for(std::chrono::seconds(1));

    geometry_msgs::msg::TransformStamped transform;
    transform = tf_buffer->lookupTransform(
        "base_link",
        "tool0",
        tf2::TimePointZero
    );

    TCPorientation[0] = transform.transform.rotation.x;
    TCPorientation[1] = transform.transform.rotation.y;
    TCPorientation[2] = transform.transform.rotation.z;
    TCPorientation[3] = transform.transform.rotation.w;
}

bool moveToPoint(geometry_msgs::msg::Pose target_pose, int triangleIndex, movementDirection movementDir, waypointType waypoint){
    auto logger = rclcpp::get_logger("moveToPoint");

    moveit_msgs::msg::RobotTrajectory trajectory;
    std::vector<geometry_msgs::msg::Pose> target_poses;

    target_poses.push_back(target_pose);
    double fraction = gripper_group_interface->computeCartesianPath(target_poses, 0.01, trajectory, true);

    //Full Cartesian path achieved
    if (fraction >= 0.9) {
        moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
        //so our new trajectory which was calculated in computeCartesianPath is now stored in new cartesian_plan
        cartesian_plan.trajectory = trajectory;
        //now we can and do 
        gripper_group_interface->execute(cartesian_plan);
        if(movementDir == movementDirection::FORWARD){
            Waypoint newWaypoint = {target_pose, waypoint, triangleIndex};
            pathHistory.push(newWaypoint);
        }
        target_poses.pop_back();
        return true;
    }else {
        target_poses.pop_back();
        RCLCPP_ERROR(logger, "Cartesian path only %.1f%% complete — collision likely blocked it!", fraction * 100.0);
        return false;
    }
}

geometry_msgs::msg::Pose targetPose(const Triangle &triangle){
    auto logger = rclcpp::get_logger("targetPose");

    geometry_msgs::msg::Pose target_pose;

    target_pose.position.x = triangle.centreOfTriangle[0] * 0.001f;
    target_pose.position.y = - triangle.centreOfTriangle[1] * 0.001f + 1.00f;
    target_pose.position.z = triangle.centreOfTriangle[2] * 0.001f;

    #ifndef DEBUGGER
    RCLCPP_WARN(logger, "x : %f", target_pose.position.x);
    RCLCPP_WARN(logger, "y : %f", target_pose.position.y);
    RCLCPP_WARN(logger, "z : %f", target_pose.position.z);
    #endif

    tf2::Vector3 normal(
        triangle.normal_x,
        triangle.normal_y,
        triangle.normal_z
    );

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

    return target_pose;
}

AttemptToReach traceNeighbour(
    Triangle& previousTriangle, 
    Triangle& triangleToTrace, 
    Edge& edgeToPrevTriangle
){
    auto logger = rclcpp::get_logger("traceThreeNeighbours");

    geometry_msgs::msg::Pose target_pose;
    #ifdef DEBUGGER
    RCLCPP_WARN(logger, "attemppt to go to: %d", triangleToTrace.myIndex);
    RCLCPP_WARN(logger, "x y z: %f, %f, %f", triangleToTrace.centreOfTriangle[0], 
                                                triangleToTrace.centreOfTriangle[1], 
                                                triangleToTrace.centreOfTriangle[2]
                                            );
    #endif

    //move to this triangle------------------------------------------------------------------------------------------------------------

    //if center to center failed
    if(moveToPoint(targetPose(triangleToTrace), triangleToTrace.myIndex) == false){
        #ifdef DEBUGGER
        RCLCPP_WARN(logger, "failed to go straight");
        #endif
        //try going to an edge
        target_pose.position.x = edgeToPrevTriangle.centreOfEdge[0] * 0.001f;
        target_pose.position.y = - edgeToPrevTriangle.centreOfEdge[1] * 0.001f + 1.00f;
        target_pose.position.z = edgeToPrevTriangle.centreOfEdge[2] * 0.001f;
        //use the orientation of the old triangle to avoid collisions
        double TCPorientation[4] = {0,0,0,0};
        getTCPorientation(TCPorientation);
        target_pose.orientation.x = TCPorientation[0];
        target_pose.orientation.y = TCPorientation[1];
        target_pose.orientation.z = TCPorientation[2];
        target_pose.orientation.w = TCPorientation[3];
        #ifdef DEBUGGER
        RCLCPP_WARN(logger, "x y z of edge: %f, %f, %f", target_pose.position.x, target_pose.position.y, target_pose.position.z);
        RCLCPP_WARN(logger, "orientation of edge: %f, %f, %f, %f", target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);
        #endif
        //if even edge is unreachable, then the triangles unreachability counter goes up and we have to try next neighbour
        if(moveToPoint(target_pose, 0, movementDirection::FORWARD, waypointType::EDGE) == false){
            #ifdef DEBUGGER
            RCLCPP_WARN(logger, "edge failed too");
            #endif
            triangleToTrace.unreachableCounter++;
            return AttemptToReach::FAILED;
        }else{
            //if we did go to an edge then we can try to go to the ceter of the next triangle
            if(moveToPoint(targetPose(triangleToTrace), triangleToTrace.myIndex) == false){
                #ifdef DEBUGGER
                RCLCPP_WARN(logger, "edge to center failed");
                RCLCPP_WARN(logger, "x y z of triangle: %f, %f, %f", target_pose.position.x, target_pose.position.y, target_pose.position.z);
                #endif
                //if the center is unreachable then we go back to "initial" triangle
                triangleToTrace.unreachableCounter++;
                //delete an edge waypoint
                pathHistory.pop();
                moveToPoint(targetPose(previousTriangle), 0, movementDirection::BACKWARDS);
                return AttemptToReach::FAILED;
            }else{
                #ifdef DEBUGGER
                RCLCPP_WARN(logger, "edge to center success");
                #endif
                triangleToTrace.traced = true;
                traced[triangleToTrace.myIndex] = true;
                return AttemptToReach::TRIANGLE_REACHED;
            }
        }
    }else{
        #ifdef DEBUGGER
        RCLCPP_WARN(logger, "straight success");
        #endif
        triangleToTrace.traced = true;
        traced[triangleToTrace.myIndex] = true;
        return AttemptToReach::TRIANGLE_REACHED;
    }
}

AttemptToReach attemptToReachNextClosest(std::vector<Triangle> vectorOfDesiredTriangles, int &closestTriangleIndex){
    auto logger = rclcpp::get_logger("attemptToReachNextClosest");

    double currentTCP[3] = {0, 0, 0};
    getTCPpose(currentTCP);
    int closestTriangle = 0;
    std::size_t size = vectorOfDesiredTriangles.size();

    for(std::size_t i = 0; i < size; i++){
        //look for the closest in a given vector
        closestTriangle = getClosestTriangle(vectorOfDesiredTriangles, currentTCP);

        #ifdef DEBUGGER
        RCLCPP_WARN(logger, "attempt to reach index %d", vectorOfDesiredTriangles[closestTriangle].myIndex);
        RCLCPP_WARN(logger, "x y z: %f, %f, %f", vectorOfDesiredTriangles[closestTriangle].centreOfTriangle[0], 
                                                vectorOfDesiredTriangles[closestTriangle].centreOfTriangle[1], 
                                                vectorOfDesiredTriangles[closestTriangle].centreOfTriangle[2]
                                            );
        #endif

        if(moveToPoint(targetPose(vectorOfDesiredTriangles[closestTriangle]), vectorOfDesiredTriangles[closestTriangle].myIndex)){
            //if the closest triangle was reached - exit the function
            closestTriangleIndex = vectorOfDesiredTriangles[closestTriangle].myIndex;
            return AttemptToReach::TRIANGLE_REACHED;
        }else{
            //increase a counter for unreachability
            vectorOfDesiredTriangles[closestTriangle].unreachableCounter++;
            //otherwise - update a vector and run getClosest again
            vectorOfDesiredTriangles.erase(vectorOfDesiredTriangles.begin() + closestTriangle);
        }
    }
    return AttemptToReach::EMPTY_VECTOR;
}

std::pair<std::vector<int>, std::vector<int>> triangleWithLeastNeighbours(std::vector<Triangle> &vectorOfTriangles, std::vector<bool> &traced, Triangle triangleToTrace){
    auto logger = rclcpp::get_logger("triangleWithLeastNeighbours");

    double currentTCP[3] = {0, 0, 0};
    getTCPpose(currentTCP);

    std::vector<int> validNeighbours;
    std::vector<int> edgeIndices;

    for(std::size_t i = 0; i < triangleToTrace.myNeighbours.size(); i++){
        int neighbourIndex = triangleToTrace.myNeighbours[i];
        //if the value stored in an array is a valid one and if the triangles stored was not traced
        if((neighbourIndex != -1) && (vectorOfTriangles[neighbourIndex].traced == false) && (traced[neighbourIndex] == false)){
            //valuyes from an array
            validNeighbours.push_back(neighbourIndex);
            //the position of this value
            edgeIndices.push_back(i);
            #ifdef DEBUGGER
            RCLCPP_WARN(logger, "stored triangle index: %d", neighbourIndex);
            #endif
        }
    }

    //helper vector
    std::vector<int> order(validNeighbours.size());
    std::iota(order.begin(), order.end(), 0);

    std::sort(order.begin(), order.end(), [&](int a, int b){
        int countA = vectorOfTriangles[validNeighbours[a]].getValidNeighbours(traced, vectorOfTriangles);
        int countB = vectorOfTriangles[validNeighbours[b]].getValidNeighbours(traced, vectorOfTriangles);
        if(countA != countB){
            return countA < countB;
        }else{
            return distanceToTCP(vectorOfTriangles[validNeighbours[a]], currentTCP) < 
                   distanceToTCP(vectorOfTriangles[validNeighbours[b]], currentTCP);
        }
    });

    //reorder both vectors according to sort result
    std::vector<int> sortedNeighbours;
    std::vector<int> sortedEdgeIndices;
    for(int i : order){
        sortedNeighbours.push_back(validNeighbours[i]);
        sortedEdgeIndices.push_back(edgeIndices[i]);
    }

    return {sortedNeighbours, sortedEdgeIndices};
}

int startOperation(std::vector<Triangle> vectorOfTriangles, std::vector<bool> &traced, Triangle &currentTriangle){
    auto logger = rclcpp::get_logger("startOperation");

    #ifdef DEBUGGER
    RCLCPP_WARN(logger, "im here: %d", currentTriangle.myIndex);
    #endif

    int nextToTraceIndex = 0;
    auto result = triangleWithLeastNeighbours(vectorOfTriangles, traced, currentTriangle);

    std::vector<int> sortedNeighbours = result.first;
    std::vector<int> sortedEdges = result.second;

    #ifdef DEBUGGER
    for(int i = 0; i<(int)sortedNeighbours.size(); i++){
        RCLCPP_WARN(logger, "sortedNeighbours size: %d", sortedNeighbours[i]);
    }
    #endif

    int neighbourNumber = 0;
    int faildeAttempts = 0;
    AttemptToReach neighbourReachAttempt;
    if(!sortedNeighbours.empty() && !sortedEdges.empty()){
        neighbourReachAttempt = traceNeighbour(currentTriangle, 
                                                vectorOfTriangles[sortedNeighbours[neighbourNumber]], 
                                                currentTriangle.triangleEdges[sortedEdges[neighbourNumber]]);

        while((neighbourReachAttempt != AttemptToReach::TRIANGLE_REACHED) && 
                (faildeAttempts < (int)sortedNeighbours.size() - 1)){
            neighbourNumber++;
            faildeAttempts++;
            #ifdef DEBUGGER
            RCLCPP_WARN(logger, "goint for : %d", sortedNeighbours[neighbourNumber]);
            #endif
            if(sortedNeighbours[neighbourNumber] != currentTriangle.myIndex){
                neighbourReachAttempt = traceNeighbour(currentTriangle, 
                                                        vectorOfTriangles[sortedNeighbours[neighbourNumber]], 
                                                        currentTriangle.triangleEdges[sortedEdges[neighbourNumber]]);
            }
        }
        if(neighbourReachAttempt == AttemptToReach::TRIANGLE_REACHED){
            #ifdef DEBUGGER
            RCLCPP_WARN(logger, "next index: %d", sortedNeighbours[neighbourNumber]);
            #endif
            currentTriangle.traced = true;
            traced[currentTriangle.myIndex] = true;
            nextToTraceIndex = sortedNeighbours[neighbourNumber];
        }else if(faildeAttempts >= (int)sortedNeighbours.size() - 1){
            #ifdef DEBUGGER
            RCLCPP_WARN(logger, "too many attempts");
            #endif
            pathHistory.pop();
            if(pathHistory.top().typeOfWaypoint == waypointType::EDGE){
                pathHistory.pop();
            }
            //go to the last successful triangle
            moveToPoint(pathHistory.top().pose, pathHistory.top().triangleIndex, movementDirection::BACKWARDS);
            nextToTraceIndex = pathHistory.top().triangleIndex;
        }
    }else{
        #ifdef DEBUGGER
        RCLCPP_WARN(logger, "empty vectors");
        #endif
        pathHistory.pop();
        if(pathHistory.top().typeOfWaypoint == waypointType::EDGE){
            pathHistory.pop();
        }
        //go to the last successful triangle
        moveToPoint(pathHistory.top().pose, pathHistory.top().triangleIndex, movementDirection::BACKWARDS);
        nextToTraceIndex = pathHistory.top().triangleIndex;
        return nextToTraceIndex;
    }
    return nextToTraceIndex;
    }