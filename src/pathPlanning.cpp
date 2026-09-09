#include "trace_cube.hpp"
#include <iostream>

std::stack<Waypoint> pathHistory;
std::stack<Waypoint> stackOfReachableWaypoints;
extern std::vector<bool> traced;
std::vector<Triangle> plannedPath;

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
        -90.0 * M_PI / 180.0,
        -110 * M_PI / 180.0,
        -45.0 * M_PI / 180.0,
        -110.0 * M_PI / 180.0,
        -260.0 * M_PI / 180.0,
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

void getTCPpose(double* currentTCP)
{
    auto logger = rclcpp::get_logger("currentTCP");

    static auto tf_buffer =
        std::make_shared<tf2_ros::Buffer>(node->get_clock());

    static auto tf_listener =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    try
    {
        if (!tf_buffer->canTransform(
                "base_link",
                "tool0",
                tf2::TimePointZero,
                tf2::durationFromSec(1.0)))
        {
            RCLCPP_WARN(logger,
                        "Transform base_link -> tool0 unavailable");
            return;
        }

        auto transform = tf_buffer->lookupTransform(
            "base_link",
            "tool0",
            tf2::TimePointZero);

        currentTCP[0] = transform.transform.translation.x;
        currentTCP[1] = transform.transform.translation.y;
        currentTCP[2] = transform.transform.translation.z;
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR(logger, "TF error: %s", ex.what());
    }
}

void getTCPorientation(double* TCPorientation)
{
    auto logger = rclcpp::get_logger("getTCPorientation");

    static auto tf_buffer =
        std::make_shared<tf2_ros::Buffer>(node->get_clock());

    static auto tf_listener =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    try
    {
        if (!tf_buffer->canTransform(
                "base_link",
                "tool0",
                tf2::TimePointZero,
                tf2::durationFromSec(1.0)))
        {
            RCLCPP_WARN(logger,
                        "Transform base_link -> tool0 unavailable");
            return;
        }

        auto transform = tf_buffer->lookupTransform(
            "base_link",
            "tool0",
            tf2::TimePointZero);

        TCPorientation[0] = transform.transform.rotation.x;
        TCPorientation[1] = transform.transform.rotation.y;
        TCPorientation[2] = transform.transform.rotation.z;
        TCPorientation[3] = transform.transform.rotation.w;
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR(logger, "TF error: %s", ex.what());
    }
}

bool moveToPoint(geometry_msgs::msg::Pose target_pose, int triangleIndex, movementDirection movementDir, waypointType waypoint){
    auto logger = rclcpp::get_logger("moveToPoint");

    moveit_msgs::msg::RobotTrajectory trajectory;
    std::vector<geometry_msgs::msg::Pose> target_poses;

    target_poses.push_back(target_pose);
    double fraction = gripper_group_interface->computeCartesianPath(target_poses, 0.01, trajectory, true);

    //Full Cartesian path achieved
    if (fraction >= 0.9) {
        //PLANNING ONLY — the physical robot is NOT moved here.
        //Advance the virtual start state to the end of this segment so the
        //next planned segment chains from here instead of from the
        //stationary real robot pose. This is what lets the whole path be
        //planned before any motion happens.
        if(!trajectory.joint_trajectory.points.empty()){
            // 1. Get the shared pointer safely
            auto current_state_ptr = gripper_group_interface->getCurrentState();
            
            // 2. Check if the pointer is null before dereferencing
            if (!current_state_ptr) {
                return false; // Safely exit without segfaulting
            }

            // 3. If valid, proceed safely
            moveit::core::RobotState endState(*current_state_ptr);
            endState.setJointGroupPositions(
                gripper_group_interface->getName(),
                trajectory.joint_trajectory.points.back().positions
            );
            endState.update();
            gripper_group_interface->setStartState(endState);
        }

        if(movementDir == movementDirection::FORWARD){
            Waypoint newWaypoint = {target_pose, waypoint, triangleIndex, trajectory};
            pathHistory.push(newWaypoint);
            //dont use it as a counter, use .isTraced on the Triangle
            //also share any ideas you have regarding the "planning" part
            stackOfReachableWaypoints.push(newWaypoint);
            RCLCPP_ERROR(logger, "type of waypoint: %d",(int)pathHistory.top().typeOfWaypoint);
        }
        target_poses.pop_back();
        return true;
    }else {
        target_poses.pop_back();
        return false;
    }
}

geometry_msgs::msg::Pose targetPose(const Triangle &triangle){
    auto logger = rclcpp::get_logger("targetPose");

    geometry_msgs::msg::Pose target_pose;

    target_pose.position.x = - (triangle.centreOfTriangle[0] * 0.001f) - (triangle.normal_x * 0.05f);
    target_pose.position.y = - (triangle.centreOfTriangle[1] * 0.001f) + 0.65f - (triangle.normal_y * 0.05f);
    target_pose.position.z = (triangle.centreOfTriangle[2] * 0.001f) + (triangle.normal_z * 0.05f);

    tf2::Vector3 normal(
        - triangle.normal_x,
        - triangle.normal_y,
        triangle.normal_z
    );

    normal.normalize();

    tf2::Vector3 z_axis = - normal;  // Z into the surface
    z_axis.normalize();

    tf2::Vector3 world_up(0.0, 0.0, 1.0);
    if (std::abs(z_axis.dot(world_up)) > 0.99) {
        world_up = tf2::Vector3(1.0, 0.0, 0.0);
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
    double TCPorientation[4] = {0,0,0,0};
    getTCPorientation(TCPorientation);
    geometry_msgs::msg::Pose target_pose;

    //move to this triangle------------------------------------------------------------------------------------------------------------
    //always go through the shared edge centre before the next triangle's
    //centre — no direct center-to-center shortcut, regardless of the angle
    //between the two triangles' normals.
    target_pose.position.x = - (edgeToPrevTriangle.centreOfEdge[0] * 0.001f) - (triangleToTrace.normal_x * 0.05f);
    target_pose.position.y = - (edgeToPrevTriangle.centreOfEdge[1] * 0.001f) + 0.65f - (triangleToTrace.normal_y * 0.05f);
    target_pose.position.z = (edgeToPrevTriangle.centreOfEdge[2] * 0.001f) + (triangleToTrace.normal_z * 0.05f);
    //use the orientation of the old triangle to avoid collisions
    getTCPorientation(TCPorientation);
    target_pose.orientation.x = TCPorientation[0];
    target_pose.orientation.y = TCPorientation[1];
    target_pose.orientation.z = TCPorientation[2];
    target_pose.orientation.w = TCPorientation[3];

    //if the edge is unreachable, then the triangle's unreachability counter goes up and we have to try next neighbour
    if(moveToPoint(target_pose, 0, movementDirection::FORWARD, waypointType::EDGE) == false){
        triangleToTrace.unreachableCounter++;
        return AttemptToReach::FAILED;
    }else{
        //if we did go to an edge then we can try to go to the ceter of the next triangle
        if(moveToPoint(targetPose(triangleToTrace), triangleToTrace.myIndex) == false){
            //if the center is unreachable then we go back to "initial" triangle
            triangleToTrace.unreachableCounter++;
            //delete an edge waypoint
            pathHistory.pop();
            moveToPoint(targetPose(previousTriangle), 0, movementDirection::BACKWARDS);
            return AttemptToReach::FAILED;
        }else{
            triangleToTrace.traced = true;
            traced[triangleToTrace.myIndex] = true;
            return AttemptToReach::TRIANGLE_REACHED;
        }
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
            #ifndef DEBUGGER
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
            return vectorOfTriangles[validNeighbours[a]].centreOfTriangle[2] < 
                   vectorOfTriangles[validNeighbours[b]].centreOfTriangle[2];
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

int startOperation(std::vector<Triangle> &vectorOfTriangles, std::vector<bool> &traced, Triangle &currentTriangle){
    auto logger = rclcpp::get_logger("startOperation");

    int nextToTraceIndex = 0;
    auto result = triangleWithLeastNeighbours(vectorOfTriangles, traced, currentTriangle);

    std::vector<int> sortedNeighbours = result.first;
    std::vector<int> sortedEdges = result.second;

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
            if(sortedNeighbours[neighbourNumber] != currentTriangle.myIndex){
                neighbourReachAttempt = traceNeighbour(currentTriangle, 
                                                        vectorOfTriangles[sortedNeighbours[neighbourNumber]], 
                                                        currentTriangle.triangleEdges[sortedEdges[neighbourNumber]]);
            }
        }
        if(neighbourReachAttempt == AttemptToReach::TRIANGLE_REACHED){
            currentTriangle.traced = true;
            traced[currentTriangle.myIndex] = true;
            nextToTraceIndex = sortedNeighbours[neighbourNumber];
        }else if(faildeAttempts >= (int)sortedNeighbours.size() - 1){
            pathHistory.pop();
            if (pathHistory.size() == 0){
                return -1;
            }
            if(pathHistory.top().typeOfWaypoint == waypointType::EDGE){
                pathHistory.pop();
            }
            //go to the last successful triangle
            moveToPoint(pathHistory.top().pose, pathHistory.top().triangleIndex, movementDirection::BACKWARDS);
            nextToTraceIndex = pathHistory.top().triangleIndex;
        }
    }else{
        pathHistory.pop();
        if (pathHistory.size() == 0){
                return -1;
        }
        if(pathHistory.top().typeOfWaypoint == waypointType::EDGE){
            moveToPoint(pathHistory.top().pose, 0, movementDirection::BACKWARDS);
            pathHistory.pop();
            if (pathHistory.size() == 0){
                return -1;
            }
        }
        //go to the last successful triangle
        moveToPoint(pathHistory.top().pose, pathHistory.top().triangleIndex, movementDirection::BACKWARDS);
        nextToTraceIndex = pathHistory.top().triangleIndex;
        return nextToTraceIndex;
    }
    return nextToTraceIndex;
    }

std::vector<Waypoint> extractOrderedPath(std::stack<Waypoint> stackCopy){
    //pathHistory is a stack (most recent push on top); this walks a COPY of
    //it (the original pathHistory is left intact) and reverses it back into
    //the chronological order the waypoints were actually planned in
    std::stack<Waypoint> isolatedLocalStack = stackCopy;
    std::vector<Waypoint> reversedOrder;
    while(!isolatedLocalStack.empty()){
        reversedOrder.push_back(isolatedLocalStack.top());
        isolatedLocalStack.pop();
    }
    std::reverse(reversedOrder.begin(), reversedOrder.end());
    return reversedOrder;
}

float computeCoveragePercent(const std::vector<Triangle> &vectorOfTriangles, const std::vector<Triangle> &plannedPathVec){
    auto logger = rclcpp::get_logger("startOperation");
    if(vectorOfTriangles.empty()){
        #ifdef DEBUGGER
        RCLCPP_WARN(logger, "vectorOfTriangles.empty()");
        #endif
        return 0.0f;
    }
    #ifdef DEBUGGER
    RCLCPP_WARN(logger, "vectorOfTriangles NOT");
    #endif
    return 100.0f * (float)plannedPathVec.size() / (float)vectorOfTriangles.size();
}

bool confirmPathExecution(float coveragePercent){
    auto logger = rclcpp::get_logger("confirmPathExecution");
    RCLCPP_WARN(logger, "planned path covers %.1f%% of the surface triangles", coveragePercent);
    std::cout << "Execute this planned path? [Y/N]: " << std::flush;
    std::string response;
    std::getline(std::cin, response);
    return (!response.empty() && (response[0] == 'Y' || response[0] == 'y'));
}

void executePlannedPath(const std::vector<Waypoint> &orderedWaypoints){
    auto logger = rclcpp::get_logger("executePlannedPath");
    //pure replay: each trajectory was already computed and verified during
    //planning, so this loop performs no planning or decision-making at all
    for(const auto &wp : orderedWaypoints){
        moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
        cartesian_plan.trajectory = wp.trajectory;
        gripper_group_interface->execute(cartesian_plan);
        RCLCPP_WARN(logger, "executed waypoint for %f, %f, %f", wp.pose.position.x, wp.pose.position.y, wp.pose.position.z);
    }
}