#include "trace_cube.hpp"
#include <iostream>

std::stack<Waypoint> pathHistory;
std::stack<Waypoint> stackOfReachableWaypoints;
extern std::vector<bool> traced;
std::vector<Triangle> plannedPath;

// Distance from tool0 to the pen tip along tool0's local +Z axis.
// Positions in this file are in meters: 0.05 = 50 mm, 0.01 = 10 mm.
constexpr double PEN_LENGTH = 0.05;
constexpr int PEN_ROTATION_STEPS = 50;

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
    gripper_group_interface->setMaxAccelerationScalingFactor(1.0);
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

bool getTCPpose(double* currentTCP)
{
    auto logger = rclcpp::get_logger("currentTCP");

    try
    {
        if (!tf_buffer->canTransform(
                "base_link",
                "tool0",
                tf2::TimePointZero,
                tf2::durationFromSec(1.0)))
        {
            RCLCPP_ERROR(
                logger,
                "Transform base_link -> tool0 unavailable"
            );

            return false;
        }

        auto transform = tf_buffer->lookupTransform(
            "base_link",
            "tool0",
            tf2::TimePointZero
        );

        currentTCP[0] = transform.transform.translation.x;
        currentTCP[1] = transform.transform.translation.y;
        currentTCP[2] = transform.transform.translation.z;

        return true;
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR(
            logger,
            "TF error: %s",
            ex.what()
        );

        return false;
    }
}

void getTCPorientation(double* TCPorientation)
{
    auto logger = rclcpp::get_logger("getTCPorientation");

    try
    {
        if (!tf_buffer->canTransform(
                "base_link",
                "tool0",
                tf2::TimePointZero,
                tf2::durationFromSec(1.0)))
        {
            RCLCPP_WARN(
                logger,
                "Transform base_link -> tool0 unavailable"
            );

            return;
        }

        auto transform = tf_buffer->lookupTransform(
            "base_link",
            "tool0",
            tf2::TimePointZero
        );

        TCPorientation[0] = transform.transform.rotation.x;
        TCPorientation[1] = transform.transform.rotation.y;
        TCPorientation[2] = transform.transform.rotation.z;
        TCPorientation[3] = transform.transform.rotation.w;
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR(
            logger,
            "TF error: %s",
            ex.what()
        );
    }
}

bool moveToPoint(geometry_msgs::msg::Pose target_pose, int triangleIndex, movementDirection movementDir, waypointType waypoint){
    auto logger = rclcpp::get_logger("moveToPoint");

    moveit_msgs::msg::RobotTrajectory trajectory;
    std::vector<geometry_msgs::msg::Pose> target_poses;

    //RCLCPP_ERROR(logger, "MOVE TARGET ORIENTATION: x=%f y=%f z=%f w=%f",target_pose.orientation.x,target_pose.orientation.y,target_pose.orientation.z,target_pose.orientation.w);

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

        Waypoint newWaypoint = {target_pose, waypoint, triangleIndex, trajectory};
        stackOfReachableWaypoints.push(newWaypoint);

        if(movementDir == movementDirection::FORWARD){
            pathHistory.push(newWaypoint);
            //dont use it as a counter, use .isTraced on the Triangle
            //also share any ideas you have regarding the "planning" part
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

    target_pose.position.x = - (triangle.centreOfTriangle[0] * 0.001f) - (triangle.normal_x * PEN_LENGTH);
    target_pose.position.y = - (triangle.centreOfTriangle[1] * 0.001f) + 0.65f - (triangle.normal_y * PEN_LENGTH);
    target_pose.position.z = (triangle.centreOfTriangle[2] * 0.001f) + 0.2f + (triangle.normal_z * PEN_LENGTH);

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


bool moveToPointWithPenRotation(
    const geometry_msgs::msg::Pose &edgePose,
    const geometry_msgs::msg::Quaternion &nextOrientation,
    const geometry_msgs::msg::Point &penTip
){
    auto logger = rclcpp::get_logger("moveToPointWithPenRotation");

    moveit_msgs::msg::RobotTrajectory trajectory;
    std::vector<geometry_msgs::msg::Pose> target_poses;

    tf2::Quaternion startQ(
        edgePose.orientation.x,
        edgePose.orientation.y,
        edgePose.orientation.z,
        edgePose.orientation.w
    );
    tf2::Quaternion endQ(
        nextOrientation.x,
        nextOrientation.y,
        nextOrientation.z,
        nextOrientation.w
    );
    startQ.normalize();
    endQ.normalize();

    // Quaternions q and -q represent the same orientation. Keep the
    // interpolation on the shorter rotational path.
    if (startQ.dot(endQ) < 0.0) {
        endQ = tf2::Quaternion(-endQ.x(), -endQ.y(), -endQ.z(), -endQ.w());
    }

    // The edgePose is already the first point of this operation.
    // From there, rotate around the pen tip while compensating tool0's
    // position so the pen tip stays at exactly the same point.
    for(int i = 1; i <= PEN_ROTATION_STEPS; i++){
        double t = (double)i / (double)PEN_ROTATION_STEPS;
        tf2::Quaternion q = startQ.slerp(endQ, t);
        q.normalize();

        tf2::Vector3 penOffset = tf2::Matrix3x3(q) * tf2::Vector3(0.0, 0.0, PEN_LENGTH);

        geometry_msgs::msg::Pose rotationPose;
        rotationPose.position.x = penTip.x - penOffset.x();
        rotationPose.position.y = penTip.y - penOffset.y();
        rotationPose.position.z = penTip.z - penOffset.z();

        rotationPose.orientation.x = q.x();
        rotationPose.orientation.y = q.y();
        rotationPose.orientation.z = q.z();
        rotationPose.orientation.w = q.w();

        target_poses.push_back(rotationPose);
    }

    double fraction = gripper_group_interface->computeCartesianPath(
        target_poses, 0.01, trajectory, true
    );

    if(fraction >= 0.9){
        if(!trajectory.joint_trajectory.points.empty()){
            auto current_state_ptr = gripper_group_interface->getCurrentState();
            if(!current_state_ptr){
                return false;
            }

            moveit::core::RobotState endState(*current_state_ptr);
            endState.setJointGroupPositions(
                gripper_group_interface->getName(),
                trajectory.joint_trajectory.points.back().positions
            );
            endState.update();
            gripper_group_interface->setStartState(endState);
        }

        // This rotation is part of the edge transition, so keep it in the
        // same edge waypoint that was already created by moveToPoint().
        if(!stackOfReachableWaypoints.empty() &&
        !stackOfReachableWaypoints.top().trajectory.joint_trajectory.points.empty()){
            auto &edgeTrajectory = stackOfReachableWaypoints.top().trajectory.joint_trajectory;
            rclcpp::Duration timeOffset(edgeTrajectory.points.back().time_from_start);

            // Check whether the end of the move-to-edge trajectory
            // matches the beginning of the rotation trajectory.
            if(!trajectory.joint_trajectory.points.empty())
            {
                const auto &edgeLast =
                    edgeTrajectory.points.back().positions;

                const auto &rotationFirst =
                    trajectory.joint_trajectory.points.front().positions;

                if(edgeLast.size() == rotationFirst.size())
                {
                    double maxBoundaryError = 0.0;

                    for(std::size_t j = 0; j < edgeLast.size(); ++j)
                    {
                        maxBoundaryError = std::max(
                            maxBoundaryError,
                            std::abs(
                                edgeLast[j] -
                                rotationFirst[j]
                            )
                        );
                    }

                    RCLCPP_WARN(
                        logger,
                        "EDGE -> ROTATION boundary error = %.8f rad",
                        maxBoundaryError
                    );
                }
            }



            bool firstRotationPoint = true;
            for(auto rotationPoint : trajectory.joint_trajectory.points){
                if(firstRotationPoint){
                    firstRotationPoint = false;
                    continue;
                }

                rclcpp::Duration rotationTime(rotationPoint.time_from_start);
                rclcpp::Duration totalTime = timeOffset + rotationTime;

                rotationPoint.time_from_start.sec =
                    static_cast<int32_t>(totalTime.nanoseconds() / 1000000000LL);

                rotationPoint.time_from_start.nanosec =
                    static_cast<uint32_t>(totalTime.nanoseconds() % 1000000000LL);

                edgeTrajectory.points.push_back(rotationPoint);
            }

            // stackOfReachableWaypoints.top().pose = target_poses.back();

            if(!pathHistory.empty() && pathHistory.top().typeOfWaypoint == waypointType::EDGE){
                pathHistory.top().trajectory = stackOfReachableWaypoints.top().trajectory;
            }
        }

        RCLCPP_ERROR(logger, "pen rotation at edge planned successfully");
        return true;
    }

    RCLCPP_ERROR(logger, "pen rotation at edge failed, fraction=%f", fraction);

    // moveToPoint() already added the edge waypoint before this rotation
    // was attempted. The rotation is part of that same waypoint, so if the
    // rotation fails, that edge waypoint must be removed again. Otherwise
    // an unreachable/invalid waypoint remains in the planned path.
    // if(!stackOfReachableWaypoints.empty() &&
    //    stackOfReachableWaypoints.top().typeOfWaypoint == waypointType::EDGE){
    //     stackOfReachableWaypoints.pop();
    // }

    return false;
}


AttemptToReach traceNeighbour(
    Triangle& previousTriangle, 
    Triangle& triangleToTrace, 
    Edge& edgeToPrevTriangle
){
    auto logger = rclcpp::get_logger("traceThreeNeighbours");
    geometry_msgs::msg::Pose previousPose = targetPose(previousTriangle);
    geometry_msgs::msg::Pose target_pose;

    //move to this triangle------------------------------------------------------------------------------------------------------------
    //try going to an edge
    target_pose.position.x = - (edgeToPrevTriangle.centreOfEdge[0] * 0.001f) - (previousTriangle.normal_x * PEN_LENGTH);
    target_pose.position.y = - (edgeToPrevTriangle.centreOfEdge[1] * 0.001f) + 0.65f - (previousTriangle.normal_y * PEN_LENGTH);
    target_pose.position.z = (edgeToPrevTriangle.centreOfEdge[2] * 0.001f) + 0.2f + (previousTriangle.normal_z * PEN_LENGTH);
    //use the orientation of the old triangle to avoid collisions
    target_pose.orientation.x = previousPose.orientation.x;
    target_pose.orientation.y = previousPose.orientation.y;
    target_pose.orientation.z = previousPose.orientation.z;
    target_pose.orientation.w = previousPose.orientation.w;
    //if even edge is unreachable, then the triangles unreachability counter goes up and we have to try next neighbour
    if(moveToPoint(target_pose, 0, movementDirection::FORWARD, waypointType::EDGE) == false){
        #ifdef DEBUGGER
        RCLCPP_WARN(logger, "edge failed too");
        #endif
        triangleToTrace.unreachableCounter++;
        return AttemptToReach::FAILED;
    }else{
        //At the edge, keep the pen tip fixed and rotate the pen so its +Z
        //axis becomes perpendicular to the next triangle.
        geometry_msgs::msg::Pose nextPose = targetPose(triangleToTrace);
        geometry_msgs::msg::Point edgePenTip;
        edgePenTip.x = - (edgeToPrevTriangle.centreOfEdge[0] * 0.001f);
        edgePenTip.y = - (edgeToPrevTriangle.centreOfEdge[1] * 0.001f) + 0.65f;
        edgePenTip.z = (edgeToPrevTriangle.centreOfEdge[2] * 0.001f) + 0.2f;

        if(!moveToPointWithPenRotation(
        target_pose,
        nextPose.orientation,
        edgePenTip))
{
    RCLCPP_WARN(
        logger,
        "Rotation failed. Keeping edge visit and planning return."
    );

    triangleToTrace.unreachableCounter++;

    // Remove EDGE only from algorithmic path history.
    // We didn't successfully cross into the neighbour.
    if(!pathHistory.empty() &&
       pathHistory.top().typeOfWaypoint == waypointType::EDGE)
    {
        pathHistory.pop();
    }

    // IMPORTANT:
    // The EDGE trajectory remains in stackOfReachableWaypoints.
    //
    // The virtual MoveIt state is currently at the edge because
    // moveToPoint(edge) succeeded, while the failed rotation did
    // not advance the virtual state.
    //
    // Therefore plan a return from edge -> previous triangle.
    if(!moveToPoint(
            targetPose(previousTriangle),
            previousTriangle.myIndex,
            movementDirection::BACKWARDS))
    {
        RCLCPP_ERROR(
            logger,
            "Failed to plan return from edge to previous triangle"
        );
    }

    return AttemptToReach::FAILED;
}

        //The pen is now perpendicular to the next triangle at the same edge point.
        //Continue from the edge to the center of the next triangle.
        if(moveToPoint(nextPose, triangleToTrace.myIndex) == false){
            #ifdef DEBUGGER
            RCLCPP_WARN(logger, "edge to center failed");
            RCLCPP_WARN(logger, "x y z of triangle: %f, %f, %f", target_pose.position.x, target_pose.position.y, target_pose.position.z);
            #endif
            //if the center is unreachable then we go back to "initial" triangle
            triangleToTrace.unreachableCounter++;
            // The center move FAILED, so no triangle waypoint was added
            if(!pathHistory.empty() &&
            pathHistory.top().typeOfWaypoint == waypointType::EDGE){
                pathHistory.pop();
            }

            // Return to the triangle from which this neighbour was attempted.
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

}

AttemptToReach attemptToReachNextClosest(std::vector<Triangle> vectorOfDesiredTriangles, int &closestTriangleIndex){
    auto logger = rclcpp::get_logger("attemptToReachNextClosest");

    double currentTCP[3];

    if (!getTCPpose(currentTCP)) {
        RCLCPP_ERROR(
            logger,
            "Cannot determine TCP position. Aborting triangle selection."
        );

        return AttemptToReach::EMPTY_VECTOR;
    }
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

    // #ifdef DEBUGGER
    // RCLCPP_WARN(logger, "checking if my build is working");
    // #endif

    int nextToTraceIndex = 0;
    auto result = triangleWithLeastNeighbours(vectorOfTriangles, traced, currentTriangle);

    /*untraced neighbour-triangle indices of currentTriangle, best candidate first*/
    std::vector<int> sortedNeighbours = result.first;
    /*currentTriangle's three edge slots leads to each candidate*/
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
                moveToPoint(pathHistory.top().pose, 0, movementDirection::BACKWARDS);
                pathHistory.pop();
                if (pathHistory.size() == 0){
                    return -1;
                }
            }
            moveToPoint(pathHistory.top().pose, pathHistory.top().triangleIndex, movementDirection::BACKWARDS);
            nextToTraceIndex = pathHistory.top().triangleIndex;
        }
    }else{
        /*if in the current position there are no reachable triangles*/
        pathHistory.pop();
        if (pathHistory.size() == 0){
                return -1;
        }
        /*move to a previous waypoint*/
        /*if it was an edge between triangles then move backwards twice*/
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

bool executePlannedPath(
    const std::vector<Waypoint> &orderedWaypoints)
{
    auto logger =
        rclcpp::get_logger("executePlannedPath");

    if(orderedWaypoints.empty())
    {
        RCLCPP_ERROR(
            logger,
            "No waypoints available for execution"
        );

        return false;
    }

    /*
     * Instead of executing every stored trajectory separately,
     * construct ONE continuous joint trajectory containing all
     * of them.
     */
    moveit_msgs::msg::RobotTrajectory combinedTrajectory;

    auto &combinedJointTrajectory =
        combinedTrajectory.joint_trajectory;

    bool firstSegment = true;

    /*
     * This stores the time of the final point currently contained
     * in the combined trajectory.
     *
     * Every new trajectory segment has its own time_from_start,
     * usually beginning again from zero. Therefore its timestamps
     * must be shifted before appending it.
     */
    int64_t combinedEndTimeNs = 0;

  

    /*
     * Helper function:
     * convert nanoseconds back into the ROS Duration message
     * used by trajectory_msgs::msg::JointTrajectoryPoint.
     */
    auto setTimeFromNanoseconds =
        [](builtin_interfaces::msg::Duration &duration,
           int64_t nanoseconds)
    {
        duration.sec =
            static_cast<int32_t>(
                nanoseconds / 1000000000LL
            );

        duration.nanosec =
            static_cast<uint32_t>(
                nanoseconds % 1000000000LL
            );
    };

    /* ============================================================
       COMBINE ALL STORED TRAJECTORIES
       ============================================================ */

    for(std::size_t i = 0;
        i < orderedWaypoints.size();
        ++i)
    {
        const auto &wp =
            orderedWaypoints[i];

        const auto &segment =
            wp.trajectory.joint_trajectory;

        /* --------------------------------------------------------
           Sanity checks
           -------------------------------------------------------- */

        if(segment.points.empty())
        {
            RCLCPP_ERROR(
                logger,
                "Waypoint %zu has an empty trajectory",
                i
            );

            return false;
        }

        if(segment.joint_names.empty())
        {
            RCLCPP_ERROR(
                logger,
                "Waypoint %zu has no joint names",
                i
            );

            return false;
        }

        /* --------------------------------------------------------
           First trajectory establishes the joint ordering
           -------------------------------------------------------- */

        if(firstSegment)
        {
            combinedJointTrajectory.joint_names =
                segment.joint_names;
        }
        else
        {
            /*
             * Every trajectory must use exactly the same joints
             * in exactly the same order.
             */
            if(segment.joint_names !=
               combinedJointTrajectory.joint_names)
            {
                RCLCPP_ERROR(
                    logger,
                    "Joint-name mismatch at waypoint %zu",
                    i
                );

                return false;
            }
        }

        /* --------------------------------------------------------
           Verify continuity between trajectories
           -------------------------------------------------------- */

        if(!firstSegment)
        {
            const auto &previousEnd =
                combinedJointTrajectory
                    .points.back()
                    .positions;

            const auto &currentStart =
                segment.points.front().positions;

            if(previousEnd.size() !=
               currentStart.size())
            {
                RCLCPP_ERROR(
                    logger,
                    "Joint-vector size mismatch at waypoint %zu",
                    i
                );

                return false;
            }

            double maxGap = 0.0;
            std::string worstJoint;

            for(std::size_t j = 0;
                j < previousEnd.size();
                ++j)
            {
                double gap =
                    std::abs(
                        previousEnd[j] -
                        currentStart[j]
                    );

                if(gap > maxGap)
                {
                    maxGap = gap;
                    worstJoint =
                        segment.joint_names[j];
                }
            }

            /*
             * Your diagnostic run showed 0.00000000 rad,
             * which is exactly what we want.
             */
            if(maxGap > 0.001)
            {
                RCLCPP_ERROR(
                    logger,
                    "Trajectory discontinuity before waypoint %zu: "
                    "%.8f rad on joint %s",
                    i,
                    maxGap,
                    worstJoint.c_str()
                );

                return false;
            }
        }

        /* --------------------------------------------------------
           Each individual trajectory has its own local clock.

           Example:

           trajectory A:
               0.0
               0.2
               0.4

           trajectory B:
               0.0
               0.1
               0.3

           We cannot simply append B because time would go backwards.

           Instead B becomes:
               0.5
               0.7

           relative to the accumulated trajectory.
           -------------------------------------------------------- */

        int64_t segmentStartTimeNs =
            rclcpp::Duration(
                segment.points.front().time_from_start
            ).nanoseconds();

        /*
         * The first point of every trajectory after the first
         * represents the same state as the last point of the
         * previous trajectory.
         *
         * Do not store that duplicate point twice.
         */
        std::size_t firstPointToCopy =
            firstSegment ? 0 : 1;

        /*
         * A segment containing only its duplicate starting point
         * contains no additional motion.
         */
        if(firstPointToCopy >= segment.points.size())
        {
            firstSegment = false;
            continue;
        }

        int64_t previousLocalTimeNs =
            segmentStartTimeNs;

        for(std::size_t pointIndex =
                firstPointToCopy;
            pointIndex < segment.points.size();
            ++pointIndex)
        {
            auto newPoint =
                segment.points[pointIndex];

            int64_t localTimeNs =
                rclcpp::Duration(
                    newPoint.time_from_start
                ).nanoseconds();

            /*
             * Make sure time progresses inside the original
             * trajectory.
             */
            if(pointIndex > 0 &&
               localTimeNs <= previousLocalTimeNs)
            {
                RCLCPP_ERROR(
                    logger,
                    "Non-increasing time inside waypoint %zu "
                    "at trajectory point %zu",
                    i,
                    pointIndex
                );

                return false;
            }

            /*
             * Convert this point's timestamp into time relative
             * to the beginning of THIS segment.
             */
            int64_t relativeTimeNs =
                localTimeNs -
                segmentStartTimeNs;

            /*
             * Shift it so it begins after everything that has
             * already been added to the combined trajectory.
             */
            int64_t newTimeNs =
                combinedEndTimeNs +
                relativeTimeNs;

            /*
             * Except for the very first trajectory point,
             * timestamps must increase continuously.
             */
            if(!combinedJointTrajectory.points.empty())
            {
                int64_t previousCombinedTimeNs =
                    rclcpp::Duration(
                        combinedJointTrajectory
                            .points.back()
                            .time_from_start
                    ).nanoseconds();

                if(newTimeNs <= previousCombinedTimeNs)
                {
                    RCLCPP_ERROR(
                        logger,
                        "Combined trajectory time is not increasing "
                        "at waypoint %zu point %zu",
                        i,
                        pointIndex
                    );

                    return false;
                }
            }

            setTimeFromNanoseconds(
                newPoint.time_from_start,
                newTimeNs
            );

            combinedJointTrajectory.points.push_back(
                newPoint
            );

            

            previousLocalTimeNs =
                localTimeNs;
        }

        /*
         * The next trajectory must start after the final
         * timestamp we just inserted.
         */
        if(!combinedJointTrajectory.points.empty())
        {
            combinedEndTimeNs =
                rclcpp::Duration(
                    combinedJointTrajectory
                        .points.back()
                        .time_from_start
                ).nanoseconds();
        }

        firstSegment = false;
    }

    /* ============================================================
       FINAL VALIDATION
       ============================================================ */

    if(combinedJointTrajectory.points.empty())
    {
        RCLCPP_ERROR(
            logger,
            "Combined trajectory contains no points"
        );

        return false;
    }

    RCLCPP_WARN(
        logger,
        "Combined %zu stored waypoints into %zu trajectory points",
        orderedWaypoints.size(),
        combinedJointTrajectory.points.size()
    );

    double totalDurationSeconds =
        static_cast<double>(combinedEndTimeNs) /
        1000000000.0;

    RCLCPP_WARN(
        logger,
        "Combined trajectory duration: %.3f seconds",
        totalDurationSeconds
    );

    /* ============================================================
       CHECK THE ROBOT AGAINST ONLY THE FIRST TRAJECTORY POINT
       ============================================================ */

    auto currentState =
        gripper_group_interface->getCurrentState(1.0);

    if(!currentState)
    {
        RCLCPP_ERROR(
            logger,
            "Could not obtain current robot state before execution"
        );

        return false;
    }

    const auto &firstPoint =
        combinedJointTrajectory.points.front();

    if(firstPoint.positions.size() !=
       combinedJointTrajectory.joint_names.size())
    {
        RCLCPP_ERROR(
            logger,
            "First trajectory point has invalid joint data"
        );

        return false;
    }

    double maxStartError = 0.0;
    std::string worstStartJoint;

    for(std::size_t j = 0;
        j < combinedJointTrajectory.joint_names.size();
        ++j)
    {
        const std::string &jointName =
            combinedJointTrajectory.joint_names[j];

        double actual =
            currentState->getVariablePosition(
                jointName
            );

        double planned =
            firstPoint.positions[j];

        double error =
            std::abs(actual - planned);

        if(error > maxStartError)
        {
            maxStartError = error;
            worstStartJoint = jointName;
        }
    }

    RCLCPP_WARN(
        logger,
        "Initial physical -> planned start error: "
        "%.8f rad on joint %s",
        maxStartError,
        worstStartJoint.c_str()
    );

    /* ============================================================
       EXECUTE THE COMPLETE PATH ONCE
       ============================================================ */

    moveit::planning_interface::
        MoveGroupInterface::Plan completePlan;

    completePlan.trajectory =
        combinedTrajectory;

    RCLCPP_WARN(
        logger,
        "Executing complete combined trajectory..."
    );

    auto result =
        gripper_group_interface->execute(
            completePlan
        );

    if(result !=
       moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(
            logger,
            "Combined trajectory execution FAILED"
        );

        return false;
    }

    RCLCPP_WARN(
        logger,
        "Complete combined trajectory executed successfully"
    );

    return true;
}