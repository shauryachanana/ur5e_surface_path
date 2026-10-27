#include "trace_cube.hpp"
#include "stateMachineIKU/tracingStateMachine.hpp"
#include <iostream>
#include <climits>

extern tracingStateMachine sm;

std::stack<Waypoint> pathHistory;
std::stack<Waypoint> stackOfReachableWaypoints;
extern std::vector<bool> traced;
std::vector<Triangle> plannedPath;

// Traced triangles where there was still an untraced neighbour.
// Recovery only considers these triangles; it never scans the whole mesh.
std::vector<int> branchPoints;

// Distance from tool0 to the pen tip along tool0's local +Z axis.
// Positions in this file are in meters: 0.05 = 50 mm, 0.01 = 10 mm.
constexpr double PEN_LENGTH = 0.05;
constexpr int PEN_ROTATION_STEPS = 50;

//pen tip height above the surface during a jump [m]
constexpr double JUMP_LIFT = 0.05;
constexpr int MAX_JUMP_TRIES = 3;

std::size_t lastUsefulWaypointCount = 0;

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
    gripper_group_interface->setMaxAccelerationScalingFactor(0.4);
}

void goHome(){
    auto logger = rclcpp::get_logger("goGome");
    #ifdef DEGUBBER
    RCLCPP_WARN(logger, "going home");
    #endif

    std::vector<double> preferred_joints = {
        -90.0 * M_PI / 180.0,
        -110.0 * M_PI / 180.0,
        -45.0 * M_PI / 180.0,
        -110.0 * M_PI / 180.0,
        -260.0 * M_PI / 180.0,
        -180.0 * M_PI / 180.0
    };

    RCLCPP_WARN(logger, "set joint target");

    gripper_group_interface->setJointValueTarget(preferred_joints);

    moveit::planning_interface::MoveGroupInterface::Plan home_plan;

    RCLCPP_WARN(logger, "planning");

    // Plan only once, using the existing interface.
    auto result = gripper_group_interface->plan(home_plan);

    RCLCPP_ERROR(
        logger,
        "MoveIt planning result code: %d",
        result.val);

    if (static_cast<bool>(result)) {
        RCLCPP_WARN(logger, "planning succeeded; executing home plan");

        auto execution_result = gripper_group_interface->execute(home_plan);

        if (static_cast<bool>(execution_result)) {
            RCLCPP_WARN(logger, "initial position reached");
        } else {
            RCLCPP_ERROR(logger, "home trajectory execution failed");
        }
    } else {
        RCLCPP_ERROR(logger, "initial positioning failed");
    }
}

bool getTCPpose(){
    auto logger = rclcpp::get_logger("currentTCP");

    // RCLCPP_WARN(
    //     logger,
    //     "getTCPpose"
    // );

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

        sm.vars.currentTCP[0] = transform.transform.translation.x;
        sm.vars.currentTCP[1] = transform.transform.translation.y;
        sm.vars.currentTCP[2] = transform.transform.translation.z;

        // RCLCPP_WARN(
        //     logger,
        //     "getTCPpose good"
        // );

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

//TODO: make orientation sm var
void getTCPorientation(double* TCPorientation){
    auto logger = rclcpp::get_logger("getTCPorientation");
    #ifdef DEBUGGER
    RCLCPP_WARN(
        logger,
        "getTCPorient"
    );
    #endif

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
        #ifdef DEBUGGER
        RCLCPP_WARN(
            logger,
            "getTCPorient success"
        );
        #endif
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

bool moveToPoint(movementDirection movementDir, waypointType waypoint, const std::vector<geometry_msgs::msg::Pose> &viaPoses){
    auto logger = rclcpp::get_logger("moveToPoint");

    #ifdef DEBUGGER
    RCLCPP_WARN(
        logger,
        "attempt to move"
    );
    #endif

    moveit_msgs::msg::RobotTrajectory trajectory;
    //via poses (if any) are visited first, in the same Cartesian path
    std::vector<geometry_msgs::msg::Pose> target_poses = viaPoses;

    target_poses.push_back(sm.vars.target_pose);
    double fraction = gripper_group_interface->computeCartesianPath(target_poses, 0.01, trajectory, true);

    //Full Cartesian path achieved
    //a jump (via poses) has to be planned completely, a normal move needs 90 %
    if (fraction >= (viaPoses.empty() ? 0.9 : 0.99)) {
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

        Waypoint newWaypoint = {sm.vars.target_pose, waypoint, sm.vars.triangleToMemorize, trajectory};
        stackOfReachableWaypoints.push(newWaypoint);

        if(movementDir == movementDirection::FORWARD){
            pathHistory.push(newWaypoint);
            //dont use it as a counter, use .isTraced on the Triangle
            //also share any ideas you have regarding the "planning" part
            RCLCPP_ERROR(logger, "type of waypoint: %d",(int)pathHistory.top().typeOfWaypoint);
        }
        
        target_poses.pop_back();
        #ifdef DEBUGGER
        RCLCPP_WARN(
            logger,
            "successful move"
        );
        #endif
        return true;
    }else {
        #ifdef DEBUGGER
        RCLCPP_WARN(
            logger,
            "failed move"
        );
        #endif
        target_poses.pop_back();
        return false;
    }
}

static tf2::Quaternion orientationFor(Triangle* triangle)
{
    // KEEP YOUR ORIGINAL WORKING NORMAL SIGNS HERE
    tf2::Vector3 normal(
        -triangle->normal_x ,
        -triangle->normal_y,
         triangle->normal_z
    );
    normal.normalize();

    tf2::Vector3 z_axis = -normal;
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

    return q;
}

geometry_msgs::msg::Pose poseFor(Triangle* triangle){
    geometry_msgs::msg::Pose p;

    // Use your existing orientationFor function instead of duplicating math
    tf2::Quaternion q = orientationFor(triangle);
    q.normalize();

    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();
    p.orientation.w = q.w();

    const tf2::Vector3 penTip(
        -triangle->centreOfTriangle[0] * 0.001,
        -triangle->centreOfTriangle[1] * 0.001 + 0.65f,
        triangle->centreOfTriangle[2] * 0.001
    );

    const tf2::Vector3 toolToTip =
        tf2::Matrix3x3(q) * tf2::Vector3(0.0, 0.0, PEN_LENGTH);

    p.position.x = penTip.x() - toolToTip.x();
    p.position.y = penTip.y() - toolToTip.y();
    p.position.z = penTip.z() - toolToTip.z();

    return p;
}

void targetPose(Triangle* triangle){
    auto logger = rclcpp::get_logger("targetPose");
    #ifdef DEBUGGER
    RCLCPP_WARN(
        logger,
        "targetPose"
    );
    #endif
    sm.vars.triangleToMemorize = triangle;
    sm.vars.oldPose = sm.vars.target_pose;
    sm.vars.target_pose = poseFor(triangle);
}

geometry_msgs::msg::Pose liftedPose(Triangle* triangle){
    auto logger = rclcpp::get_logger("liftedPose");
    #ifdef DEBUGGER
    RCLCPP_WARN(logger, "liftedPose");
    #endif

    geometry_msgs::msg::Pose p = poseFor(triangle);
    tf2::Quaternion q = orientationFor(triangle);

    tf2::Vector3 liftOffset = tf2::Matrix3x3(q) * tf2::Vector3(0.0, 0.0, JUMP_LIFT);

    p.position.x -= liftOffset.x();
    p.position.y -= liftOffset.y();
    p.position.z -= liftOffset.z();

    return p;
}

void edgeTipPose(){
    sm.vars.edgePenTip.x = - (sm.vars.sortedEdges[sm.vars.iterator]->centreOfEdge[0] * 0.001f);
    sm.vars.edgePenTip.y = - (sm.vars.sortedEdges[sm.vars.iterator]->centreOfEdge[1] * 0.001f) + 0.65f;
    sm.vars.edgePenTip.z = (sm.vars.sortedEdges[sm.vars.iterator]->centreOfEdge[2] * 0.001f);
}

bool moveToPointWithPenRotation(){
    auto logger = rclcpp::get_logger("moveToPointWithPenRotation");

        #ifdef DEBUGGER
    RCLCPP_WARN(
        logger,
        "moveToPointWithPenRotation"
    );
    #endif

    moveit_msgs::msg::RobotTrajectory trajectory;
    std::vector<geometry_msgs::msg::Pose> target_poses;

    tf2::Quaternion startQ(
        sm.vars.oldPose.orientation.x,
        sm.vars.oldPose.orientation.y,
        sm.vars.oldPose.orientation.z,
        sm.vars.oldPose.orientation.w
    );
    tf2::Quaternion endQ(
        sm.vars.target_pose.orientation.x,
        sm.vars.target_pose.orientation.y,
        sm.vars.target_pose.orientation.z,
        sm.vars.target_pose.orientation.w
    );
    startQ.normalize();
    endQ.normalize();

    // Quaternions q and -q represent the same orientation. Keep the
    // interpolation on the shorter rotational path.
    if (startQ.dot(endQ) < 0.0) {
        endQ = tf2::Quaternion(-endQ.x(), -endQ.y(), -endQ.z(), -endQ.w());
    }

    // The sm.vars.edgePenTip is already the first point of this operation.
    // From there, rotate around the pen tip while compensating tool0's
    // position so the pen tip stays at exactly the same point.
    for(int i = 1; i <= PEN_ROTATION_STEPS; i++){
        double t = (double)i / (double)PEN_ROTATION_STEPS;
        tf2::Quaternion q = startQ.slerp(endQ, t);
        q.normalize();

        tf2::Vector3 penOffset = tf2::Matrix3x3(q) * tf2::Vector3(0.0, 0.0, PEN_LENGTH);

        geometry_msgs::msg::Pose rotationPose;
        rotationPose.position.x = sm.vars.edgePenTip.x - penOffset.x();
        rotationPose.position.y = sm.vars.edgePenTip.y - penOffset.y();
        rotationPose.position.z = sm.vars.edgePenTip.z - penOffset.z();

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
                RCLCPP_WARN(
                    logger,
                    "moveToPointWithPenRotation fail"
                );
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
    moveit_msgs::msg::RobotTrajectory t2;
    const double f2 = gripper_group_interface->computeCartesianPath(target_poses, 0.01, t2, false);
    RCLCPP_ERROR(logger, "diag: same path WITHOUT collision check: fraction=%f", f2);

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

bool goBackOnHistory(){
    auto logger = rclcpp::get_logger("goBackOnHistory");
        #ifdef DEBUGGER
    RCLCPP_WARN(
        logger,
        "goBackOnHistory"
    );
    #endif
    if(pathHistory.empty()) return false;
    pathHistory.pop();                       // the waypoint we are standing on
    if(pathHistory.empty()) return false;

    if(pathHistory.top().typeOfWaypoint == waypointType::EDGE){
        sm.vars.target_pose = pathHistory.top().pose;
        sm.vars.triangleToMemorize = pathHistory.top().triangle;
        moveToPoint(movementDirection::BACKWARDS);
        pathHistory.pop();
        if(pathHistory.empty()) return false;
    }
    sm.vars.target_pose = pathHistory.top().pose;
    sm.vars.triangleToMemorize = pathHistory.top().triangle;

    moveToPoint(movementDirection::BACKWARDS);

    sm.vars.currentTriangle = sm.vars.triangleToMemorize;
    sm.vars.iterator = 0;
    return true;
}

void markAttemptFailed(){
    auto logger = rclcpp::get_logger("markAttemptFailed");
        #ifdef DEBUGGER
    RCLCPP_WARN(
        logger,
        "markAttemptFailed"
    );
    #endif
    for(std::size_t i = 0; i < sm.vars.previousTriangle->myNeighbours.size(); i++){
        if(sm.vars.previousTriangle->myNeighbours[i] == sm.vars.currentTriangle->myIndex){
            sm.vars.previousTriangle->neighbourFailed[i] = true;
        }
    }
    sm.vars.currentTriangle->unreachableCounter++;
}

void edgePoseWithOldOrient(){
    auto logger = rclcpp::get_logger("edgePoseWithOldOrient");
    RCLCPP_WARN(
        logger,
        "edgePoseWithOldOrient"
    );
    
    // Get orientation of the previous triangle
    tf2::Quaternion q = orientationFor(sm.vars.previousTriangle);
    sm.vars.target_pose.orientation.x = q.x();
    sm.vars.target_pose.orientation.y = q.y();
    sm.vars.target_pose.orientation.z = q.z();
    sm.vars.target_pose.orientation.w = q.w();

    // Calculate edge pen tip in base_link coordinates
    const tf2::Vector3 edgeTip(
        - (sm.vars.sortedEdges[sm.vars.iterator]->centreOfEdge[0] * 0.001f),
        - (sm.vars.sortedEdges[sm.vars.iterator]->centreOfEdge[1] * 0.001f) + 0.65f,
          (sm.vars.sortedEdges[sm.vars.iterator]->centreOfEdge[2] * 0.001f)
    );

    // Properly rotate the pen length offset using the orientation matrix
    const tf2::Vector3 toolToTip =
        tf2::Matrix3x3(q) * tf2::Vector3(0.0, 0.0, PEN_LENGTH);

    sm.vars.target_pose.position.x = edgeTip.x() - toolToTip.x();
    sm.vars.target_pose.position.y = edgeTip.y() - toolToTip.y();
    sm.vars.target_pose.position.z = edgeTip.z() - toolToTip.z();
}

void attemptToReachNextClosest(){
    if(sm.vars.initialTriangle == AttemptToReach::TRIANGLE_REACHED) return;

    auto logger = rclcpp::get_logger("attemptToReachNextClosest");
        #ifdef DEBUGGER
    RCLCPP_WARN(
        logger,
        "attemptToReachNextClosest"
    );
    #endif
    sm.vars.iterator = 0;
    while(chosenVector < 3){
        switch (chosenVector){
        case 0:
            sm.vars.vectorOfDesiredTriangles = singleNeighbourTrangles;
            break;
        case 1:
            sm.vars.vectorOfDesiredTriangles = doubleNeighbourTriangles;
            break;
        case 2:
            sm.vars.vectorOfDesiredTriangles = vectorOfTriangles;
            break;
        }
        if(!sm.vars.vectorOfDesiredTriangles.empty()){
            break;
        }
        chosenVector++;
    }
    RCLCPP_WARN(
        logger,
        "chosenVector %d", chosenVector
    );
    sm.vars.size = sm.vars.vectorOfDesiredTriangles.size();
}

void triangleWithLeastNeighbours(){
    auto logger = rclcpp::get_logger("triangleWithLeastNeighbours");
        #ifdef DEBUGGER
    RCLCPP_WARN(
        logger,
        "triangleWithLeastNeighbours"
    );
    #endif

    sm.vars.sortedNeighbours.clear();
    sm.vars.sortedEdges.clear();

    getTCPpose();

    std::vector<int> validNeighbours;
    std::vector<int> edgeIndices;

    for(std::size_t i = 0; i < sm.vars.previousTriangle->myNeighbours.size(); i++){
        int neighbourIndex = sm.vars.previousTriangle->myNeighbours[i];
        //if the value stored in an array is a valid one and if the triangles stored was not traced
        if((neighbourIndex != -1) && 
            (vectorOfTriangles[neighbourIndex]->traced == false) && 
            (traced[neighbourIndex] == false) &&
            (!sm.vars.previousTriangle->neighbourFailed[i])){
            //valuyes from an array
            validNeighbours.push_back(neighbourIndex);
            //the position of this value
            edgeIndices.push_back(i);
        }
    }

    //helper vector
    std::vector<int> order(validNeighbours.size());
    std::iota(order.begin(), order.end(), 0);

    std::sort(order.begin(), order.end(), [&](int a, int b){
        int countA = vectorOfTriangles[validNeighbours[a]]->getValidNeighbours();
        int countB = vectorOfTriangles[validNeighbours[b]]->getValidNeighbours();
        if(countA != countB){
            return countA < countB;
        }else{
            return vectorOfTriangles[validNeighbours[a]]->centreOfTriangle[2] < 
                   vectorOfTriangles[validNeighbours[b]]->centreOfTriangle[2];
        }
    });

    //reorder both vectors according to sort result
    for(int i : order){
        sm.vars.sortedNeighbours.push_back(vectorOfTriangles[validNeighbours[i]]);
        sm.vars.sortedEdges.push_back(sm.vars.previousTriangle->triangleEdges[edgeIndices[i]]);
    }

    return;
}

void rememberBranchPoint(){
    auto logger = rclcpp::get_logger("rememberBranchPoint");
        #ifdef DEBUGGER
    RCLCPP_WARN(
        logger,
        "rememberBranchPoint"
    );
    #endif
    if(sm.vars.previousTriangle->getValidNeighbours() == 0)
        return;

    if(std::find(branchPoints.begin(), branchPoints.end(), sm.vars.previousTriangle->myIndex) == branchPoints.end())
        branchPoints.push_back(sm.vars.previousTriangle->myIndex);
}

void getBranchCandidates(){
    auto logger = rclcpp::get_logger("getBranchCandidates");
        #ifdef DEBUGGER
    RCLCPP_WARN(
        logger,
        "getBranchCandidates"
    );
    #endif
    sm.vars.candidates.clear();

    // pathHistory already contains the surface route that reached the dead end
    // use that route as the distance metric instead of scanning all mesh triangles.
    std::unordered_map<int, int> pathDistance;
    std::stack<Waypoint> historyCopy = pathHistory;
    int distance = 0;

    while(!historyCopy.empty()){
        const Waypoint &waypoint = historyCopy.top();
        if(waypoint.typeOfWaypoint == waypointType::TRIANGLE && waypoint.triangle->myIndex >= 0){
            if(pathDistance.find(waypoint.triangle->myIndex) == pathDistance.end())
                pathDistance[waypoint.triangle->myIndex] = distance;
            distance++;
        }
        historyCopy.pop();
    }

    for(int index : branchPoints){
        // invalid index
        if(index < 0 || index >= (int)vectorOfTriangles.size())
            continue;

        Triangle* triangle = vectorOfTriangles[index];

        // no reachable neighbours
        if(triangle->getValidNeighbours() == 0)
            continue;
        if(triangle->unreachableCounter >= MAX_JUMP_TRIES)
            continue;

        auto it = pathDistance.find(index);
        sm.vars.candidates.push_back({triangle, it == pathDistance.end() ? INT_MAX : it->second});
    }

    std::sort(sm.vars.candidates.begin(), sm.vars.candidates.end(), [&](const BranchCandidate &a, const BranchCandidate &b){
        // candidates with a smaller pathDistance come first.
        if(a.pathDistance != b.pathDistance)
            return a.pathDistance < b.pathDistance;

        // if two candidates have the same path distance, the one with fewer previous failed attempts is preferred
        const Triangle *ta = a.triangle;
        const Triangle *tb = b.triangle;
        return ta->unreachableCounter < tb->unreachableCounter;
    });

    return;
}

bool planOMPLPose(){
    auto logger = rclcpp::get_logger("planOMPLPose");
        #ifdef DEBUGGER
    RCLCPP_WARN(
        logger,
        "planOMPLPose"
    );
    #endif
    gripper_group_interface->setPoseTarget(sm.vars.target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(gripper_group_interface->plan(plan));
    gripper_group_interface->clearPoseTargets();

    if(!success || plan.trajectory.joint_trajectory.points.empty() || sm.vars.currentState == nullptr)
        return false;

    moveit::core::RobotState endState(*sm.vars.currentState);
    endState.setJointGroupPositions(
        gripper_group_interface->getName(),
        plan.trajectory.joint_trajectory.points.back().positions
    );
    endState.update();
    gripper_group_interface->setStartState(endState);

    Waypoint newWaypoint = {
        sm.vars.target_pose,
        waypointType::TRIANGLE,
        sm.vars.currentTriangle,
        plan.trajectory
    };
    stackOfReachableWaypoints.push(newWaypoint);

    return true;
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

float computeCoveragePercent(const std::vector<Triangle> &plannedPathVec){
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

// ---- DIAGNOSTIC SWITCH: uncomment to run waypoint-by-waypoint ----
// #define EXECUTE_BY_WAYPOINTS

bool executePlannedPath(const std::vector<Waypoint> &orderedWaypoints)
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

    #ifdef EXECUTE_BY_WAYPOINTS
    {
    RCLCPP_WARN(logger, "=== DIAGNOSTIC: executing %zu waypoints one by one ===",
                orderedWaypoints.size());
    bool allOk = true;
    const double jointSpeed = 0.5;   // rad/s, slow on purpose

    for(std::size_t i = 0; i < orderedWaypoints.size(); ++i)
    {
        const auto &wp = orderedWaypoints[i];

        // straight line from where the robot REALLY is to this pose
        gripper_group_interface->setStartStateToCurrentState();

        moveit_msgs::msg::RobotTrajectory traj = wp.trajectory;   // as planned, all types
        const double fraction = 1.0;
        // moveit_msgs::msg::RobotTrajectory traj;
        // double fraction;
        // if(wp.typeOfWaypoint == waypointType::EDGE){
        //     traj = wp.trajectory;        // edge move + pen rotation, as planned
        //     fraction = 1.0;
        // }else{
        //     std::vector<geometry_msgs::msg::Pose> single{wp.pose};
        //     fraction = gripper_group_interface->computeCartesianPath(single, 0.005, traj, true);
        // }

        // re-time the points (Cartesian output has no usable timestamps)
        auto &pts = traj.joint_trajectory.points;
        double t = 0.0;
        for(std::size_t k = 0; k < pts.size(); ++k){
            if(k > 0){
                double maxDelta = 0.0;
                for(std::size_t j = 0; j < pts[k].positions.size(); ++j)
                    maxDelta = std::max(maxDelta,
                        std::abs(pts[k].positions[j] - pts[k-1].positions[j]));
                t += std::max(0.001, maxDelta / jointSpeed);
            }
            pts[k].time_from_start.sec     = static_cast<int32_t>(t);
            pts[k].time_from_start.nanosec = static_cast<uint32_t>((t - static_cast<int32_t>(t)) * 1e9);
            pts[k].velocities.clear();
            pts[k].accelerations.clear();
        }

        std::string status = "SKIPPED(fraction<0.9)";
        if(fraction >= 0.9){
            if(pts.size() > 1){
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                plan.trajectory = traj;
                status = (gripper_group_interface->execute(plan) ==
                            moveit::core::MoveItErrorCode::SUCCESS) ? "OK" : "EXEC_FAIL";
            } else {
                status = "ALREADY_THERE";
            }
        }
        if(status != "OK" && status != "ALREADY_THERE") allOk = false;

        // where did the robot really end up?
        rclcpp::sleep_for(std::chrono::milliseconds(300));
        geometry_msgs::msg::TransformStamped tf;
        try{
            tf = tf_buffer->lookupTransform("base_link", "tool0", tf2::TimePointZero);
        }catch(const tf2::TransformException &e){
            RCLCPP_ERROR(logger, "WP %zu: TF failed: %s", i, e.what());
            allOk = false;
            continue;
        }

        tf2::Quaternion qT(wp.pose.orientation.x, wp.pose.orientation.y,
                            wp.pose.orientation.z, wp.pose.orientation.w);
        tf2::Quaternion qA(tf.transform.rotation.x, tf.transform.rotation.y,
                            tf.transform.rotation.z, tf.transform.rotation.w);
        qT.normalize(); qA.normalize();
        const tf2::Vector3 pT(wp.pose.position.x, wp.pose.position.y, wp.pose.position.z);
        const tf2::Vector3 pA(tf.transform.translation.x, tf.transform.translation.y,
                                tf.transform.translation.z);
        const tf2::Vector3 tipT = pT + tf2::Matrix3x3(qT) * tf2::Vector3(0, 0, PEN_LENGTH);
        const tf2::Vector3 tipA = pA + tf2::Matrix3x3(qA) * tf2::Vector3(0, 0, PEN_LENGTH);

        RCLCPP_WARN(logger,
            "WP %zu/%zu %s tri=%d | fraction=%.3f pts=%zu exec=%s\n"
            "   tool0 target (m): %.4f %.4f %.4f | reached: %.4f %.4f %.4f | err %.2f mm, rot %.2f deg\n"
            "   pen tip target (m): %.4f %.4f %.4f | reached: %.4f %.4f %.4f | tip err %.2f mm\n"
            "   pen tip target in MESH frame (mm): %.1f %.1f %.1f",
            i, orderedWaypoints.size(),
            wp.typeOfWaypoint == waypointType::EDGE ? "EDGE" : "TRI ",
            wp.triangle ? wp.triangle->myIndex : -1,
            fraction, pts.size(), status.c_str(),
            pT.x(), pT.y(), pT.z(), pA.x(), pA.y(), pA.z(),
            (pA - pT).length() * 1000.0,
            qT.angleShortestPath(qA) * 180.0 / M_PI,
            tipT.x(), tipT.y(), tipT.z(), tipA.x(), tipA.y(), tipA.z(),
            (tipA - tipT).length() * 1000.0,
            -tipT.x() * 1000.0, -(tipT.y() - 0.65) * 1000.0, tipT.z() * 1000.0);

        // ---- orientation details ----
        double rT, pT_, yT, rA, pA_, yA;
        tf2::Matrix3x3(qT).getRPY(rT, pT_, yT);
        tf2::Matrix3x3(qA).getRPY(rA, pA_, yA);

        const tf2::Vector3 penDirT = tf2::Matrix3x3(qT) * tf2::Vector3(0, 0, 1);
        const tf2::Vector3 penDirA = tf2::Matrix3x3(qA) * tf2::Vector3(0, 0, 1);

        RCLCPP_WARN(logger,
            "   orient target quat (x y z w): %.4f %.4f %.4f %.4f | reached: %.4f %.4f %.4f %.4f\n"
            "   orient target RPY (deg): %.1f %.1f %.1f | reached: %.1f %.1f %.1f\n"
            "   pen direction target: (%.3f, %.3f, %.3f) | reached: (%.3f, %.3f, %.3f) | angle between %.2f deg",
            qT.x(), qT.y(), qT.z(), qT.w(),
            qA.x(), qA.y(), qA.z(), qA.w(),
            rT * 180.0 / M_PI, pT_ * 180.0 / M_PI, yT * 180.0 / M_PI,
            rA * 180.0 / M_PI, pA_ * 180.0 / M_PI, yA * 180.0 / M_PI,
            penDirT.x(), penDirT.y(), penDirT.z(),
            penDirA.x(), penDirA.y(), penDirA.z(),
            penDirT.angle(penDirA) * 180.0 / M_PI);
    }

    RCLCPP_WARN(logger, "=== DIAGNOSTIC finished, all waypoints ok: %s ===",
                allOk ? "YES" : "NO");
    return allOk;
    }
#else

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

        int64_t previousLocalTimeNs = segmentStartTimeNs;

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

            if(pointIndex > 0 && localTimeNs <= previousLocalTimeNs)
            {
                double maxDelta = 0.0;
                for(std::size_t j = 0; j < newPoint.positions.size(); ++j)
                {
                    maxDelta = std::max(maxDelta, std::abs(newPoint.positions[j] - segment.points[pointIndex - 1].positions[j]));
                }
                localTimeNs = previousLocalTimeNs + std::max<int64_t>(1000000, static_cast<int64_t>(maxDelta * 1e9)); // 1 rad/s, min 1 ms
                newPoint.velocities.clear();
                newPoint.accelerations.clear();
            }

            /*
             * Make sure time progresses inside the original
             * trajectory.
             */
            if(pointIndex > 0 &&
               localTimeNs <= previousLocalTimeNs)
            {
    RCLCPP_ERROR(
        logger,
        "Invalid segment timing: waypoint=%zu, type=%d, "
        "triangle=%d, point=%zu/%zu, previous=%.9f s, "
        "current=%.9f s, pose=(%.4f, %.4f, %.4f)",
        i,
        static_cast<int>(wp.typeOfWaypoint),
        wp.triangle ? wp.triangle->myIndex : -1,
        pointIndex,
        segment.points.size(),
        static_cast<double>(previousLocalTimeNs) / 1e9,
        static_cast<double>(localTimeNs) / 1e9,
        wp.pose.position.x,
        wp.pose.position.y,
        wp.pose.position.z
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
#endif
}

void restoreVirtualStartState(){
    auto logger = rclcpp::get_logger("restoreVirtualStartState");
    RCLCPP_WARN(
        logger,
        "restoreVirtualStartState"
    );
    auto& sr = stackOfReachableWaypoints;
    if(sr.empty() || sr.top().trajectory.joint_trajectory.points.empty()){
        gripper_group_interface->setStartStateToCurrentState(); return; }
    moveit::core::RobotState st(*sm.vars.currentState);
    st.setJointGroupPositions(gripper_group_interface->getName(),
        sr.top().trajectory.joint_trajectory.points.back().positions);
    st.update();
    gripper_group_interface->setStartState(st);
}