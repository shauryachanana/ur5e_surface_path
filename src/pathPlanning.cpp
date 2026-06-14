#include "trace_cube.hpp"

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

    // currentTCP[3] = transform.transform.rotation.x;
    // currentTCP[4] = transform.transform.rotation.y;
    // currentTCP[5] = transform.transform.rotation.z;
    // currentTCP[6] = transform.transform.rotation.w;
}

bool moveToPoint(geometry_msgs::msg::Pose target_pose){
    auto logger = rclcpp::get_logger("moveToPoint");

    moveit_msgs::msg::RobotTrajectory trajectory;
    std::vector<geometry_msgs::msg::Pose> target_poses;

    target_poses.push_back(target_pose);
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
        target_poses.pop_back();
        return true;
    }else {
        target_poses.pop_back();
        RCLCPP_ERROR(logger, "Cartesian path only %.1f%% complete — collision likely blocked it!", fraction * 100.0);
        return false;
    }
}

void traceThreeNeighbours(
    std::vector<Triangle> &vectorOfTriangles, 
    int previousTriangle, 
    Triangle& triangleToTrace, 
    Edge& edgeToPrevTriangle
){
    auto logger = rclcpp::get_logger("traceThreeNeighbours");

    geometry_msgs::msg::Pose target_pose;

    double TCPorientation[7] = {0,0,0,0,0,0,0};
    getTCPpose(TCPorientation);

    //move to this triangle------------------------------------------------------------------------------------------------------------
    target_pose.position.x = triangleToTrace.centre_x * 0.001f;
    target_pose.position.y = triangleToTrace.centre_y * 0.001f + 1.00f;
    target_pose.position.z = triangleToTrace.centre_z * 0.001f;

    tf2::Vector3 normal(
        triangleToTrace.normal_x,
        triangleToTrace.normal_y,
        triangleToTrace.normal_z
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

    //if center to center failed
    if(moveToPoint(target_pose) == false){
        //trt going to an edge
        target_pose.position.x = edgeToPrevTriangle.center[0] * 0.001f;
        target_pose.position.y = edgeToPrevTriangle.center[1] * 0.001f + 1.00f;
        target_pose.position.z = edgeToPrevTriangle.center[2] * 0.001f;
        //use the orientation of the old triangle to avoid collisions
        target_pose.orientation.x = TCPorientation[3];
        target_pose.orientation.y = TCPorientation[4];
        target_pose.orientation.z = TCPorientation[5];
        target_pose.orientation.w = TCPorientation[6];
        //if even edge is unreachable, then the triangles unreachability counter goes up
        if(moveToPoint(target_pose) == false){
            triangleToTrace.unreachableCounter++;
        }else{
            //if we did go to an edge then we can try to go to the ceter of the next triangle
            target_pose.position.x = triangleToTrace.centre_x * 0.001f;
            target_pose.position.y = triangleToTrace.centre_y * 0.001f + 1.00f;
            target_pose.position.z = triangleToTrace.centre_z * 0.001f;
            target_pose.orientation.x = q.x();
            target_pose.orientation.y = q.y();
            target_pose.orientation.z = q.z();
            target_pose.orientation.w = q.w();
            if(moveToPoint(target_pose) == false){
                //if the center is ynreachable then we go back to "initial" triangle
                target_pose.position.x = vectorOfTriangles[previousTriangle].centre_x * 0.001f;
                target_pose.position.y = vectorOfTriangles[previousTriangle].centre_y * 0.001f + 1.00f;
                target_pose.position.z = vectorOfTriangles[previousTriangle].centre_z * 0.001f;
                target_pose.orientation.x = TCPorientation[3];
                target_pose.orientation.y = TCPorientation[4];
                target_pose.orientation.z = TCPorientation[5];
                target_pose.orientation.w = TCPorientation[6];
                triangleToTrace.unreachableCounter++;
                moveToPoint(target_pose);
            }
        }
    }

    //trace neghbours------------------------------------------------------------------------------------------------------------------
    for(int j = 0; j < 3; j++){
        for(int i = 0; i < 3; i++){
            if((triangleToTrace.myNeighbours[i] != -1) && //is there is a neighbour
                (vectorOfTriangles[triangleToTrace.myNeighbours[i]].unreachableCounter < 3) && //and it is not unreachable
                (!vectorOfTriangles[triangleToTrace.myNeighbours[i]].traced)){ //and it was not traced
                    traceThreeNeighbours(
                        vectorOfTriangles, 
                        triangleToTrace.myIndex, 
                        vectorOfTriangles[triangleToTrace.myNeighbours[i]], 
                        triangleToTrace.triangleEdges[i]
                    ); 
            }
        }
    }
    return;
}