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
    } else{
        RCLCPP_ERROR(logger, "homeplan failed!");
    }
}

void getTCPpose(double* currentTCP){
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

    currentTCP[0] = transform.transform.translation.x,
    currentTCP[1] = transform.transform.translation.y,
    currentTCP[2] = transform.transform.translation.z;
}