#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char** argv){
    constexpr double PI      = M_PI;

    rclcpp::init(argc, argv);

    auto const node = std::make_shared<rclcpp::Node>(
        "trace_cube",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    auto const logger = rclcpp::get_logger("trace_cube");

    using moveit::planning_interface::MoveGroupInterface;

    auto gripper_group_interface = MoveGroupInterface(node, "ur_manipulator");

    gripper_group_interface.setPlanningPipelineId("ompl");
    gripper_group_interface.setPlannerId("RRTConnectkConfigDefault");

    gripper_group_interface.setPlanningTime(5.0);
    gripper_group_interface.setMaxVelocityScalingFactor(1.0);
    gripper_group_interface.setMaxAccelerationScalingFactor(0.0);
    
    RCLCPP_INFO(logger, "Planning pipeline: %s", gripper_group_interface.getPlanningPipelineId().c_str());
    RCLCPP_INFO(logger, "Planner ID: %s", gripper_group_interface.getPlannerId().c_str());
    RCLCPP_INFO(logger, "Planning time: %.2f", gripper_group_interface.getPlanningTime());

    std::vector<double> preferred_joints = {
        -101.0 * PI / 180.0,
        -115.0 * PI / 180.0,
        -40.0 * PI / 180.0,
        -205.0 * PI / 180.0,
        -101.0 * PI / 180.0,
        -180.0 * PI / 180.0
    };

    gripper_group_interface.setJointValueTarget(preferred_joints);
    moveit::planning_interface::MoveGroupInterface::Plan home_plan;
    
    geometry_msgs::msg::Pose target_pose;
  
    auto ok = static_cast<bool>(gripper_group_interface.plan(home_plan));

    if (!ok) {
        RCLCPP_ERROR(logger, "Phase 1 planning failed!");
        rclcpp::shutdown();
        return 1;
    }
    gripper_group_interface.execute(home_plan);

    std::vector<geometry_msgs::msg::Pose> target_poses;
    
    target_pose.position.x = -0.25;
    target_pose.position.y = 0.60;
    target_pose.position.z = 0.5;
    target_pose.orientation.x = - 1 / sqrt(2);
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 1 / sqrt(2);
    target_poses.push_back(target_pose);
    
  

    int i = 1;
    while((abs(target_pose.position.x + 0.25) >= 0.02) || (abs(target_pose.position.z - 0.08) >= 0.02)){

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = gripper_group_interface.computeCartesianPath(target_poses, 0.01, trajectory, true);
    
    RCLCPP_INFO(logger, "Cartesian Path coverage: %.2f%%", fraction * 100.00);

    if (fraction >= 1.0) {
    RCLCPP_INFO(logger, "Full Cartesian path achieved — executing...");

    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
    cartesian_plan.trajectory = trajectory;
    gripper_group_interface.execute(cartesian_plan);
    }
    else if (fraction > 0.9) {
        // Accept >90% coverage (optional — remove for strict full-path only)
        RCLCPP_WARN(logger, "Partial path (%.1f%%) — executing anyway", fraction * 100.0);
        moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
        cartesian_plan.trajectory = trajectory;
        gripper_group_interface.execute(cartesian_plan);
    }
    else {
        RCLCPP_ERROR(logger, "Cartesian path only %.1f%% complete — collision likely blocked it!", fraction * 100.0);
        return 1;
    }

    target_poses.pop_back();

    if(i != 0){
       target_pose.position.z = target_pose.position.z - 0.01; 
    }

    if (i % 2 != 0){
        target_pose.position.x = - target_pose.position.x;
    }

    target_poses.push_back(target_pose);
    i = i + 1;

    }

    preferred_joints = {
        0.0,
        - M_PI / 2.0,
        0.0,
        0.0,
        0.0,
        0.0
    };

    gripper_group_interface.setJointValueTarget(preferred_joints);
    
    ok = static_cast<bool>(gripper_group_interface.plan(home_plan));

    if(!ok){
        RCLCPP_ERROR(logger, "Planning to home failed");
        rclcpp::shutdown();
        return 1;
    }

    gripper_group_interface.execute(home_plan);
    RCLCPP_INFO(logger, "Cube Face Tracing complete. Returning to home position now...");


    rclcpp::shutdown();
    return 0;
}