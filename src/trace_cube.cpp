#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char** argv){
    rclcpp::init(argc, argv);

    auto const node = std::make_shared<rclcpp::Node>(
        "trace_cube",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    auto const logger = rclcpp::get_logger("trace_cube");

    using moveit::planning_interface::MoveGroupInterface;

    //create an object and pass our node and the name of the structure you want to control (founf in "Planning" tab)
    auto gripper_group_interface = MoveGroupInterface(node, "ur_manipulator");

    //set id from "Context" tab sed desired planning library (open motion plannin library)
    gripper_group_interface.setPlanningPipelineId("ompl");
    //decides which motion algorithm to use
    gripper_group_interface.setPlannerId("RRTConnectkConfigDefault");

    //~the longer thr better
    gripper_group_interface.setPlanningTime(5.0);
    //from 0 to 1
    gripper_group_interface.setMaxVelocityScalingFactor(1.0);
    //0 to 1, 0 for const velocity 
    gripper_group_interface.setMaxAccelerationScalingFactor(0.0);
    
    RCLCPP_INFO(logger, "Planning pipeline: %s", gripper_group_interface.getPlanningPipelineId().c_str());
    RCLCPP_INFO(logger, "Planner ID: %s", gripper_group_interface.getPlannerId().c_str());
    RCLCPP_INFO(logger, "Planning time: %.2f", gripper_group_interface.getPlanningTime());

    //the initial position (right before the tracing)
    std::vector<double> preferred_joints = {
        -101.0 * M_PI / 180.0,
        -115.0 * M_PI / 180.0,
        -40.0 * M_PI / 180.0,
        -205.0 * M_PI / 180.0,
        -101.0 * M_PI / 180.0,
        -180.0 * M_PI / 180.0
    };

    //sets the joint val as a target but doesnt move yet
    gripper_group_interface.setJointValueTarget(preferred_joints);
    
    moveit::planning_interface::MoveGroupInterface::Plan home_plan;
    //msg type to set a pose
    geometry_msgs::msg::Pose target_pose;
  
    //for the obect created before, call plan which saves the trajectory to reach preferred_joints to home_plan address
    //not a straight line, arbitrary trajectory
    auto ok = static_cast<bool>(gripper_group_interface.plan(home_plan));

    //after conversion to bool we see if position is reachable
    if (!ok) {
        RCLCPP_ERROR(logger, "Phase 1 planning failed!");
        rclcpp::shutdown();
        return 1;
    }

    //if it is reachable, we can and do move to a target joint position
    gripper_group_interface.execute(home_plan);

    //create a vector that contains the set of "waypoints"
    std::vector<geometry_msgs::msg::Pose> target_poses;
    
    //for now its hardcoded as we dont have sensors yet 
    //left top corner
    target_pose.position.x = -0.25;
    target_pose.position.y = 0.60;
    target_pose.position.z = 0.5;
    target_pose.orientation.x = - 1 / sqrt(2);
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 1 / sqrt(2);
    //add this pose to our vector
    target_poses.push_back(target_pose);

    int i = 1;
    //x+0.25 is a right border of the cube and z-0.08 id the closes we can get to the ground 
    //so the daigonal from top left to bottom right
    //0.02 allowed error
    while((abs(target_pose.position.x + 0.25) >= 0.02) || (abs(target_pose.position.z - 0.08) >= 0.02)){

        //a msg type from moveit
        moveit_msgs::msg::RobotTrajectory trajectory;
        //cartesian allows linear movement, no curves
        //computeCartesianPath(vector of the poses, step at every mm, trajectory, avoid colisions?)
        //no movement
        double fraction = gripper_group_interface.computeCartesianPath(target_poses, 0.01, trajectory, true);
        
        RCLCPP_INFO(logger, "Cartesian Path coverage: %.2f%%", fraction * 100.00);

        //Full Cartesian path achieved
        if (fraction >= 0.9) {
            moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
            //so our new trajectory which was calculated in computeCartesianPath is now stored in new cartesian_plan
            cartesian_plan.trajectory = trajectory;
            //now we can and do 
            gripper_group_interface.execute(cartesian_plan);
        }
        else {
            RCLCPP_ERROR(logger, "Cartesian path only %.1f%% complete — collision likely blocked it!", fraction * 100.0);
            return 1;
        }

        //remove the pose that was just executed, now we have empty array again
        target_poses.pop_back();

        //every 2 new waypoints the arm will either go to the other side of the cube or move 1 mm down
        if (i % 2 != 0){
            target_pose.position.x = -1 * target_pose.position.x;
        } else{
            target_pose.position.z = target_pose.position.z - 0.01;
        }

        //now we add a new trajactory to the vector
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

    RCLCPP_INFO(logger, "Cube Face Tracing complete. Returning to home position now...");
    gripper_group_interface.execute(home_plan);

    rclcpp::shutdown();
    return 0;
}