#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

#include <fstream>
#include <vector>
#include <math.h>
#include <chrono>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#define DEBUGGER
#define TEMPORARY

struct Triangle{
    int id;   // CHANGED: index in STL order, needed for visited.csv
    float x0, y0, z0;
    float x1, y1, z1;
    float x2, y2, z2;
    float centre_x, centre_y, centre_z;
    float normal_x, normal_y, normal_z;
};

void calculateTriangleNormal(Triangle& triangle, shapes::Mesh* mesh, int index){
    triangle.normal_x = mesh->triangle_normals[index*3 + 0];
    triangle.normal_y = mesh->triangle_normals[index*3 + 1];
    triangle.normal_z = mesh->triangle_normals[index*3 + 2];
}

int main(int argc, char** argv){
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
        -101.0 * M_PI / 180.0,
        -115.0 * M_PI / 180.0,
        -25.0 * M_PI / 180.0,
        -205.0 * M_PI / 180.0,
        -101.0 * M_PI / 180.0,
        -180.0 * M_PI / 180.0
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

    /*------------PROCESS STL FILE------------*/

    node->declare_parameter("mesh_filename", "bowl_surface.stl");
    std::string mesh_filename = node->get_parameter("mesh_filename").as_string();
    std::string mesh_path = "file://" +
        ament_index_cpp::get_package_share_directory("ur5e_surface_path") +
        "/meshes/" + mesh_filename;

    shapes::Mesh* mesh = shapes::createMeshFromResource(
        mesh_path
    );

    std::vector<Triangle> vectorOfTriangles;

    for(unsigned int i = 0; i< mesh->triangle_count; i++){
        int firstVertexOfTriangle = mesh->triangles[i*3 + 0];
        int secondVertexOfTriangle = mesh->triangles[i*3 + 1];
        int thirdVertexOfTriangle = mesh->triangles[i*3 + 2];

        Triangle triangle;
        triangle.id = i;   // CHANGED

        triangle.x0 = mesh->vertices[firstVertexOfTriangle * 3 + 0];
        triangle.y0 = mesh->vertices[firstVertexOfTriangle * 3 + 1];
        triangle.z0 = mesh->vertices[firstVertexOfTriangle * 3 + 2];

        triangle.x1 = mesh->vertices[secondVertexOfTriangle * 3 + 0];
        triangle.y1 = mesh->vertices[secondVertexOfTriangle * 3 + 1];
        triangle.z1 = mesh->vertices[secondVertexOfTriangle * 3 + 2];

        triangle.x2 = mesh->vertices[thirdVertexOfTriangle * 3 + 0];
        triangle.y2 = mesh->vertices[thirdVertexOfTriangle * 3 + 1];
        triangle.z2 = mesh->vertices[thirdVertexOfTriangle * 3 + 2];

        triangle.centre_x = (triangle.x0 + triangle.x1 + triangle.x2) / 3.0f;
        triangle.centre_y = (triangle.y0 + triangle.y1 + triangle.y2) / 3.0f;
        triangle.centre_z = (triangle.z0 + triangle.z1 + triangle.z2) / 3.0f;

        calculateTriangleNormal(triangle, mesh, i);

        vectorOfTriangles.push_back(triangle);
    }

    #ifdef TEMPORARY
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    rclcpp::sleep_for(std::chrono::seconds(1));

    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer->lookupTransform(
            "base_link",
            "tool0",
            tf2::TimePointZero
        );
    } catch (tf2::TransformException& ex) {
        RCLCPP_ERROR(logger, "TF failed: %s", ex.what());
        rclcpp::shutdown();
        return 1;
    }

    double currentTCP[3] = {
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z
    };

    RCLCPP_WARN(logger, "TCP x: %f", currentTCP[0]);
    RCLCPP_WARN(logger, "TCP y: %f", currentTCP[1]);
    RCLCPP_WARN(logger, "TCP z: %f", currentTCP[2]);
    #endif

    #ifndef TEMPORARY
    geometry_msgs::msg::PoseStamped tcp_pose = gripper_group_interface.getCurrentPose();
    double currentTCP[3] = {tcp_pose.pose.position.x, tcp_pose.pose.position.y, tcp_pose.pose.position.z};
    #endif

    #ifdef DEBUGGER
    RCLCPP_WARN(logger, "x: %2f", vectorOfTriangles[0].centre_x);
    RCLCPP_WARN(logger, "y: %2f", vectorOfTriangles[0].centre_y);
    RCLCPP_WARN(logger, "z: %2f", vectorOfTriangles[0].centre_z);
    #endif

    /* CHANGED: take ALL triangles, in STL order. The old "same normal as the
       closest triangle" filter was made for a flat cube face and keeps almost
       nothing on a bowl. NOTE: STL order is not a continuous sweep yet. */
    std::vector<Triangle> sameSideTriangles = vectorOfTriangles;

    #ifdef DEBUGGER
    RCLCPP_WARN(logger, "triangles to visit: %zu", sameSideTriangles.size());
    #endif
    /*----------------------------------------*/

    moveit_msgs::msg::RobotTrajectory trajectory;

    // CHANGED: log every triangle so coverage_report.py can use it
    std::ofstream visited("visited.csv");
    visited << "triangle_id,fraction,executed\n";

    /*----------START THE OPERATION----------*/

    for(unsigned int i = 0; i < sameSideTriangles.size(); i++){
        RCLCPP_WARN(logger, "start computation number %d", i);
        target_pose.position.x = sameSideTriangles[i].centre_x * 0.001f;
        target_pose.position.y = 1.0f - sameSideTriangles[i].centre_y * 0.001f - 0.1f; // UNCONFIRMED Y transform
        target_pose.position.z = sameSideTriangles[i].centre_z * 0.001f;

        #ifndef TEMPORARY
        target_pose.orientation.x = - 1 / sqrt(2);
        target_pose.orientation.y = 0.0;
        target_pose.orientation.z = 0.0;
        target_pose.orientation.w = 1 / sqrt(2);
        #endif
        #ifdef TEMPORARY
        tf2::Vector3 normal(
            sameSideTriangles[i].normal_x,
            sameSideTriangles[i].normal_y,
            sameSideTriangles[i].normal_z
        );
        normal.normalize();
        tf2::Vector3 z_axis = normal;

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
        #endif
        target_poses.push_back(target_pose);

        double fraction = gripper_group_interface.computeCartesianPath(target_poses, 0.01, trajectory, true);

        #ifdef DEBUGGER
        RCLCPP_WARN(logger, "Cartesian Path coverage: %.2f%%", fraction * 100.00);
        #endif

        // CHANGED: 1.0 instead of 0.9, log instead of abort
        bool done = fraction >= 1.0;
        visited << sameSideTriangles[i].id << "," << fraction << "," << (done ? 1 : 0) << "\n";
        visited.flush();

        if (done) {
            moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
            cartesian_plan.trajectory = trajectory;
            gripper_group_interface.execute(cartesian_plan);
        }
        else {
            RCLCPP_ERROR(logger, "Triangle %d: Cartesian path only %.1f%% complete, skipped",
                         sameSideTriangles[i].id, fraction * 100.0);
            // CHANGED: no "return 1" here
        }

        target_poses.pop_back();
    }
    /*---------------------------------------*/

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

    RCLCPP_INFO(logger, "Tracing complete. Returning to home position now...");
    gripper_group_interface.execute(home_plan);

    rclcpp::shutdown();
    return 0;
}
