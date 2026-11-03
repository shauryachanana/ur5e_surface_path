#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometry_msgs/msg/pose.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <fstream>
#include <cmath>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("reachability",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto mesh_name = node->get_parameter_or<std::string>("mesh_filename", "ect_hf.stl");
  double liftoff = node->get_parameter_or<double>("liftoff_m", 0.002);
  int yaws = node->get_parameter_or<int>("yaw_samples", 8);
  double tool_sign = node->get_parameter_or<double>("tool_z_sign", 1.0);
  std::string sha = node->get_parameter_or<std::string>("stl_sha256", "unknown");

  moveit::planning_interface::MoveGroupInterface mg(node, "ur_manipulator");
  auto model = mg.getRobotModel();
  auto jmg = model->getJointModelGroup("ur_manipulator");
  planning_scene::PlanningScene scene(model);
  moveit::core::RobotState state(model);
  state.setToDefaultValues();

  std::string path = "file://" + ament_index_cpp::get_package_share_directory("ur5e_surface_path")
                     + "/meshes/" + mesh_name;
  shapes::Mesh* mesh = shapes::createMeshFromResource(path);

  std::ofstream out("reachability.csv");
  out << "# stl=" << mesh_name << " sha256=" << sha << " triangles=" << mesh->triangle_count
      << " liftoff_m=" << liftoff << " yaw_samples=" << yaws << " tool_z_sign=" << tool_sign << "\n";
  out << "triangle_id,reachable\n";

  for (unsigned i = 0; i < mesh->triangle_count; ++i) {
    double c[3] = {0, 0, 0};
    for (int k = 0; k < 3; ++k) {
      unsigned v = mesh->triangles[i*3 + k];
      for (int a = 0; a < 3; ++a) c[a] += mesh->vertices[v*3 + a] / 3.0;
    }
    tf2::Vector3 n(mesh->triangle_normals[i*3], mesh->triangle_normals[i*3+1], mesh->triangle_normals[i*3+2]);
    n.normalize();
    tf2::Vector3 z = n * tool_sign;
    tf2::Vector3 up(0, 0, -1);
    if (std::abs(z.dot(up)) > 0.99) up = tf2::Vector3(0, 1, 0);
    tf2::Vector3 x0 = up.cross(z).normalized();
    tf2::Vector3 y0 = z.cross(x0).normalized();

    bool ok = false;
    for (int s = 0; s < yaws && !ok; ++s) {
      double th = 2 * M_PI * s / yaws;
      tf2::Vector3 x = x0 * std::cos(th) + y0 * std::sin(th);
      tf2::Vector3 y = z.cross(x).normalized();
      tf2::Matrix3x3 R(x.x(), y.x(), z.x(), x.y(), y.y(), z.y(), x.z(), y.z(), z.z());
      tf2::Quaternion q; R.getRotation(q); q.normalize();
      geometry_msgs::msg::Pose p;
      // same transform as trace_cube.cpp (UNCONFIRMED Y), plus lift-off along the normal
      p.position.x = c[0]*0.001 + n.x()*liftoff;
      p.position.y = 1.0 - c[1]*0.001 - 0.1 + n.y()*liftoff;
      p.position.z = c[2]*0.001 + n.z()*liftoff;
      p.orientation.x = q.x(); p.orientation.y = q.y(); p.orientation.z = q.z(); p.orientation.w = q.w();
      if (state.setFromIK(jmg, p, "tool0", 0.02) && state.satisfiesBounds()) {
        collision_detection::CollisionRequest req; collision_detection::CollisionResult res;
        scene.checkCollision(req, res, state);
        ok = !res.collision;
      }
    }
    out << i << "," << (ok ? 1 : 0) << "\n";
  }
  rclcpp::shutdown();
  return 0;
}
