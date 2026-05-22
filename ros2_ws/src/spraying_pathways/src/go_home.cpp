#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "rclcpp/parameter_client.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <thread>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("go_home_node");

  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);
  std::thread executor_thread([&executor]() { executor->spin(); });

  double max_vel = node->declare_parameter("max_velocity", 0.1);
  double max_acc = node->declare_parameter("max_acceleration", 0.05);
  double plan_time = node->declare_parameter("planning_time", 10.0);

  // === Load kinematics.yaml and set parameters ===
  std::string kinematics_path = ament_index_cpp::get_package_share_directory("ur_moveit_config") + "/config/kinematics.yaml";
  YAML::Node yaml = YAML::LoadFile(kinematics_path);
  if (yaml["/**"] && yaml["/**"]["ros__parameters"] && yaml["/**"]["ros__parameters"]["robot_description_kinematics"]) {
    auto kinematics = yaml["/**"]["ros__parameters"]["robot_description_kinematics"];
    for (auto group : kinematics) {
      std::string group_name = group.first.as<std::string>();
      for (auto param : group.second) {
        std::string param_name = "robot_description_kinematics." + group_name + "." + param.first.as<std::string>();
        auto value = param.second;
        if (value.IsScalar()) {
          if (value.Tag() == "!!str" || value.IsSequence()) {
            node->declare_parameter(param_name, value.as<std::string>());
          } else if (value.IsScalar()) {
            try {
              node->declare_parameter(param_name, value.as<double>());
            } catch (...) {
              try {
                node->declare_parameter(param_name, value.as<int>());
              } catch (...) {
                node->declare_parameter(param_name, value.as<std::string>());
              }
            }
          }
        }
      }
    }
  }

  moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");
  move_group.setMaxVelocityScalingFactor(max_vel);
  move_group.setMaxAccelerationScalingFactor(max_acc);
  move_group.setPlanningTime(plan_time);
  RCLCPP_INFO(node->get_logger(), "Planning to home: vel=%.2f, acc=%.2f, time=%.1fs", max_vel, max_acc, plan_time);
  std::vector<double> home_joint_values = {0.0, -2.15, 2.15, -1.57, -1.57, 0.0};
  move_group.setJointValueTarget(home_joint_values);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto plan_result = move_group.plan(plan);

  if (plan_result == moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_INFO(node->get_logger(), "Plan found. Executing...");
    auto exec_result = move_group.execute(plan);
    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS)
      RCLCPP_ERROR(node->get_logger(), "Execution failed.");
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Planning failed.");
  }
  executor->cancel();
  executor_thread.join();
  rclcpp::shutdown();
  return 0;
}