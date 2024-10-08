#include <chrono>
#include <thread>
#include <iomanip>
#include <sstream> 
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <std_msgs/msg/color_rgba.hpp>
#include "roarm_moveit/srv/get_pose_cmd.hpp" 
geometry_msgs::msg::Pose hand_pose;
rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;

void handle_service(const std::shared_ptr<roarm_moveit::srv::GetPoseCmd::Request> request,
                    std::shared_ptr<roarm_moveit::srv::GetPoseCmd::Response> response)
{
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("get_pose_cmd_service_node");
  std::stringstream ss_x, ss_y, ss_z;

  ss_x << std::fixed << std::setprecision(6) << hand_pose.position.x;
  ss_y << std::fixed << std::setprecision(6) << hand_pose.position.y;
  ss_z << std::fixed << std::setprecision(6) << (hand_pose.position.z - 0.13381);

  response->x = std::stof(ss_x.str());
  response->y = std::stof(ss_y.str());
  response->z = std::stof(ss_z.str());
  publisher_->publish(hand_pose);
}

int main(int argc, char** argv)
{
  using namespace std::chrono_literals;
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("get_pose_cmd_node", node_options);
  auto logger = node->get_logger();
  publisher_ = node->create_publisher<geometry_msgs::msg::Pose>("hand_pose", 1);
  auto server = node->create_service<roarm_moveit::srv::GetPoseCmd>("get_pose_cmd", &handle_service);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  moveit::planning_interface::MoveGroupInterface move_group_interface(node, "hand");

  // Create some helpful lambdas
  auto current_pose = move_group_interface.getCurrentPose();

  // Loop to continuously get and print the z-coordinate of the current pose
  while (rclcpp::ok())
  {
    current_pose = move_group_interface.getCurrentPose();
    hand_pose = current_pose.pose;
    //RCLCPP_INFO(logger, "Current pose z-coordinate: %f", current_pose.pose.position.z);
    // Add a delay to avoid high CPU usage
    
    std::this_thread::sleep_for(1s); // Sleep for 1 second
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
