#include <memory>
#include <cmath>
#include <vector>
#include <iostream>
#include <Eigen/Dense> // 导入Eigen库，用于多项式拟合
#include "roarm_moveit_cmd/ik.h"
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "roarm_moveit/srv/move_circle_cmd.hpp"

void generate_circle_trajectory(std::vector<geometry_msgs::msg::Pose>& waypoints, double center_x, double center_y, double radius, double resolution) {
    // 设置圆的分辨率，即每个弧度对应的线性分辨率
    double angle_increment = 2 * M_PI / resolution;

    // 遍历圆的360度，生成圆上的点并保存到轨迹点数组中
    for (double angle = 0; angle <= 2 * M_PI; angle += angle_increment) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = center_x + radius * cos(angle);
        pose.position.y = center_y + radius * sin(angle);

        waypoints.push_back(pose);
    }
}

void handle_service(const std::shared_ptr<roarm_moveit::srv::MoveCircleCmd::Request> request,
                    std::shared_ptr<roarm_moveit::srv::MoveCircleCmd::Response> response)
{
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("move_circle_cmd_service_node");
  auto logger = node->get_logger();

  // 创建 MoveIt MoveGroup 接口
  moveit::planning_interface::MoveGroupInterface move_group(node, "hand");

  // 创建圆的轨迹点
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose pose;

  double center_x = request->x;
  double center_y = -1*request->y;
  double center_z = request->z;
  double radius = request->radius;
  double resolution = 100; 
  generate_circle_trajectory(waypoints, center_x, center_y, radius, resolution);

  // 多项式拟合
  Eigen::MatrixXd A(waypoints.size(), 4);
  Eigen::VectorXd b_x(waypoints.size());
  Eigen::VectorXd b_y(waypoints.size());

  for (size_t i = 0; i < waypoints.size(); ++i) {
    double angle = 2 * M_PI * i / waypoints.size();
    A(i, 0) = 1;
    A(i, 1) = cos(angle);
    A(i, 2) = sin(angle);
    A(i, 3) = cos(2 * angle); // 为了使拟合更准确，可以添加更多的项

    b_x(i) = waypoints[i].position.x;
    b_y(i) = waypoints[i].position.y;
  }

  Eigen::VectorXd coeffs_x = A.colPivHouseholderQr().solve(b_x);
  Eigen::VectorXd coeffs_y = A.colPivHouseholderQr().solve(b_y);

  // 清除原有的圆上的点
  waypoints.clear();
  
  // 生成拟合后的圆上的点
  for (double angle = 0; angle <= 2 * M_PI; angle += 0.1) { // 步长可以自行调整
    geometry_msgs::msg::Pose pose;
    pose.position.x = coeffs_x(0) + coeffs_x(1) * cos(angle) + coeffs_x(2) * sin(angle) + coeffs_x(3) * cos(2 * angle);
    pose.position.y = coeffs_y(0) + coeffs_y(1) * cos(angle) + coeffs_y(2) * sin(angle) + coeffs_y(3) * cos(2 * angle);

    waypoints.push_back(pose);
  }

  // 循环处理每个目标点
  for (size_t i = 0; i < waypoints.size(); ++i) {
    geometry_msgs::msg::Pose target_pose = waypoints[i];

    // 这里调用 cartesian_to_polar 和 simpleLinkageIkRad 函数，计算关节角度
    double base_r, BASE_point_RAD;
    cartesian_to_polar(1000*target_pose.position.x,1000*target_pose.position.y, &base_r, &BASE_point_RAD);
    simpleLinkageIkRad(l2, l3, base_r, 1000*target_pose.position.z);

    // 将关节角度作为目标设置给机械臂
    std::vector<double> target = {BASE_point_RAD, -SHOULDER_point_RAD, ELBOW_point_RAD};
    move_group.setJointValueTarget(target);

    // 执行轨迹
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(success){
    move_group.execute(my_plan);
    response->success = true;
    response->message = "MoveCircleCmd executed successfully";
    RCLCPP_INFO(logger, "MoveCircleCmd service executed successfully");
    }else{
    response->success = false;
    response->message = "Planning failed!";
    RCLCPP_ERROR(logger, "Planning failed!");
    } 
  }

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("move_circle_cmd_service");

  // 创建服务
  auto server = node->create_service<roarm_moveit::srv::MoveCircleCmd>("move_circle_cmd", &handle_service);

  RCLCPP_INFO(node->get_logger(), "MoveCircleCmd service is ready.");

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}

