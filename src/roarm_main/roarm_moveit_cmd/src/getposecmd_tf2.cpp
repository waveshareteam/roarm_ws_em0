#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "roarm_moveit/srv/get_pose_cmd.hpp" // 包含你定义的服务头文件
#include <iostream>
#include <cmath>

geometry_msgs::msg::Pose hand_pose;  // 全局变量存储手的位姿

class RobotPosePublisher : public rclcpp::Node
{
public:
    RobotPosePublisher() : Node("robot_pose_publisher")
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    void update_end_effector_pose() 
    {
        try
        {
            geometry_msgs::msg::TransformStamped transformStamped;
            
            rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
            
            publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("hand_pose", 1);
            
            transformStamped = tf_buffer_->lookupTransform(base_frame, end_effector_frame, tf2::TimePointZero);
 
            hand_pose.position.x = transformStamped.transform.translation.x;
            hand_pose.position.y = transformStamped.transform.translation.y;
            hand_pose.position.z = transformStamped.transform.translation.z - 0.13381;
            publisher_->publish(hand_pose);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s", base_frame.c_str(), end_effector_frame.c_str(), ex.what());
        }
    }

private:
    std::string base_frame = "base_link";
    std::string end_effector_frame = "hand_tcp"; 
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

void handle_service(const std::shared_ptr<roarm_moveit::srv::GetPoseCmd::Request> request,
                    std::shared_ptr<roarm_moveit::srv::GetPoseCmd::Response> response,
                    std::shared_ptr<RobotPosePublisher> node)
{
    node->update_end_effector_pose();

    response->x = hand_pose.position.x;
    response->y = hand_pose.position.y;
    response->z = hand_pose.position.z;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotPosePublisher>();
    
    auto server = node->create_service<roarm_moveit::srv::GetPoseCmd>("get_pose_cmd", 
                    std::bind(&handle_service, std::placeholders::_1, std::placeholders::_2, node));

    RCLCPP_INFO(node->get_logger(), "Service is ready to receive requests.");

    rclcpp::spin(node);  

    rclcpp::shutdown(); 

    return 0;
}

