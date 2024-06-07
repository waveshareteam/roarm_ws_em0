#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/logger.hpp"

using TwistMsg = geometry_msgs::msg::Twist;
using TwistStampedMsg = geometry_msgs::msg::TwistStamped;
using TwistSub = rclcpp::Subscription<TwistMsg>::SharedPtr;
using TwistPub = rclcpp::Publisher<TwistStampedMsg>::SharedPtr;

class TwistToTwistStamped : public rclcpp::Node
{
public:
    TwistToTwistStamped() : Node("twist_to_twist_stamped")
    {
        twist_sub = this->create_subscription<TwistMsg>(
            "/webappcontrol", 10, std::bind(&TwistToTwistStamped::twist_callback, this, std::placeholders::_1));
        twist_pub = this->create_publisher<TwistStampedMsg>("/servo_node/delta_twist_cmds", 10);

        RCLCPP_INFO(this->get_logger(), "Node initialized and ready to receive Twist messages.");
    }

private:
    void twist_callback(const std::shared_ptr<TwistMsg> msg)
    {
        auto twist_stamped_msg = std::make_shared<TwistStampedMsg>();
        twist_stamped_msg->header.stamp = this->now();
        twist_stamped_msg->header.frame_id = "base_link"; // Example frame_id
        twist_stamped_msg->twist = *msg;

        twist_pub->publish(*twist_stamped_msg); // Pass a const reference to the message

        RCLCPP_DEBUG(this->get_logger(), "Published TwistStamped message with twist: [%f, %f, %f, %f, %f, %f]",
                     twist_stamped_msg->twist.linear.x, twist_stamped_msg->twist.linear.y, twist_stamped_msg->twist.linear.z,
                     twist_stamped_msg->twist.angular.x, twist_stamped_msg->twist.angular.y, twist_stamped_msg->twist.angular.z);
    }

    TwistSub twist_sub;
    TwistPub twist_pub;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto twist_to_twist_stamped = std::make_shared<TwistToTwistStamped>();
    rclcpp::spin(twist_to_twist_stamped);
    rclcpp::shutdown();
    
    return 0;
}

