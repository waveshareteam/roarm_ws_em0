#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <cmath>
#include <vector>

using std::placeholders::_1;

double t1 = 0.0;
double t2 = 0.0;
double t3 = 0.0;

double l1 = 0.23682;
double l2 = 0.28384;

double x = 0.0;
double y = 0.0;
double z = 0.0;
double y_ = 0.0;

class EndStatePublisher : public rclcpp::Node
{
public:
    EndStatePublisher()
    : Node("pub_end_state")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&EndStatePublisher::xyz_callback, this, _1));
    }

private:
    void xyz_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        t1 = msg->position[0];
        t2 = (M_PI / 2) - msg->position[1];
        t3 = msg->position[2];
        
        x = (l1 * std::cos(t2) + l2 * std::cos(t2 - t3)) * std::cos(t1);
        y = (l1 * std::cos(t2) + l2 * std::cos(t2 - t3)) * std::sin(t1);
        z = (l1 * std::sin(t2) + l2 * std::sin(t2 - t3)) + 0.03796;
        RCLCPP_INFO(this->get_logger(),
                    "x = %f, y = %f, z = %f",
                    x, y, z);

    }
    // rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    // rclcpp::TimerBase::SharedPtr timer_;
    // sensor_msgs::msg::JointState joint_state_msg_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EndStatePublisher>());
    rclcpp::shutdown();
    return 0;
}
