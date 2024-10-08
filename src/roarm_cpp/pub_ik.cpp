// Node to publish the joint angles by calculating the ik. Run this with serial_ctrl node to update both simulation and the hardware.
// The msg can be sent from the command line or using another node over the topic "coordinates_cpp"  

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/header.hpp>
#include <cmath>
#include <vector>

using std::placeholders::_1;

double x = 0.0;
double y = 0.0;
double z = 0.49075;

class JointStatePublisher : public rclcpp::Node
{
public:
    JointStatePublisher()
    : Node("pub_joint_state")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
            "coordinates_cpp", 10, std::bind(&JointStatePublisher::xyz_callback, this, _1));
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&JointStatePublisher::timer_callback, this));

        joint_state_msg_ = sensor_msgs::msg::JointState();
    }

private:
    void xyz_callback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        if (0.0 <= msg->x && msg->x <= 0.4 && -0.4 <= msg->y && msg->y <= 0.4 && 
            -0.17803 <= msg->z && msg->z <= 0.49 && std::sqrt(msg->x * msg->x + msg->y * msg->y + msg->z * msg->z) <= 0.49)
        {
            x = msg->x;
            y = msg->y;
            z = msg->z;
            RCLCPP_INFO(this->get_logger(), "Valid set of points");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "The given coordinates are out of the workspace");
        }
    }

    void timer_callback()
    {
        double theta1 = std::atan2(y, x);

        double t1 = theta1;

        // Transformation matrix for joint 1
        std::vector<std::vector<double>> T1 = {
            {std::cos(t1), std::sin(t1), 0, 0},
            {-std::sin(t1), std::cos(t1), 0, 0},
            {0, 0, 1, 0},
            {0, 0, 0, 1}
        };

        // Apply transformation matrix T1 to the end-effector position (x, y, z)
        std::vector<double> end_effector_pos = {x, y, z - 0.03796, 1.0};
        std::vector<double> transformed_pos(4, 0);
        for (size_t i = 0; i < 4; ++i)
        {
            for (size_t j = 0; j < 4; ++j)
            {
                transformed_pos[i] += T1[i][j] * end_effector_pos[j];
            }
        }

        double x_transformed = transformed_pos[0];
        double z_transformed = transformed_pos[2];

        auto inverse_kinematics = [](double x_1, double y_1, double l1, double l2) {
            double cos_theta3 = (x_1 * x_1 + y_1 * y_1 - l1 * l1 - l2 * l2) / (2 * l1 * l2);
            double sin_theta3_1 = std::sqrt(1 - cos_theta3 * cos_theta3);
            double sin_theta3_2 = -std::sqrt(1 - cos_theta3 * cos_theta3);

            double theta2_1 = std::atan2(y_1, x_1) - std::atan2(l2 * sin_theta3_1, l1 + l2 * cos_theta3);
            double theta2_2 = std::atan2(y_1, x_1) - std::atan2(l2 * sin_theta3_2, l1 + l2 * cos_theta3);

            double theta3_1 = std::atan2(sin_theta3_1, cos_theta3);
            double theta3_2 = std::atan2(sin_theta3_2, cos_theta3);

            return std::make_tuple(theta2_1, theta3_1, theta2_2, theta3_2);
        };

        // Constants
        double l1 = std::sqrt(0.23682*0.23682 + 0.03*0.03);
        double l2 = 0.21599;
        double x_1 = x_transformed;
        double y_1 = z_transformed;

        auto [theta2_1, theta3_1, theta2_2, theta3_2] = inverse_kinematics(x_1, y_1, l1, l2);
        
        double theta2[] = {M_PI / 2 - (theta2_1 + std::atan2(0.03, 0.23682)), M_PI / 2 - (theta2_2 + std::atan2(0.03, 0.23682))};
        double theta3[] = {-(theta3_1 - std::atan2(0.03, 0.23682)), -(theta3_2 - std::atan2(0.03, 0.23682))};
        
        double theta_final_2 = 0.0;
        double theta_final_3 = 0.0;
        double theta_4 = 0.0;
        for (int i = 0; i < 2; ++i) {
          if (theta2[i] < -1.570796 || theta2[i] > 1.570796)
            continue;
          else
            theta_final_2 = theta2[i];

          if (theta3[i] < -0.78539815 || theta3[i] > 3.1415926)
            continue;
          else
            theta_final_3 = theta3[i];
        }
        theta_4 = -theta_final_2 - theta_final_3;

        joint_state_msg_.header.stamp = this->now();
        joint_state_msg_.name = {"base_to_L1", "L1_to_L2", "L2_to_L3", "L3_to_L4"};
        joint_state_msg_.position = {theta1, theta_final_2, theta_final_3, theta_4};
        joint_state_msg_.velocity = {0.0, 0.0, 0.0, 0.0};
        joint_state_msg_.effort = {0.0, 0.0, 0.0, 0.0};

        publisher_->publish(joint_state_msg_);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::JointState joint_state_msg_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStatePublisher>());
    rclcpp::shutdown();
    return 0;
}
