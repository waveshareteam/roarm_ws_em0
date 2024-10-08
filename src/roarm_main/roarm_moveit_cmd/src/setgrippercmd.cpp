#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class GripperActionClient : public rclcpp::Node
{
public:
    using GripperCommand = control_msgs::action::GripperCommand;
    using GoalHandleGripperCommand = rclcpp_action::ClientGoalHandle<GripperCommand>;

    GripperActionClient()
    : Node("gripper_action_client")
    {
        // 创建 Action 客户端
        this->action_client_ = rclcpp_action::create_client<GripperCommand>(this, "/gripper_controller/gripper_cmd");

        // 创建订阅者，订阅 /gripper_cmd 话题
        this->subscription_ = this->create_subscription<std_msgs::msg::Float32>(
            "/gripper_cmd", 10,
            std::bind(&GripperActionClient::listener_callback, this, std::placeholders::_1));
    }

private:
    rclcpp_action::Client<GripperCommand>::SharedPtr action_client_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;

    void listener_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        // 将接收到的 int32 值转换为浮点数 position
        double position = static_cast<double>(msg->data);
        RCLCPP_INFO(this->get_logger(), "Received position: %f", position);

        // 发送目标
        send_goal(position);
    }

    void send_goal(double position)
    {
        // 构建 GripperCommand 的 Goal 消息
        auto goal_msg = GripperCommand::Goal();
        goal_msg.command.position = position;
        goal_msg.command.max_effort = 10.0;  // 固定 max_effort

        // 等待 Action 服务启动
        if (!this->action_client_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available");
            return;
        }

        // 异步发送 Goal
        auto send_goal_options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&GripperActionClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.result_callback =
            std::bind(&GripperActionClient::result_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&GripperActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);

        this->action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void goal_response_callback(GoalHandleGripperCommand::SharedPtr goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_INFO(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void result_callback(const GoalHandleGripperCommand::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
        }
        RCLCPP_INFO(this->get_logger(), "Result received: position = %f", result.result->position);
    }

    void feedback_callback(GoalHandleGripperCommand::SharedPtr,
                           const std::shared_ptr<const GripperCommand::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Received feedback: %f", feedback->position);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GripperActionClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

