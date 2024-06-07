#include <cinttypes>
#include <memory>
#include <regex>
#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
#include "ros2web_interfaces/srv/http.hpp"

using json = nlohmann::json;
using HTTP = ros2web_interfaces::srv::HTTP;

rclcpp::Node::SharedPtr g_node = nullptr;

void handle_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<HTTP::Request> request,
    const std::shared_ptr<HTTP::Response> response)
{
  (void)request_header;
  
  std::string txt = request->query;
  std::regex re(R"(^a=(\d+)&b=(\d+)$)");
  std::smatch m;
  std::regex_match(txt, m, re);

  if (m.size() == 3)
  {
    int a = std::stoi(m[1]);
    int b = std::stoi(m[2]);
    int sum = a + b;

    RCLCPP_INFO(g_node->get_logger(), "%d+%d=%d", a, b, sum);

    json res;
    res["sum"] = sum;
    response->text = res.dump();
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  g_node = rclcpp::Node::make_shared("ros2web_example_cpp");
  auto server = g_node->create_service<HTTP>("http/get/add_two_ints", handle_service);
  rclcpp::spin(g_node);
  rclcpp::shutdown();
  g_node = nullptr;
  return 0;
}
