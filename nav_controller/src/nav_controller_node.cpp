#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "actionlib_msgs/msg/goal_status.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "known_locations_interfaces/srv/locations_get.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

class NavControllerNode : public rclcpp::Node
{
public:
  NavControllerNode() : Node("nav_controller_node")
  {

    // Topics internos del action
    status_topic_ = "navigation/status";
    goal_topic_   = "nav_control/goal";
    locations_get_srv_ = "locations_manager/get";

    // Sub al status del action server
    status_sub_ = this->create_subscription<actionlib_msgs::msg::GoalStatus>(
      status_topic_, rclcpp::QoS(10),
      std::bind(&NavControllerNode::nav_status_cb, this, std::placeholders::_1)
    );

    // Pub al goal del action server
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      goal_topic_, rclcpp::QoS(10)
    );

    // Client al servicio known_locations/get
    location_get_client_ = this->create_client<known_locations_interfaces::srv::LocationsGet>(
      locations_get_srv_
    );

    

    // Servicio 1: recibe string (location_name), llama known_locations/get y publica goal
    go_to_location_srv_ = this->create_service<known_locations_interfaces::srv::LocationsGet>(
      "nav_controller/go_to_location",
      std::bind(&NavControllerNode::handle_go_to_location, this, std::placeholders::_1, std::placeholders::_2)
    );

    RCLCPP_INFO(this->get_logger(), "nav_controller_node up.");
    RCLCPP_INFO(this->get_logger(), "Sub status: %s", status_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Pub goal:   %s", goal_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Locations get:  %s", locations_get_srv_.c_str());
  }

private:

  // ===== Status callback =====
void nav_status_cb(const actionlib_msgs::msg::GoalStatus::SharedPtr msg)
{
  last_status_code_ = msg->status;

  if (!waiting_nav_result_) {
    return;
  }

  if (msg->status == actionlib_msgs::msg::GoalStatus::SUCCEEDED) {
    nav_success_ = true;
    nav_done_ = true;
  }
  else if (msg->status == actionlib_msgs::msg::GoalStatus::ABORTED ||
           msg->status == actionlib_msgs::msg::GoalStatus::CANCELED) {
    nav_success_ = false;
    nav_done_ = true;
  }
}


  void go_to(const geometry_msgs::msg::PoseStamped & pose)
  {
    goal_pub_->publish(pose);
    RCLCPP_INFO(this->get_logger(), "Published goal: [%.2f, %.2f]", pose.pose.position.x, pose.pose.position.y);
  }

  // ===== Service handlers =====
  void handle_go_to_location(
    const std::shared_ptr<known_locations_interfaces::srv::LocationsGet::Request> req,
    std::shared_ptr<known_locations_interfaces::srv::LocationsGet::Response> resp)
  {
    resp->success = false;
    resp->message = "ERROR";

    if (req->location_name.empty()) {
      resp->message = "location_name is empty";
      return;
    }

    if (!location_get_client_->wait_for_service(1s)) {
      resp->message = "known_locations/get service not available";
      return;
    }

    auto loc_req = std::make_shared<known_locations_interfaces::srv::LocationsGet::Request>();
    loc_req->location_name = req->location_name;

    auto future = location_get_client_->async_send_request(loc_req);

    // Espera respuesta sin “re-spinear” raro
    auto ret = rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 2s);
    if (ret != rclcpp::FutureReturnCode::SUCCESS) {
      resp->message = "Timeout/failed calling known_locations/get";
      return;
    }

    const auto location_res = future.get();
    if (!location_res->success) {
      resp->message = "known_locations/get: " + location_res->message;
      return;
    }

    geometry_msgs::msg::PoseStamped goal_msg;
    goal_msg.header.stamp = this->now();
    goal_msg.header.frame_id = "map";
    goal_msg.pose = location_res->pose;

    // Publica al tópico del action goal
    goal_pub_->publish(goal_msg);

    // responde
    resp->pose = location_res->pose;
    resp->success = true;
    resp->message = "Goal published to " + goal_topic_;
    RCLCPP_INFO(this->get_logger(), "Go-to-known '%s' -> published goal.", req->location_name.c_str());
  }

  void handle_go_to_pose(
    const std::shared_ptr<known_locations_interfaces::srv::LocationsServer::Request> req,
    std::shared_ptr<known_locations_interfaces::srv::LocationsServer::Response> resp)
  {

  }


  // ===== Members =====
  std::string nav_action_name_;
  std::string locations_get_srv_;

  std::string status_topic_;
  std::string goal_topic_;

  rclcpp::Subscription<actionlib_msgs::msg::GoalStatus>::SharedPtr status_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;

  rclcpp::Client<known_locations_interfaces::srv::LocationsGet>::SharedPtr location_get_client_;

  rclcpp::Service<known_locations_interfaces::srv::LocationsGet>::SharedPtr go_to_location_srv_;
  rclcpp::Service<geometry_msgs::msg::PoseStamped>::SharedPtr go_to_pose_srv_;

  std::string last_status_text_{"NO_STATUS"};
  int last_status_code_{-1};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavControllerNode>());
  rclcpp::shutdown();
  return 0;
}
