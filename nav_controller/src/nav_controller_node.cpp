#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "actionlib_msgs/msg/goal_status.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "known_locations_interfaces/srv/locations_get.hpp"

using namespace std::chrono_literals;

class NavControllerNode : public rclcpp::Node
{
public:
  explicit NavControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("nav_controller_node", options)
  {
    // ===== Config =====
    status_topic_       = "navigation/status";
    goal_topic_         = "nav_control/goal";
    locations_get_srv_  = "locations_manager/get";

    // Callback groups para permitir concurrencia (servicio bloquea, sub sigue llegando)
    cbg_status_  = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    cbg_service_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // ===== Sub status =====
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = cbg_status_;

    status_sub_ = this->create_subscription<actionlib_msgs::msg::GoalStatus>(
      status_topic_, rclcpp::QoS(10),
      std::bind(&NavControllerNode::nav_status_cb, this, std::placeholders::_1),
      sub_opts
    );

    // ===== Pub goal =====
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic_, rclcpp::QoS(10));

    // ===== Client locations_get =====
    location_get_client_ = this->create_client<known_locations_interfaces::srv::LocationsGet>(locations_get_srv_);

    // ===== Service go_to_location =====
  

    go_to_location_srv_ = this->create_service<known_locations_interfaces::srv::LocationsGet>(
      "nav_controller/go_to_location",
      std::bind(&NavControllerNode::handle_go_to_location, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      cbg_service_
    );



    RCLCPP_INFO(this->get_logger(), "nav_controller_node up.");
    RCLCPP_INFO(this->get_logger(), "Sub status: %s", status_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Pub goal:   %s", goal_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Locations get: %s", locations_get_srv_.c_str());
  }

private:
  // ===== Status callback =====
  void nav_status_cb(const actionlib_msgs::msg::GoalStatus::SharedPtr msg)
  {
    // Guarda último status “global”
    {
      std::lock_guard<std::mutex> lk(nav_mtx_);
      last_status_code_ = msg->status;
    }

    // Si no estamos esperando resultado, ignorar
    if (!waiting_nav_result_.load()) {
      return;
    }

    // Nota: esto asume que el status que llega corresponde al goal más reciente.
    bool done = false;
    bool ok = false;

    if (msg->status == actionlib_msgs::msg::GoalStatus::SUCCEEDED) {
      done = true;
      ok = true;
    } else if (msg->status == actionlib_msgs::msg::GoalStatus::ABORTED) {
      done = true;
      ok = false;
    }

    if (done) {
      {
        std::lock_guard<std::mutex> lk(nav_mtx_);
        nav_done_ = true;
        nav_success_ = ok;
      }
      nav_cv_.notify_all();
    }
  }

  // ===== Service handler: go_to_location =====
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
      resp->message = "locations_manager/get service not available";
      return;
    }

    // 1) Pedir pose al locations_get
    auto loc_req = std::make_shared<known_locations_interfaces::srv::LocationsGet::Request>();
    loc_req->location_name = req->location_name;

    auto future = location_get_client_->async_send_request(loc_req);

    // OJO: aquí NO bloqueas el executor; esto espera la respuesta del servicio de locations.
    // MultiThreadedExecutor lo maneja bien.
    auto status = future.wait_for(5s);
    if (status != std::future_status::ready) {
      resp->message = "Timeout/failed calling locations_manager/get";
      return;
    }

    auto location_res = future.get();
    if (!location_res->success) {
      resp->message = "locations_manager/get: " + location_res->message;
      return;
    }

    // 2) Preparar goal
    geometry_msgs::msg::PoseStamped goal_msg;
    goal_msg.header.stamp = this->now();
    goal_msg.header.frame_id = "map";
    goal_msg.pose = location_res->pose;

    // 3) Reset de estado de espera (antes de publicar)
    {
      std::lock_guard<std::mutex> lk(nav_mtx_);
      nav_done_ = false;
      nav_success_ = false;
      last_status_code_ = -1;
    }
    waiting_nav_result_.store(true);

    // 4) Publicar goal
    goal_pub_->publish(goal_msg);
    RCLCPP_INFO(this->get_logger(), "Goal published for '%s' [%.2f, %.2f]",
                req->location_name.c_str(),
                goal_msg.pose.position.x, goal_msg.pose.position.y);

    // 5) Esperar status final (bloqueante)
    const auto timeout = 120s;  // ajústalo
    bool finished = false;
    bool ok = false;

    {
      std::unique_lock<std::mutex> lk(nav_mtx_);
      finished = nav_cv_.wait_for(lk, timeout, [&]() { return nav_done_; });
      ok = nav_success_;
    }

    waiting_nav_result_.store(false);

    if (!finished) {
      resp->message = "Navigation timeout waiting for result";
      resp->success = false;
      resp->pose = location_res->pose;
      return;
    }

    // 6) Responder según resultado
    resp->pose = location_res->pose;
    resp->success = ok;
    resp->message = ok ? "Navigation SUCCEEDED" : "Navigation FAILED (ABORTED/CANCELED)";
  }

  // ===== Members =====
  std::string locations_get_srv_;
  std::string status_topic_;
  std::string goal_topic_;

  rclcpp::CallbackGroup::SharedPtr cbg_status_;
  rclcpp::CallbackGroup::SharedPtr cbg_service_;

  rclcpp::Subscription<actionlib_msgs::msg::GoalStatus>::SharedPtr status_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Client<known_locations_interfaces::srv::LocationsGet>::SharedPtr location_get_client_;

  rclcpp::Service<known_locations_interfaces::srv::LocationsGet>::SharedPtr go_to_location_srv_;

  // Estado navegación
  std::atomic<bool> waiting_nav_result_{false};
  std::mutex nav_mtx_;
  std::condition_variable nav_cv_;
  bool nav_done_{false};
  bool nav_success_{false};

  int last_status_code_{-1};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<NavControllerNode>();

  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}

