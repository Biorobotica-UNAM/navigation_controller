#include <chrono>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "actionlib_msgs/msg/goal_status.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "known_locations_interfaces/srv/locations_get.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class NavControllerNode : public rclcpp::Node
{
public:
  explicit NavControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("nav_controller_node", options),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // ===== Config =====
    status_topic_       = "navigation/status";
    goal_topic_         = "nav_control/goal";
    locations_get_srv_  = "locations_manager/get";

    // Timeout policy for a complete semantic navigation request.
    // The controller still lets motion_planner/simple_move replan internally,
    // but it now bounds the whole operation using the robot-goal map distance.
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<int>("base_timeout_seconds", 20);
    this->declare_parameter<int>("min_timeout_seconds", 45);
    this->declare_parameter<int>("max_timeout_seconds", 240);
    this->declare_parameter<double>("expected_speed_mps", 0.25);
    this->declare_parameter<double>("timeout_safety_factor", 3.0);

    // Test-only hook for executor/supervisor error propagation checks.
    // Keep disabled in normal runs. When enabled, a reserved semantic
    // location can bypass locations_manager/get so tests can exercise the
    // terminal navigation statuses below without depending on stored map
    // locations. If test_location_publish_goal is false, the controller waits
    // for an injected /navigation/status instead of sending a real nav goal.
    this->declare_parameter<bool>("enable_test_location_bypass", false);
    this->declare_parameter<std::string>("test_location_name", "__nav_test__");
    this->declare_parameter<double>("test_location_x", 0.0);
    this->declare_parameter<double>("test_location_y", 0.0);
    this->declare_parameter<double>("test_location_z", 0.0);
    this->declare_parameter<bool>("test_location_publish_goal", true);

    this->get_parameter("map_frame", map_frame_);
    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("base_timeout_seconds", base_timeout_seconds_);
    this->get_parameter("min_timeout_seconds", min_timeout_seconds_);
    this->get_parameter("max_timeout_seconds", max_timeout_seconds_);
    this->get_parameter("expected_speed_mps", expected_speed_mps_);
    this->get_parameter("timeout_safety_factor", timeout_safety_factor_);
    this->get_parameter("enable_test_location_bypass", enable_test_location_bypass_);
    this->get_parameter("test_location_name", test_location_name_);
    this->get_parameter("test_location_x", test_location_x_);
    this->get_parameter("test_location_y", test_location_y_);
    this->get_parameter("test_location_z", test_location_z_);
    this->get_parameter("test_location_publish_goal", test_location_publish_goal_);

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
    stop_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/navigation/navigation_ctrl/stop",
      rclcpp::QoS(10));

    // ===== Client locations_get =====
    location_get_client_ = this->create_client<known_locations_interfaces::srv::LocationsGet>(locations_get_srv_);

    // ===== Service go_to_location =====
  

    go_to_location_srv_ = this->create_service<known_locations_interfaces::srv::LocationsGet>(
      "nav_controller/go_to_location",
      std::bind(&NavControllerNode::handle_go_to_location, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(),
      cbg_service_
    );



    RCLCPP_INFO(this->get_logger(), "nav_controller_node up.");
    RCLCPP_INFO(this->get_logger(), "Sub status: %s", status_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Pub goal:   %s", goal_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Locations get: %s", locations_get_srv_.c_str());
    if (enable_test_location_bypass_) {
      RCLCPP_WARN(
        this->get_logger(),
        "TEST MODE enabled: location '%s' bypasses %s and uses synthetic pose [%.2f, %.2f, %.2f]",
        test_location_name_.c_str(),
        locations_get_srv_.c_str(),
        test_location_x_,
        test_location_y_,
        test_location_z_);
      if (!test_location_publish_goal_) {
        RCLCPP_WARN(
          this->get_logger(),
          "TEST MODE: synthetic test location will not publish real navigation goals");
      }
    }
  }

private:
  static const char * status_name(int status)
  {
    switch (status) {
      case actionlib_msgs::msg::GoalStatus::PENDING:    return "PENDING";
      case actionlib_msgs::msg::GoalStatus::ACTIVE:     return "ACTIVE";
      case actionlib_msgs::msg::GoalStatus::PREEMPTED:  return "PREEMPTED";
      case actionlib_msgs::msg::GoalStatus::SUCCEEDED:  return "SUCCEEDED";
      case actionlib_msgs::msg::GoalStatus::ABORTED:    return "ABORTED";
      case actionlib_msgs::msg::GoalStatus::REJECTED:   return "REJECTED";
      case actionlib_msgs::msg::GoalStatus::PREEMPTING: return "PREEMPTING";
      case actionlib_msgs::msg::GoalStatus::RECALLING:  return "RECALLING";
      case actionlib_msgs::msg::GoalStatus::RECALLED:   return "RECALLED";
      case actionlib_msgs::msg::GoalStatus::LOST:       return "LOST";
      default:                                          return "UNKNOWN";
    }
  }

  static bool is_terminal_status(int status)
  {
    // GoalStatus comes from the lower navigation stack through
    // /navigation/status. The controller only completes its service when it
    // receives a terminal state. Non-terminal states, such as PENDING,
    // ACTIVE, PREEMPTING, and RECALLING, are progress/status updates.
    //
    // Terminal states handled here:
    // - SUCCEEDED: the robot reached the accepted navigation goal.
    // - ABORTED: execution started but navigation could not finish. Common
    //   causes are no valid path, repeated path-tracking failure, blocked
    //   goal, or a safety layer reporting that motion cannot continue.
    // - REJECTED: the navigation stack refused the goal before executing it,
    //   for example because the request was invalid for the planner.
    // - PREEMPTED: an active goal was cancelled/replaced by a newer goal or
    //   stop request after it had already started.
    // - RECALLED: a pending goal was cancelled before it became active.
    // - LOST: the lower-level action state can no longer be tracked; this is
    //   rare, but it is terminal because the controller cannot know whether
    //   the goal is still valid.
    //
    // Tests may inject the same statuses on /navigation/status. In test mode
    // prefer test_location_publish_goal=false to avoid racing real planner
    // statuses against the injected status.
    switch (status) {
      case actionlib_msgs::msg::GoalStatus::SUCCEEDED:
      case actionlib_msgs::msg::GoalStatus::PREEMPTED:
      case actionlib_msgs::msg::GoalStatus::ABORTED:
      case actionlib_msgs::msg::GoalStatus::REJECTED:
      case actionlib_msgs::msg::GoalStatus::RECALLED:
      case actionlib_msgs::msg::GoalStatus::LOST:
        return true;
      default:
        return false;
    }
  }

  bool get_current_pose(geometry_msgs::msg::Pose & pose, std::string & error_message)
  {
    try {
      auto tf_stamped = tf_buffer_.lookupTransform(
        map_frame_,
        base_frame_,
        tf2::TimePointZero,
        500ms);

      pose.position.x = tf_stamped.transform.translation.x;
      pose.position.y = tf_stamped.transform.translation.y;
      pose.position.z = tf_stamped.transform.translation.z;
      pose.orientation = tf_stamped.transform.rotation;
      error_message.clear();
      return true;
    } catch (const tf2::TransformException & ex) {
      std::ostringstream oss;
      oss << "cannot locate robot TF " << map_frame_ << " -> " << base_frame_ << ": " << ex.what();
      error_message = oss.str();
      RCLCPP_WARN(
        this->get_logger(),
        "Could not compute navigation distance from TF %s -> %s: %s",
        map_frame_.c_str(),
        base_frame_.c_str(),
        ex.what());
      return false;
    }
  }

  std::chrono::seconds compute_navigation_timeout(
    const geometry_msgs::msg::Pose & goal_pose,
    double & distance,
    std::string & error_message)
  {
    geometry_msgs::msg::Pose current_pose;
    if (!get_current_pose(current_pose, error_message)) {
      distance = -1.0;
      return std::chrono::seconds(0);
    }

    const double dx = goal_pose.position.x - current_pose.position.x;
    const double dy = goal_pose.position.y - current_pose.position.y;
    distance = std::sqrt(dx * dx + dy * dy);

    // Distance-based upper bound:
    // base + (straight-line distance / expected speed) * safety factor,
    // clamped to configured min/max limits. Straight-line distance is a lower
    // bound for path length, so the safety factor absorbs path curvature,
    // replanning and temporary obstacle waits.
    const double speed = std::max(expected_speed_mps_, 0.05);
    const double travel_seconds = (distance / speed) * timeout_safety_factor_;
    int timeout_seconds =
      static_cast<int>(std::ceil(static_cast<double>(base_timeout_seconds_) + travel_seconds));

    timeout_seconds = std::max(timeout_seconds, min_timeout_seconds_);
    timeout_seconds = std::min(timeout_seconds, max_timeout_seconds_);

    return std::chrono::seconds(timeout_seconds);
  }

  std::string build_status_message(const std::string & prefix, int status, const std::string & text)
  {
    std::ostringstream oss;
    oss << prefix << " [" << status_name(status) << "]";
    if (!text.empty()) {
      oss << ": " << text;
    }
    return oss.str();
  }

  bool should_use_test_location(const std::string & location_name) const
  {
    return enable_test_location_bypass_ && location_name == test_location_name_;
  }

  known_locations_interfaces::srv::LocationsGet::Response::SharedPtr build_test_location_response()
  {
    // Synthetic response used only by the test hook above. This deliberately
    // does not register or modify persistent locations.
    auto response = std::make_shared<known_locations_interfaces::srv::LocationsGet::Response>();
    response->success = true;
    response->message = "test location bypass";
    response->pose.position.x = test_location_x_;
    response->pose.position.y = test_location_y_;
    response->pose.position.z = test_location_z_;
    response->pose.orientation.w = 1.0;
    return response;
  }

  // ===== Status callback =====
  void nav_status_cb(const actionlib_msgs::msg::GoalStatus::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(nav_mtx_);
    last_status_code_ = msg->status;
    last_status_text_ = msg->text;

    if (!waiting_nav_result_.load() || !is_terminal_status(msg->status)) {
      return;
    }

    nav_done_ = true;
    nav_success_ = (msg->status == actionlib_msgs::msg::GoalStatus::SUCCEEDED);
    // Preserve the planner text so Navigation::GoToDetailed and GoTo can
    // forward the actionable reason to the executor/supervisor.
    nav_result_status_code_ = msg->status;
    nav_result_status_text_ = msg->text;
    nav_cv_.notify_all();
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

    known_locations_interfaces::srv::LocationsGet::Response::SharedPtr location_res;

    const bool using_test_location = should_use_test_location(req->location_name);

    if (using_test_location) {
      location_res = build_test_location_response();
      RCLCPP_WARN(
        this->get_logger(),
        "Using synthetic test location '%s'; %s was not called",
        req->location_name.c_str(),
        locations_get_srv_.c_str());
    } else {
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

      location_res = future.get();
      if (!location_res->success) {
        resp->message = "locations_manager/get: " + location_res->message;
        return;
      }
    }

    // 2) Preparar goal
    geometry_msgs::msg::PoseStamped goal_msg;
    goal_msg.header.stamp = this->now();
    goal_msg.header.frame_id = "map";
    goal_msg.pose = location_res->pose;

    // 3) Compute timeout before publishing the goal. If the robot TF is
    // missing, fail fast: without the current robot pose the controller cannot
    // estimate progress or produce a meaningful navigation timeout.
    double estimated_distance = -1.0;
    std::string timeout_error;
    const auto timeout = compute_navigation_timeout(
      location_res->pose,
      estimated_distance,
      timeout_error);

    if (!timeout_error.empty()) {
      resp->success = false;
      resp->pose = location_res->pose;
      resp->message = timeout_error;
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Navigation timeout for '%s': %ld s, distance %.2f m",
      req->location_name.c_str(),
      timeout.count(),
      estimated_distance);

    // 4) Reset de estado de espera (antes de publicar)
    {
      std::lock_guard<std::mutex> lk(nav_mtx_);
      nav_done_ = false;
      nav_success_ = false;
      last_status_code_ = -1;
      last_status_text_.clear();
      nav_result_status_code_ = -1;
      nav_result_status_text_.clear();
    }
    waiting_nav_result_.store(true);

    // 5) Publicar goal
    if (!using_test_location || test_location_publish_goal_) {
      goal_pub_->publish(goal_msg);
      RCLCPP_INFO(this->get_logger(), "Goal published for '%s' [%.2f, %.2f]",
                  req->location_name.c_str(),
                  goal_msg.pose.position.x, goal_msg.pose.position.y);
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "TEST MODE: not publishing nav_control/goal for synthetic location '%s'",
        req->location_name.c_str());
    }

    // 6) Esperar status final (bloqueante)
    bool finished = false;
    bool ok = false;
    int final_status_code = -1;
    std::string final_status_text;

    {
      std::unique_lock<std::mutex> lk(nav_mtx_);
      finished = nav_cv_.wait_for(lk, timeout, [&]() { return nav_done_; });
      ok = nav_success_;
      final_status_code = finished ? nav_result_status_code_ : last_status_code_;
      final_status_text = finished ? nav_result_status_text_ : last_status_text_;
    }

    waiting_nav_result_.store(false);

    if (!finished) {
      std_msgs::msg::Bool stop_msg;
      stop_msg.data = true;
      stop_pub_->publish(stop_msg);

      std::ostringstream oss;
      oss << "Navigation timeout after " << timeout.count() << " s";
      if (estimated_distance >= 0.0) {
        oss << " for distance " << estimated_distance << " m";
      }
      oss << ". Last status " << status_name(final_status_code);
      if (!final_status_text.empty()) {
        oss << ": " << final_status_text;
      }

      resp->message = oss.str();
      resp->success = false;
      resp->pose = location_res->pose;
      return;
    }

    // 7) Responder según resultado
    resp->pose = location_res->pose;
    resp->success = ok;
    resp->message = build_status_message(
      ok ? "Navigation SUCCEEDED" : "Navigation FAILED",
      final_status_code,
      final_status_text);
  }

  // ===== Members =====
  std::string locations_get_srv_;
  std::string status_topic_;
  std::string goal_topic_;
  std::string map_frame_;
  std::string base_frame_;

  int base_timeout_seconds_{20};
  int min_timeout_seconds_{45};
  int max_timeout_seconds_{240};
  double expected_speed_mps_{0.25};
  double timeout_safety_factor_{3.0};
  bool enable_test_location_bypass_{false};
  std::string test_location_name_{"__nav_test__"};
  double test_location_x_{0.0};
  double test_location_y_{0.0};
  double test_location_z_{0.0};
  bool test_location_publish_goal_{true};

  rclcpp::CallbackGroup::SharedPtr cbg_status_;
  rclcpp::CallbackGroup::SharedPtr cbg_service_;

  rclcpp::Subscription<actionlib_msgs::msg::GoalStatus>::SharedPtr status_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub_;
  rclcpp::Client<known_locations_interfaces::srv::LocationsGet>::SharedPtr location_get_client_;

  rclcpp::Service<known_locations_interfaces::srv::LocationsGet>::SharedPtr go_to_location_srv_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Estado navegación
  std::atomic<bool> waiting_nav_result_{false};
  std::mutex nav_mtx_;
  std::condition_variable nav_cv_;
  bool nav_done_{false};
  bool nav_success_{false};

  int last_status_code_{-1};
  std::string last_status_text_;
  int nav_result_status_code_{-1};
  std::string nav_result_status_text_;
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
