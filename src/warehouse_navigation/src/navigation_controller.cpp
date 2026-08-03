#include "warehouse_navigation/navigation_controller.hpp"

#include <chrono>
#include <sstream>
#include <stdexcept>

namespace warehouse_navigation {

NavigationController::NavigationController(
    const std::string& robot_id,
    const rclcpp::NodeOptions& options)
: Node("navigation_controller_" + robot_id, options),
  robot_id_(robot_id),
  nav2_namespace_(robot_id_to_namespace(robot_id))
{
    declare_parameter("nav2_timeout_sec", 5.0);
    declare_parameter("default_goal_timeout_sec", 120.0);
    declare_parameter("goal_reached_tolerance", 0.25);

    nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(
        this, nav2_namespace_ + "/navigate_to_pose"
    );

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        nav2_namespace_ + "/odom",
        rclcpp::QoS(10),
        [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
            odom_callback(msg);
        }
    );

    task_status_pub_ = create_publisher<warehouse_msgs::msg::TaskStatus>(
        "/warehouse/" + robot_id_ + "/task_status",
        rclcpp::QoS(10)
    );

    RCLCPP_INFO(get_logger(),
        "NavigationController started for robot '%s' (nav2_ns: '%s')",
        robot_id_.c_str(), nav2_namespace_.c_str());
}

bool NavigationController::send_goal(
    const geometry_msgs::msg::PoseStamped& goal_pose,
    const std::string& goal_id,
    bool is_charging_goal,
    double timeout_seconds)
{
    if (!nav_action_client_->wait_for_action_server(
            std::chrono::duration<double>(
                get_parameter("nav2_timeout_sec").as_double())))
    {
        RCLCPP_ERROR(get_logger(),
            "[%s] Nav2 action server unavailable (goal: %s)",
            robot_id_.c_str(), goal_id.c_str());
        navigation_state_.store(NavigationState::GOAL_FAILED);
        return false;
    }

    {
        std::lock_guard<std::mutex> lk(goal_handle_mutex_);
        if (current_goal_handle_ &&
            navigation_state_.load() == NavigationState::NAVIGATING)
        {
            nav_action_client_->async_cancel_goal(current_goal_handle_);
        }
    }

    NavigationGoal nav_goal;
    nav_goal.goal_id         = goal_id;
    nav_goal.pose            = goal_pose;
    nav_goal.robot_id        = robot_id_;
    nav_goal.is_charging_goal = is_charging_goal;
    nav_goal.timeout_seconds = timeout_seconds;
    current_goal_             = nav_goal;

    auto goal_msg = NavigateToPose::Goal{};
    goal_msg.pose = goal_pose;

    auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions{};
    opts.goal_response_callback = [this](const GoalHandle::SharedPtr& gh) {
        goal_response_callback(gh);
    };
    opts.feedback_callback = [this](
        GoalHandle::SharedPtr gh,
        const std::shared_ptr<const NavigateToPose::Feedback> fb)
    {
        feedback_callback(gh, fb);
    };
    opts.result_callback = [this](const GoalHandle::WrappedResult& res) {
        result_callback(res);
    };

    navigation_state_.store(NavigationState::NAVIGATING);
    nav_action_client_->async_send_goal(goal_msg, opts);
    arm_timeout(timeout_seconds);

    RCLCPP_INFO(get_logger(),
        "[%s] Navigating to '%s' (%.2f, %.2f)",
        robot_id_.c_str(), goal_id.c_str(),
        goal_pose.pose.position.x, goal_pose.pose.position.y);

    publish_task_status("NAVIGATING", 0.0f, false, false);
    return true;
}

bool NavigationController::cancel_current_goal() {
    std::lock_guard<std::mutex> lk(goal_handle_mutex_);
    if (!current_goal_handle_) {
        return true;
    }
    nav_action_client_->async_cancel_goal(current_goal_handle_);
    navigation_state_.store(NavigationState::GOAL_CANCELLED);
    disarm_timeout();
    RCLCPP_INFO(get_logger(), "[%s] Goal cancelled", robot_id_.c_str());
    return true;
}

bool NavigationController::pause_navigation() {
    if (navigation_state_.load() != NavigationState::NAVIGATING) {
        return false;
    }
    paused_goal_ = current_goal_;
    cancel_current_goal();
    navigation_state_.store(NavigationState::PAUSED);
    RCLCPP_INFO(get_logger(), "[%s] Navigation paused", robot_id_.c_str());
    return true;
}

bool NavigationController::resume_navigation() {
    if (navigation_state_.load() != NavigationState::PAUSED || !paused_goal_) {
        return false;
    }
    auto goal = *paused_goal_;
    paused_goal_.reset();
    RCLCPP_INFO(get_logger(), "[%s] Resuming navigation to '%s'",
                robot_id_.c_str(), goal.goal_id.c_str());
    return send_goal(goal.pose, goal.goal_id, goal.is_charging_goal, goal.timeout_seconds);
}

void NavigationController::enqueue_waypoint(const NavigationGoal& goal) {
    std::lock_guard<std::mutex> lk(queue_mutex_);
    waypoint_queue_.push_back(goal);
}

void NavigationController::clear_queue() {
    std::lock_guard<std::mutex> lk(queue_mutex_);
    waypoint_queue_.clear();
}

void NavigationController::process_next_in_queue() {
    NavigationGoal next;
    {
        std::lock_guard<std::mutex> lk(queue_mutex_);
        if (waypoint_queue_.empty()) {
            navigation_state_.store(NavigationState::IDLE);
            return;
        }
        next = waypoint_queue_.front();
        waypoint_queue_.pop_front();
    }
    send_goal(next.pose, next.goal_id, next.is_charging_goal, next.timeout_seconds);
}

geometry_msgs::msg::Pose2D NavigationController::get_current_pose() const {
    std::lock_guard<std::mutex> lk(pose_mutex_);
    return current_pose_;
}

geometry_msgs::msg::PoseStamped NavigationController::make_pose_stamped(
    double x, double y, double theta, const std::string& frame_id)
{
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = frame_id;
    ps.pose.position.x = x;
    ps.pose.position.y = y;
    ps.pose.position.z = 0.0;
    ps.pose.orientation.z = std::sin(theta * 0.5);
    ps.pose.orientation.w = std::cos(theta * 0.5);
    return ps;
}

void NavigationController::odom_callback(
    const nav_msgs::msg::Odometry::ConstSharedPtr& msg)
{
    std::lock_guard<std::mutex> lk(pose_mutex_);
    current_pose_.x = msg->pose.pose.position.x;
    current_pose_.y = msg->pose.pose.position.y;
    const auto& q = msg->pose.pose.orientation;
    current_pose_.theta = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    );
}

void NavigationController::goal_response_callback(const GoalHandle::SharedPtr& gh) {
    std::lock_guard<std::mutex> lk(goal_handle_mutex_);
    if (!gh) {
        RCLCPP_ERROR(get_logger(), "[%s] Goal rejected by Nav2", robot_id_.c_str());
        navigation_state_.store(NavigationState::GOAL_FAILED);
        publish_task_status("FAILED", 0.0f, false, true);
        disarm_timeout();
        return;
    }
    current_goal_handle_ = gh;
    RCLCPP_DEBUG(get_logger(), "[%s] Goal accepted", robot_id_.c_str());
}

void NavigationController::feedback_callback(
    GoalHandle::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback>& feedback)
{
    distance_remaining_.store(feedback->distance_remaining);
    float progress = std::max(0.0f,
        std::min(0.95f, 1.0f - static_cast<float>(feedback->distance_remaining / 20.0)));
    publish_task_status("NAVIGATING", progress, false, false);
}

void NavigationController::result_callback(const GoalHandle::WrappedResult& result) {
    disarm_timeout();
    current_goal_handle_.reset();

    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            navigation_state_.store(NavigationState::GOAL_REACHED);
            RCLCPP_INFO(get_logger(), "[%s] Goal reached: %s",
                robot_id_.c_str(),
                current_goal_ ? current_goal_->goal_id.c_str() : "unknown");
            publish_task_status("COMPLETED", 1.0f, true, false);
            process_next_in_queue();
            break;

        case rclcpp_action::ResultCode::ABORTED:
            navigation_state_.store(NavigationState::GOAL_FAILED);
            RCLCPP_ERROR(get_logger(), "[%s] Navigation aborted", robot_id_.c_str());
            publish_task_status("FAILED", 0.0f, false, true);
            break;

        case rclcpp_action::ResultCode::CANCELED:
            if (navigation_state_.load() != NavigationState::PAUSED) {
                navigation_state_.store(NavigationState::GOAL_CANCELLED);
            }
            break;

        default:
            navigation_state_.store(NavigationState::GOAL_FAILED);
            publish_task_status("FAILED", 0.0f, false, true);
            break;
    }
}

void NavigationController::arm_timeout(double seconds) {
    disarm_timeout();
    timeout_timer_ = create_wall_timer(
        std::chrono::duration<double>(seconds),
        [this]() { on_timeout(); }
    );
}

void NavigationController::disarm_timeout() {
    if (timeout_timer_) {
        timeout_timer_->cancel();
        timeout_timer_.reset();
    }
}

void NavigationController::on_timeout() {
    if (navigation_state_.load() == NavigationState::NAVIGATING) {
        RCLCPP_WARN(get_logger(),
            "[%s] Navigation timeout — cancelling goal", robot_id_.c_str());
        cancel_current_goal();
        navigation_state_.store(NavigationState::GOAL_FAILED);
        publish_task_status("FAILED", 0.0f, false, true);
    }
}

void NavigationController::publish_task_status(
    const std::string& state, float progress, bool completed, bool failed)
{
    warehouse_msgs::msg::TaskStatus msg;
    msg.robot_id   = robot_id_;
    msg.task_type  = "NAVIGATE";
    msg.task_state = state;
    msg.goal       = current_goal_ ? current_goal_->goal_id : "";
    msg.progress   = static_cast<double>(progress);
    msg.completed  = completed;
    msg.failed     = failed;
    msg.update_time = get_clock()->now();
    task_status_pub_->publish(msg);
}

std::string NavigationController::robot_id_to_namespace(const std::string& robot_id) const {
    std::string ns = robot_id;
    std::transform(ns.begin(), ns.end(), ns.begin(), ::tolower);
    ns.erase(std::remove(ns.begin(), ns.end(), '_'), ns.end());
    return "/" + ns;
}

double NavigationController::distance_2d(
    const geometry_msgs::msg::Pose2D& a,
    const geometry_msgs::msg::Pose2D& b) noexcept
{
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    return std::sqrt(dx * dx + dy * dy);
}

}  // namespace warehouse_navigation

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    if (argc < 2) {
        RCLCPP_ERROR(rclcpp::get_logger("navigation_controller"),
            "Usage: navigation_controller <robot_id>  (e.g. Robot_1)");
        return EXIT_FAILURE;
    }

    auto node = std::make_shared<warehouse_navigation::NavigationController>(argv[1]);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
