#pragma once

#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <optional>
#include <deque>
#include <atomic>
#include <mutex>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "warehouse_msgs/msg/task_status.hpp"
#include "warehouse_msgs/msg/robot_state.hpp"

namespace warehouse_navigation {

enum class NavigationState : uint8_t {
    IDLE = 0,
    NAVIGATING,
    GOAL_REACHED,
    GOAL_CANCELLED,
    GOAL_FAILED,
    PAUSED,
    WAITING_FOR_SERVER,
};

struct NavigationGoal {
    std::string goal_id;
    geometry_msgs::msg::PoseStamped pose;
    std::string robot_id;
    int priority{5};
    bool is_charging_goal{false};
    double timeout_seconds{120.0};
};

class NavigationController : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    explicit NavigationController(
        const std::string& robot_id,
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
    );
    ~NavigationController() override = default;

    bool send_goal(
        const geometry_msgs::msg::PoseStamped& goal_pose,
        const std::string& goal_id,
        bool is_charging_goal = false,
        double timeout_seconds = 120.0
    );

    bool cancel_current_goal();
    bool pause_navigation();
    bool resume_navigation();

    void enqueue_waypoint(const NavigationGoal& goal);
    void clear_queue();
    void process_next_in_queue();

    NavigationState get_navigation_state() const noexcept {
        return navigation_state_.load();
    }
    geometry_msgs::msg::Pose2D get_current_pose() const;
    double get_distance_remaining() const noexcept {
        return distance_remaining_.load();
    }
    const std::string& get_robot_id() const noexcept {
        return robot_id_;
    }

    static geometry_msgs::msg::PoseStamped make_pose_stamped(
        double x, double y, double theta,
        const std::string& frame_id = "map"
    );

private:
    std::string robot_id_;
    std::string nav2_namespace_;

    std::atomic<NavigationState> navigation_state_{NavigationState::IDLE};
    std::atomic<double> distance_remaining_{0.0};

    mutable std::mutex pose_mutex_;
    geometry_msgs::msg::Pose2D current_pose_{};

    std::optional<NavigationGoal> current_goal_;
    std::optional<NavigationGoal> paused_goal_;
    std::deque<NavigationGoal> waypoint_queue_;
    mutable std::mutex queue_mutex_;

    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
    GoalHandle::SharedPtr current_goal_handle_;
    std::mutex goal_handle_mutex_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<warehouse_msgs::msg::TaskStatus>::SharedPtr task_status_pub_;
    rclcpp::TimerBase::SharedPtr timeout_timer_;

    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);
    void goal_response_callback(const GoalHandle::SharedPtr& goal_handle);
    void feedback_callback(
        GoalHandle::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback>& feedback
    );
    void result_callback(const GoalHandle::WrappedResult& result);

    void arm_timeout(double seconds);
    void disarm_timeout();
    void on_timeout();
    void publish_task_status(const std::string& state, float progress, bool completed, bool failed);

    std::string robot_id_to_namespace(const std::string& robot_id) const;
    static double distance_2d(const geometry_msgs::msg::Pose2D& a,
                               const geometry_msgs::msg::Pose2D& b) noexcept;
};

}  // namespace warehouse_navigation
