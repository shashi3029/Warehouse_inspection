#pragma once

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <mutex>
#include <cmath>
#include <algorithm>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "warehouse_msgs/msg/robot_state.hpp"

namespace warehouse_navigation {

enum class FormationType : uint8_t {
    SQUARE   = 0,
    DIAMOND  = 1,
    COLUMN   = 2,
    LINE     = 3,
    ESCORT   = 4,
    ADAPTIVE = 5,
};

struct FormationSlot {
    int   slot_index{0};
    double offset_x{0.0};
    double offset_y{0.0};
};

struct RobotPose {
    std::string robot_id;
    geometry_msgs::msg::Pose2D pose;
    bool valid{false};
    rclcpp::Time last_update;
};

class FormationController : public rclcpp::Node {
public:
    static constexpr int    MAX_ROBOTS       = 4;
    static constexpr double DEFAULT_SPACING  = 1.5;
    static constexpr double POSE_TIMEOUT_SEC = 2.0;

    explicit FormationController(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~FormationController() override = default;

    bool switch_formation(const std::string& formation_name);
    void set_spacing(double spacing) noexcept;
    void set_leader(const std::string& robot_id);

    FormationType get_formation() const noexcept { return current_formation_; }
    double get_spacing() const noexcept { return spacing_; }
    const std::string& get_leader() const noexcept { return leader_id_; }

    std::map<std::string, geometry_msgs::msg::Pose2D> compute_formation_goals() const;

private:
    FormationType current_formation_{FormationType::SQUARE};
    double spacing_{DEFAULT_SPACING};
    std::string leader_id_;
    std::vector<std::string> robot_ids_;

    mutable std::mutex pose_mutex_;
    std::map<std::string, RobotPose> robot_poses_;

    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs_;
    std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> goal_pubs_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr formation_timer_;

    void odom_callback(const std::string& robot_id,
                       const nav_msgs::msg::Odometry::ConstSharedPtr& msg);
    void command_callback(const std_msgs::msg::String::ConstSharedPtr& msg);
    void formation_update();

    std::vector<FormationSlot> get_formation_slots(FormationType type) const;
    std::vector<FormationSlot> square_slots()  const;
    std::vector<FormationSlot> diamond_slots() const;
    std::vector<FormationSlot> column_slots()  const;
    std::vector<FormationSlot> line_slots()    const;
    std::vector<FormationSlot> escort_slots()  const;

    geometry_msgs::msg::Pose2D rotate_offset(
        double offset_x, double offset_y, double leader_theta
    ) const noexcept;

    geometry_msgs::msg::PoseStamped pose2d_to_stamped(
        const geometry_msgs::msg::Pose2D& p
    );

    static FormationType formation_from_string(const std::string& name) noexcept;
};

}  // namespace warehouse_navigation
