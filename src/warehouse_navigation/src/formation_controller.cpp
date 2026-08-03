#include "warehouse_navigation/formation_controller.hpp"

#include <chrono>
#include <sstream>
#include <stdexcept>

namespace warehouse_navigation {

FormationController::FormationController(const rclcpp::NodeOptions& options)
: Node("formation_controller", options)
{
    declare_parameter("robot_ids",
        std::vector<std::string>{"Robot_1", "Robot_2", "Robot_3", "Robot_4"});
    declare_parameter("leader_robot", std::string{"Robot_1"});
    declare_parameter("default_spacing", DEFAULT_SPACING);
    declare_parameter("formation_type", std::string{"square"});
    declare_parameter("update_rate_hz", 5.0);

    robot_ids_ = get_parameter("robot_ids").as_string_array();
    leader_id_ = get_parameter("leader_robot").as_string();
    spacing_   = get_parameter("default_spacing").as_double();
    current_formation_ = formation_from_string(
        get_parameter("formation_type").as_string());

    for (const auto& rid : robot_ids_) {
        RobotPose rp;
        rp.robot_id = rid;
        rp.valid    = false;
        robot_poses_[rid] = rp;

        std::string ns = rid;
        std::transform(ns.begin(), ns.end(), ns.begin(), ::tolower);
        ns.erase(std::remove(ns.begin(), ns.end(), '_'), ns.end());

        auto odom_sub = create_subscription<nav_msgs::msg::Odometry>(
            "/" + ns + "/odom",
            rclcpp::QoS(10),
            [this, rid](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
                odom_callback(rid, msg);
            }
        );
        odom_subs_.push_back(odom_sub);

        goal_pubs_[rid] = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/warehouse/" + rid + "/formation_goal",
            rclcpp::QoS(10)
        );
    }

    cmd_sub_ = create_subscription<std_msgs::msg::String>(
        "/warehouse/formation/command",
        rclcpp::QoS(10),
        [this](const std_msgs::msg::String::ConstSharedPtr msg) {
            command_callback(msg);
        }
    );

    double update_rate = get_parameter("update_rate_hz").as_double();
    auto period = std::chrono::duration<double>(1.0 / update_rate);
    formation_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this]() { formation_update(); }
    );

    RCLCPP_INFO(get_logger(),
        "FormationController: %zu robots, leader='%s', formation='%s', spacing=%.1f",
        robot_ids_.size(), leader_id_.c_str(),
        get_parameter("formation_type").as_string().c_str(), spacing_);
}

bool FormationController::switch_formation(const std::string& formation_name) {
    auto ft = formation_from_string(formation_name);
    if (ft == current_formation_ && formation_name != "adaptive") {
    }
    current_formation_ = ft;
    RCLCPP_INFO(get_logger(), "Formation switched to: %s", formation_name.c_str());
    return true;
}

void FormationController::set_spacing(double spacing) noexcept {
    spacing_ = std::max(0.5, spacing);
}

void FormationController::set_leader(const std::string& robot_id) {
    std::lock_guard<std::mutex> lk(pose_mutex_);
    if (robot_poses_.count(robot_id)) {
        leader_id_ = robot_id;
        RCLCPP_INFO(get_logger(), "Formation leader set to: %s", robot_id.c_str());
    }
}

std::map<std::string, geometry_msgs::msg::Pose2D>
FormationController::compute_formation_goals() const {
    std::map<std::string, geometry_msgs::msg::Pose2D> goals;

    std::lock_guard<std::mutex> lk(pose_mutex_);
    auto it = robot_poses_.find(leader_id_);
    if (it == robot_poses_.end() || !it->second.valid) {
        return goals;
    }

    const auto& leader_pose = it->second.pose;
    goals[leader_id_] = leader_pose;

    auto slots = get_formation_slots(current_formation_);
    std::size_t slot_idx = 0;

    for (const auto& rid : robot_ids_) {
        if (rid == leader_id_) continue;
        if (slot_idx >= slots.size()) break;

        const auto& slot = slots[slot_idx++];
        auto offset = rotate_offset(
            slot.offset_x * spacing_,
            slot.offset_y * spacing_,
            leader_pose.theta
        );
        geometry_msgs::msg::Pose2D goal;
        goal.x     = leader_pose.x + offset.x;
        goal.y     = leader_pose.y + offset.y;
        goal.theta = leader_pose.theta;
        goals[rid] = goal;
    }

    return goals;
}

void FormationController::odom_callback(
    const std::string& robot_id,
    const nav_msgs::msg::Odometry::ConstSharedPtr& msg)
{
    std::lock_guard<std::mutex> lk(pose_mutex_);
    auto& rp = robot_poses_[robot_id];
    rp.pose.x = msg->pose.pose.position.x;
    rp.pose.y = msg->pose.pose.position.y;
    const auto& q = msg->pose.pose.orientation;
    rp.pose.theta = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    );
    rp.valid       = true;
    rp.last_update = get_clock()->now();
}

void FormationController::command_callback(
    const std_msgs::msg::String::ConstSharedPtr& msg)
{
    const std::string& cmd = msg->data;
    if (cmd.rfind("formation:", 0) == 0) {
        switch_formation(cmd.substr(10));
    } else if (cmd.rfind("leader:", 0) == 0) {
        set_leader(cmd.substr(7));
    } else if (cmd.rfind("spacing:", 0) == 0) {
        try {
            set_spacing(std::stod(cmd.substr(8)));
        } catch (...) {}
    }
}

void FormationController::formation_update() {
    auto goals = compute_formation_goals();
    for (const auto& [rid, pose] : goals) {
        auto it = goal_pubs_.find(rid);
        if (it != goal_pubs_.end()) {
            it->second->publish(pose2d_to_stamped(pose));
        }
    }
}

std::vector<FormationSlot>
FormationController::get_formation_slots(FormationType type) const {
    switch (type) {
        case FormationType::SQUARE:   return square_slots();
        case FormationType::DIAMOND:  return diamond_slots();
        case FormationType::COLUMN:   return column_slots();
        case FormationType::LINE:     return line_slots();
        case FormationType::ESCORT:   return escort_slots();
        case FormationType::ADAPTIVE: return square_slots();
        default:                      return square_slots();
    }
}

std::vector<FormationSlot> FormationController::square_slots() const {
    return {{0, -1.0,  0.0},
            {1,  0.0, -1.0},
            {2, -1.0, -1.0}};
}

std::vector<FormationSlot> FormationController::diamond_slots() const {
    return {{0, -1.0,  0.0},
            {1,  0.0, -1.0},
            {2,  1.0,  0.0}};
}

std::vector<FormationSlot> FormationController::column_slots() const {
    return {{0,  0.0, -1.0},
            {1,  0.0, -2.0},
            {2,  0.0, -3.0}};
}

std::vector<FormationSlot> FormationController::line_slots() const {
    return {{0,  0.0,  1.0},
            {1,  0.0,  2.0},
            {2,  0.0,  3.0}};
}

std::vector<FormationSlot> FormationController::escort_slots() const {
    return {{0,  1.0,  0.0},
            {1, -1.0, -0.5},
            {2, -1.0,  0.5}};
}

geometry_msgs::msg::Pose2D FormationController::rotate_offset(
    double ox, double oy, double theta) const noexcept
{
    geometry_msgs::msg::Pose2D result;
    result.x = ox * std::cos(theta) - oy * std::sin(theta);
    result.y = ox * std::sin(theta) + oy * std::cos(theta);
    result.theta = 0.0;
    return result;
}

geometry_msgs::msg::PoseStamped FormationController::pose2d_to_stamped(
    const geometry_msgs::msg::Pose2D& p)
{
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp    = get_clock()->now();
    ps.header.frame_id = "map";
    ps.pose.position.x = p.x;
    ps.pose.position.y = p.y;
    ps.pose.position.z = 0.0;
    ps.pose.orientation.z = std::sin(p.theta * 0.5);
    ps.pose.orientation.w = std::cos(p.theta * 0.5);
    return ps;
}

FormationType FormationController::formation_from_string(const std::string& name) noexcept {
    std::string lower = name;
    std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
    if (lower == "square")   return FormationType::SQUARE;
    if (lower == "diamond")  return FormationType::DIAMOND;
    if (lower == "column")   return FormationType::COLUMN;
    if (lower == "line")     return FormationType::LINE;
    if (lower == "escort")   return FormationType::ESCORT;
    if (lower == "adaptive") return FormationType::ADAPTIVE;
    return FormationType::SQUARE;
}

}  // namespace warehouse_navigation

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<warehouse_navigation::FormationController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
