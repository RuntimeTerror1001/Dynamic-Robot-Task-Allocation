#ifndef WORLD_GAZEBO_H
#define WORLD_GAZEBO_H

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Helpers.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <gazebo_msgs/msg/model_states.hpp>

// Assuming your custom message definitions
#include "allocation_common/msg/gazebo_robot_info.hpp"
#include "allocation_common/msg/gazebo_task_info.hpp"
#include "allocation_common/msg/terminal2gazebo_info.hpp"
#include "allocation_common/msg/gazebo2world_info.hpp"

namespace allocation_gazebo {

class WorldGazebo : public ignition::gazebo::System,
                    public ignition::gazebo::ISystemConfigure,
                    public ignition::gazebo::ISystemPreUpdate
{
private:
    // ROS2 communication
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_sub_;
    rclcpp::Subscription<allocation_common::msg::Terminal2GazeboInfo>::SharedPtr terminal_info_sub_;
    rclcpp::Publisher<allocation_common::msg::Gazebo2WorldInfo>::SharedPtr gazebo2world_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_pub_;

    // World and model tracking
    std::string world_name_;
    unsigned int model_count_ = 0;

    // Noise generation helper
    double noise_scale_ = 0.0;
    double noise_rate_ = 0.01;

    // Info storage
    allocation_common::msg::GazeboRobotInfo robots_info_;
    allocation_common::msg::GazeboTaskInfo tasks_info_;
    std_msgs::msg::Float64MultiArray debug_msgs_;

    // Tracking flags
    bool model_states_callback_flag_ = false;

    // Internal helper methods
    bool update_model_info();
    double generate_noise(double scale, double probability = 0.01);

public:
    WorldGazebo() = default;
    ~WorldGazebo() override = default;

    // Ignition System interface methods
    void Configure(const ignition::gazebo::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   ignition::gazebo::EntityComponentManager &_ecm,
                   ignition::gazebo::EventManager &_eventMgr) override;

    void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                   ignition::gazebo::EntityComponentManager &_ecm) override;

    // Callback methods
    void model_states_callback(const gazebo_msgs::msg::ModelStates::SharedPtr msg);
    void terminal_info_callback(const allocation_common::msg::Terminal2GazeboInfo::SharedPtr msg);
};

} // namespace allocation_gazebo

#endif // WORLD_GAZEBO_H
