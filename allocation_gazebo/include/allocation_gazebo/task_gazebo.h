#ifndef TASK_GAZEBO_HPP
#define TASK_GAZEBO_HPP

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/World.hh>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/callback_queue.hpp>

#include <random>  // Replace Gazebo's math::Rand

namespace allocation_gazebo {

class TaskGazebo : public ignition::gazebo::System,
                   public ignition::gazebo::ISystemConfigure,
                   public ignition::gazebo::ISystemPreUpdate
{
private:
    // Ignition pointers and systems
    ignition::gazebo::Model task_model_;
    
    // ROS2 communication
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Subscription<allocation_common::msg::AllocationTaskInfo>::SharedPtr robot2task_sub_;

    // Random number generation
    std::random_device random_device_;
    std::mt19937 random_generator_;

    // Task-specific attributes
    std::string model_name_;
    std::string robot_namespace_;
    int taskID_ = -1;
    std::vector<int> destroy_tasks_;

public:
    TaskGazebo();
    ~TaskGazebo() override = default;

    // Ignition System interface methods
    void Configure(const ignition::gazebo::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   ignition::gazebo::EntityComponentManager &_ecm,
                   ignition::gazebo::EventManager &_eventMgr) override;

    void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                   ignition::gazebo::EntityComponentManager &_ecm) override;

    // Message callback (uncomment and implement as needed)
    // void task_state_callback(const allocation_common::msg::AllocationTaskInfo::SharedPtr msg);
};

} // namespace allocation_gazebo

#endif // TASK_GAZEBO_HPP