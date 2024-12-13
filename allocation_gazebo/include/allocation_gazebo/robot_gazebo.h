#ifndef ROBOT_GAZEBO_HPP
#define ROBOT_GAZEBO_HPP

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/math/Vector3.hh>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/callback_queue.hpp>

// Assuming you have a custom message, replace with your actual message type
#include <allocation_common/msgs/robot2gazebo_info.msg>

namespace allocation_gazebo {

class RobotGazebo : public ignition::gazebo::System,
                    public ignition::gazebo::ISystemConfigure,
                    public ignition::gazebo::ISystemPreUpdate
{
private:
    // Ignition pointers
    ignition::gazebo::Model model_;
    ignition::math::Vector3d desired_rot_vector_;
    ignition::math::Vector3d desired_trans_vector_;

    // ROS2 node and communication
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Subscription<allocation_common::msg::Robot2GazeboInfo>::SharedPtr robot2gazebo_sub_;
    
    std::string model_name_;
    std::string robot_namespace_;

    double Vx_cmd_ = 0.0;
    double Vy_cmd_ = 0.0;
    double w_cmd_ = 0.0;

    int robotID_ = -1;

public:
    RobotGazebo();
    ~RobotGazebo() override = default;

    // Ignition System interface methods
    void Configure(const ignition::gazebo::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   ignition::gazebo::EntityComponentManager &_ecm,
                   ignition::gazebo::EventManager &_eventMgr) override;

    void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                   ignition::gazebo::EntityComponentManager &_ecm) override;

    // Callback for robot to Gazebo messages
    void robot2gazebo_callback(const allocation_common::msg::Robot2GazeboInfo::SharedPtr msg);
};

} // namespace allocation_gazebo

#endif // ROBOT_GAZEBO_HPP