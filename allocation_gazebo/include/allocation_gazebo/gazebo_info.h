#ifndef GAZEBO_INFO_H
#define GAZEBO_INFO_H

#include <ignition/gazebo.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
#include <allocation_common/DPoint.hpp>
#include <allocation_common/Core.hpp>

constexpr double PI = 3.14159265;
constexpr double CM2M_CONVERSION = 0.01;
constexpr double M2CM_CONVERSION = 100;
constexpr double G = 9.8;

const std::string robot_name = "Robot";
const std::string task_name = "Task";

struct Pose
{
    ignition::math::Vector3d position;
    ignition::math::Quaterniond orientation;
};

struct Twist
{
    ignition::math::Vector3d linear;
    ignition::math::Vector3d angular;
};

struct Robot_state
{
    std::string robot_name;
    int robot_ID;
    Pose pose;
    Twist twist;
    Robots_mode current_mode;
    std::string reference_frame;
};

struct Task_state
{
    DPoint world_pos;
    bool is_target;
    bool is_valid;
};

#endif // GAZEBO_INFO_H
