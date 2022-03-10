#ifndef SIMULATOR_GAZEBO_PLUGIN__SIMULATOR_GAZEBO_JOINT_HPP_
#define SIMULATOR_GAZEBO_PLUGIN__SIMULATOR_GAZEBO_JOINT_HPP_

#include <gazebo/common/PID.hh>
#include <gazebo/common/Plugin.hh>
#include <rclcpp/rclcpp.hpp>

namespace gokart_gazebo_plugin
{

class Joint
{
    public:
        Joint():
        name{""},
        pid{0.0, 0.0, 0.0}
        {

        };

        void SetJoint(std::string name,
                double _p,
                double _i,
                double _d)
        {
            name = name;
            pid.SetPGain(_p);
            pid.SetIGain(_i);
            pid.SetDGain(_d);
        }

        gazebo::physics::JointPtr joint_;
        std::string name;
        gazebo::common::PID pid;
};

}

#endif