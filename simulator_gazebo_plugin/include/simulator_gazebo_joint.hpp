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
        Joint(std::string & name,
            double _p,
            double _i,
            double _d):
            name{name},
            pid{_p, _i, _d}
        {
        }

    private:
        std::string & name;
        gazebo::common::PID pid;
};

}

#endif