#include "simulator_gazebo_plugin.hpp"

#include <gazebo/physics/physics.hh>
#include <iostream>
#include <gazebo_ros/node.hpp>

namespace gokart_gazebo_plugin
{

GokartGazeboPlugin::GokartGazeboPlugin()
: robot_namespace_{""},
  last_sim_time_{0},
  last_update_time_{0},
  update_period_ms_{8},
  desired_steering_angle{0},
  desired_velocity{0}
{
}

void GokartGazeboPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Get model and world references
  model_ = model;
  world_ = model_->GetWorld();
  auto physicsEngine = world_->Physics();
  physicsEngine->SetParam("friction_model", std::string{"cone_model"});

  // Get robot namespace if one exists
  if (sdf->HasElement("robotNamespace")) {
    robot_namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
  }

  // Set up ROS node and subscribers and publishers
  ros_node_ = gazebo_ros::Node::Get(sdf);
  RCLCPP_INFO(ros_node_->get_logger(), "Loading Boldbot Gazebo Plugin");

  control_command_sub_ = ros_node_->create_subscription<ControlCommand>(
    "/control_cmd",
    1,
    [ = ](ControlCommand::SharedPtr msg) {
      desired_steering_angle = msg->steering_angle;
      desired_velocity = msg->velocity;
    }
  );

  fl_steering_joint_name = "front_left_steering_joint";
  fr_steering_joint_name = "front_right_steering_joint";
  rl_wheel_joint_name = "back_left_wheel_joint";
  rr_wheel_joint_name = "back_right_wheel_joint";

  auto fl_steering_joint = model_->GetJoint(fl_steering_joint_name);
  auto fr_steering_joint = model_->GetJoint(fr_steering_joint_name);
  auto rl_wheel_joint = model_->GetJoint(rl_wheel_joint_name);
  auto rr_wheel_joint = model_->GetJoint(rr_wheel_joint_name);

  auto fl_steering_joint_pid = gazebo::common::PID{10.0, 0.0, 0.0};
  auto fr_steering_joint_pid = gazebo::common::PID{10.0, 0.0, 0.0};
  auto rl_wheel_joint_pid = gazebo::common::PID{10.0, 0.0, 0.0};
  auto rr_wheel_joint_pid = gazebo::common::PID{10.0, 0.0, 0.0};




  RCLCPP_DEBUG(ros_node_->get_logger(), "Got joints:");
  for (auto const & j : joints_) {
    RCLCPP_DEBUG(ros_node_->get_logger(), j.first);
  }

  // Hook into simulation update loop
  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GokartGazeboPlugin::Update, this));
}

void GokartGazeboPlugin::Update()
{
  auto cur_time = world_->SimTime();
  if (last_sim_time_ == 0) {
    last_sim_time_ = cur_time;
    last_update_time_ = cur_time;
    return;
  }

  auto dt = (cur_time - last_sim_time_).Double();

  

  // Update joint PIDs
  for (auto & j : joints_) {
    auto & joint = j.second.first;
    auto & pid = j.second.second;

    auto error = joint->Position() - joint_targets_[j.first];

    auto force = pid.Update(error, dt);
    joint->SetForce(0, force);
  }

  last_sim_time_ = cur_time;
}

GZ_REGISTER_MODEL_PLUGIN(GokartGazeboPlugin)

}  // namespace boldbot_gazebo_plugin
