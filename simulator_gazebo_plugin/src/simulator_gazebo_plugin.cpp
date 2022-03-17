#include "simulator_gazebo_plugin.hpp"

#include <math.h>

#include <iostream>

#include "terminal.h"
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

namespace gokart_gazebo_plugin
{

GokartGazeboPlugin::GokartGazeboPlugin()
: last_sim_time_{0}, last_update_time_{0}, desired_steering_angle_{0.0}, desired_velocity_{0.0}
{
}

void GokartGazeboPlugin::LoadParameters(sdf::ElementPtr sdf)
{
  base_link_name_ = sdf->GetElement("baseLinkName")->Get<std::string>();
  fl_steering_joint_name_ = sdf->GetElement("frontLeftSteeringJointName")->Get<std::string>();
  fr_steering_joint_name_ = sdf->GetElement("frontRightSteeringJointName")->Get<std::string>();
  rl_motor_joint_name_ = sdf->GetElement("rearLeftMotorJointName")->Get<std::string>();
  rr_motor_joint_name_ = sdf->GetElement("rearRightMotorJointName")->Get<std::string>();
}

void GokartGazeboPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // save model reference
  model_ = model;

  // TODO: consider just checking that correct friction_model is set
  //       and fail with a log message when it is not
  world_ = model_->GetWorld();
  auto physicsEngine = world_->Physics();
  physicsEngine->SetParam("friction_model", std::string{"cone_model"});

  LoadParameters(sdf);

  // save model's base_link reference
  fprintf(stderr, "base_link_name_ = %s\n", base_link_name_.c_str());
  base_link_ = model_->GetLink(base_link_name_);

  // set up ROS node, subscribers and publishers

  ros_node_ = gazebo_ros::Node::Get(sdf);

  RCLCPP_INFO(ros_node_->get_logger(), red("Setting up ROS node..."));

  ground_truth_pub_ = ros_node_->create_publisher<Odometry>("/ground_truth", 1);

  control_command_sub_ = ros_node_->create_subscription<ControlCommand>(
    "/control_cmd", 1, [=](ControlCommand::SharedPtr msg) {
      // RCLCPP_INFO(ros_node_->get_logger(), red("Receiving new command message"));

      // 0.6 rad ~ 34 deg
      if (msg->steering_angle < -0.6) {
        desired_steering_angle_ = -0.6;
      } else if (msg->steering_angle > 0.6) {
        desired_steering_angle_ = 0.6;
      } else {
        desired_steering_angle_ = msg->steering_angle;
      }

      // 20 mps = 72 kmph = 44.74 mph
      if (msg->velocity < -20) {
        desired_velocity_ = -20;
      } else if (msg->velocity > 20) {
        desired_velocity_ = 20;
      } else {
        desired_velocity_ = msg->velocity;
      }
    });

  front_left_steering.SetJoint(fl_steering_joint_name_, 2.7, 0.5, 0.3);
  front_left_steering.joint_ = model_->GetJoint(fl_steering_joint_name_);

  front_right_steering.SetJoint(fr_steering_joint_name_, 2.7, 0.5, 0.3);
  front_right_steering.joint_ = model_->GetJoint(fr_steering_joint_name_);

  rear_left_motor.SetJoint(rl_motor_joint_name_, 4.8, 2.8, 0.0);
  rear_left_motor.joint_ = model_->GetJoint(rl_motor_joint_name_);

  rear_right_motor.SetJoint(rr_motor_joint_name_, 4.8, 2.8, 0.0);
  rear_right_motor.joint_ = model_->GetJoint(rr_motor_joint_name_);

  // Hook into simulation update loop
  update_connection_ =
    gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&GokartGazeboPlugin::Update, this));
}

void GokartGazeboPlugin::Update()
{
  auto cur_time = world_->SimTime();

  if (last_sim_time_ == 0) {
    last_sim_time_ = cur_time;
    last_update_time_ = cur_time;
    return;
  }

  // ground_truth_pub
  ignition::math::Pose3d pose = base_link_->WorldPose();  // WorldLinearVel
  ground_truth_msg_.pose.pose.position.x = pose.Pos().X();
  ground_truth_msg_.pose.pose.position.y = pose.Pos().Y();
  ground_truth_msg_.pose.pose.position.z = pose.Pos().Z();
  ground_truth_msg_.pose.pose.orientation.x = pose.Rot().X();
  ground_truth_msg_.pose.pose.orientation.y = pose.Rot().Y();
  ground_truth_msg_.pose.pose.orientation.z = pose.Rot().Z();
  ground_truth_msg_.pose.pose.orientation.w = pose.Rot().W();
  ignition::math::Vector3d lin_velocity = base_link_->WorldLinearVel();
  ground_truth_msg_.twist.twist.linear.x = lin_velocity.X();
  ground_truth_msg_.twist.twist.linear.y = lin_velocity.Y();
  ground_truth_msg_.twist.twist.linear.z = lin_velocity.Z();
  ignition::math::Vector3d angular_velocity = base_link_->WorldAngularVel();
  ground_truth_msg_.twist.twist.angular.x = angular_velocity.X();
  ground_truth_msg_.twist.twist.angular.y = angular_velocity.Y();
  ground_truth_msg_.twist.twist.angular.z = angular_velocity.Z();
  ground_truth_pub_->publish(ground_truth_msg_);

  auto dt = (cur_time - last_sim_time_).Double();

  // Compute PID for speed

  double wheel_radius = 0.14;  // [m] TODO compute automaticaly from joint definition

  double desired_radial_velocity = desired_velocity_ / wheel_radius;

  // need to check if id of rotation axis is 0
  auto err_rear_left = rear_left_motor.joint_->GetVelocity(0) - desired_radial_velocity;
  auto err_rear_right = rear_right_motor.joint_->GetVelocity(0) - desired_radial_velocity;

  auto force_rear_left = rear_left_motor.pid.Update(err_rear_left, dt);
  auto force_rear_right = rear_right_motor.pid.Update(err_rear_right, dt);

  // need to chceck if id of rotation axis is 0
  rear_left_motor.joint_->SetForce(0, force_rear_left);
  rear_right_motor.joint_->SetForce(0, force_rear_right);

  // car dimensions
  double L = 1.050;  // wheelbase [m] TODO compute automaticaly from joint definition
  double T = 1.1;    // track width [m] TODO compute automaticaly from joint definition

  // compute ackermann geometry
  double front_left_steering_desired_angle;
  double front_right_steering_desired_angle;

  if (desired_steering_angle_ < 0.005 && desired_steering_angle_ > -0.005) {
    front_left_steering_desired_angle = desired_steering_angle_;
    front_right_steering_desired_angle = desired_steering_angle_;
  } else {
    front_left_steering_desired_angle = atan(L / ((L / tan(desired_steering_angle_)) - T / 2.0));
    front_right_steering_desired_angle = atan(L / ((L / tan(desired_steering_angle_)) + T / 2.0));
  }

  // compute PID for steering
  // need to chceck if id of rotation axis is 0
  auto err_front_left_steer =
    front_left_steering.joint_->Position(0) - front_left_steering_desired_angle;
  auto err_front_right_steer =
    front_right_steering.joint_->Position(0) - front_right_steering_desired_angle;

  auto force_front_left_steer = front_left_steering.pid.Update(err_front_left_steer, dt);
  auto force_front_right_steer = front_right_steering.pid.Update(err_front_right_steer, dt);

  // need to chceck if id of rotation axis is 0
  front_left_steering.joint_->SetForce(0, force_front_left_steer);
  front_right_steering.joint_->SetForce(0, force_front_right_steer);

  // RCLCPP_INFO(
  //   ros_node_->get_logger(),
  //   CSI_RED + std::to_string(cur_time.sec) + "  " +
  //     std::to_string(rear_left_motor.joint_->GetVelocity(0)) + "  " +
  //     std::to_string(rear_right_motor.joint_->GetVelocity(0)) + rst);

  last_sim_time_ = cur_time;
}

GZ_REGISTER_MODEL_PLUGIN(GokartGazeboPlugin)

}  // namespace gokart_gazebo_plugin
