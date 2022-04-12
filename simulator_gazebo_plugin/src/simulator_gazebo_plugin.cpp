#include "simulator_gazebo_plugin.hpp"

#include <math.h>
#include <tf2_ros/transform_broadcaster.h>

#include <iostream>

#include "terminal.h"
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

namespace gokart_gazebo_plugin
{

GokartGazeboPlugin::GokartGazeboPlugin()
: last_sim_time_{0}, last_update_time_{0}, desired_steering_angle_{0.0}, desired_velocity_{0.0}
{
}

void GokartGazeboPlugin::LoadParameters(sdf::ElementPtr sdf)
{
  map_frame_name_ = sdf->GetElement("mapFrameName")->Get<std::string>();
  base_link_name_ = sdf->GetElement("baseLinkName")->Get<std::string>();
  fl_steering_joint_name_ = sdf->GetElement("frontLeftSteeringJointName")->Get<std::string>();
  fr_steering_joint_name_ = sdf->GetElement("frontRightSteeringJointName")->Get<std::string>();
  fl_motor_joint_name_ = sdf->GetElement("frontLeftMotorJointName")->Get<std::string>();
  fr_motor_joint_name_ = sdf->GetElement("frontRightMotorJointName")->Get<std::string>();
  rl_motor_joint_name_ = sdf->GetElement("rearLeftMotorJointName")->Get<std::string>();
  rr_motor_joint_name_ = sdf->GetElement("rearRightMotorJointName")->Get<std::string>();
  publish_ground_truth_transform_ = sdf->GetElement("publishGroundTruthTransform")->Get<bool>();
  publish_joint_states_ = sdf->GetElement("publishJointStates")->Get<bool>();
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

  joint_state_pub_ = ros_node_->create_publisher<JointState>("/joint_states", 1);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*ros_node_);

  control_command_sub_ = ros_node_->create_subscription<ControlCommand>(
    "/control_cmd", 1, [=](ControlCommand::SharedPtr msg) {
      // RCLCPP_INFO(ros_node_->get_logger(), red("Receiving new command message"));

      // 0.6 rad ~ 34 deg
      if (msg->steering_angle < -max_steering_angle_) {
        desired_steering_angle_ = -max_steering_angle_;
      } else if (msg->steering_angle > max_steering_angle_) {
        desired_steering_angle_ = max_steering_angle_;
      } else {
        desired_steering_angle_ = msg->steering_angle;
      }

      // 20 mps = 72 kmph = 44.74 mph
      if (msg->velocity < max_velocity_backward_) {
        desired_velocity_ = max_velocity_backward_;
      } else if (msg->velocity > max_velocity_forward_) {
        desired_velocity_ = max_velocity_forward_;
      } else {
        desired_velocity_ = msg->velocity;
      }
    });

  autoware_control_command_sub_ = ros_node_->create_subscription<AutowareControlCommand>(
    "/autoware_control_cmd", 1, [=](AutowareControlCommand::SharedPtr msg) {
      // RCLCPP_INFO(ros_node_->get_logger(), red("Receiving new command message"));

      // 0.6 rad ~ 34 deg
      if (msg->front_wheel_angle_rad < -max_steering_angle_) {
        desired_steering_angle_ = -max_steering_angle_;
      } else if (msg->front_wheel_angle_rad > max_steering_angle_) {
        desired_steering_angle_ = max_steering_angle_;
      } else {
        desired_steering_angle_ = msg->front_wheel_angle_rad;
      }

      // 20 mps = 72 kmph = 44.74 mph
      if (msg->velocity_mps < max_velocity_backward_) {
        desired_velocity_ = max_velocity_backward_;
      } else if (msg->velocity_mps > max_velocity_forward_) {
        desired_velocity_ = max_velocity_forward_;
      } else {
        desired_velocity_ = msg->velocity_mps;
      }
    });

  front_left_steering.SetJoint(fl_steering_joint_name_, 2.7, 0.5, 0.3);
  front_left_steering.joint_ = model_->GetJoint(fl_steering_joint_name_);

  front_right_steering.SetJoint(fr_steering_joint_name_, 2.7, 0.5, 0.3);
  front_right_steering.joint_ = model_->GetJoint(fr_steering_joint_name_);

  front_left_motor.SetJoint(fl_motor_joint_name_, 0.0, 0.0, 0.0);
  front_left_motor.joint_ = model_->GetJoint(fl_motor_joint_name_);

  front_right_motor.SetJoint(rr_motor_joint_name_, 0.0, 0.0, 0.0);
  front_right_motor.joint_ = model_->GetJoint(fr_motor_joint_name_);

  rear_left_motor.SetJoint(rl_motor_joint_name_, 4.8, 2.8, 0.0);
  rear_left_motor.joint_ = model_->GetJoint(rl_motor_joint_name_);

  rear_right_motor.SetJoint(rr_motor_joint_name_, 4.8, 2.8, 0.0);
  rear_right_motor.joint_ = model_->GetJoint(rr_motor_joint_name_);

  // RCLCPP_INFO(ros_node_->get_logger(), red("Wheel radius: ") +
  // rear_right_motor.joint_->GetChild()->GetSDF());

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
  ground_truth_msg_.header.stamp.sec = cur_time.sec;
  ground_truth_msg_.header.stamp.nanosec = cur_time.nsec;
  ground_truth_msg_.header.frame_id = map_frame_name_;
  ground_truth_msg_.child_frame_id = base_link_name_;
  ignition::math::Pose3d pose = base_link_->WorldPose();  // WorldLinearVel
  ground_truth_msg_.pose.pose.position.x = pose.Pos().X();
  ground_truth_msg_.pose.pose.position.y = pose.Pos().Y();
  ground_truth_msg_.pose.pose.position.z = pose.Pos().Z();
  ground_truth_msg_.pose.pose.orientation.x = pose.Rot().X();
  ground_truth_msg_.pose.pose.orientation.y = pose.Rot().Y();
  ground_truth_msg_.pose.pose.orientation.z = pose.Rot().Z();
  ground_truth_msg_.pose.pose.orientation.w = pose.Rot().W();
  ignition::math::Vector3d lin_velocity = base_link_->RelativeLinearVel();
  ground_truth_msg_.twist.twist.linear.x = lin_velocity.X();
  ground_truth_msg_.twist.twist.linear.y = lin_velocity.Y();
  ground_truth_msg_.twist.twist.linear.z = lin_velocity.Z();
  ignition::math::Vector3d angular_velocity = base_link_->RelativeAngularVel();
  ground_truth_msg_.twist.twist.angular.x = angular_velocity.X();
  ground_truth_msg_.twist.twist.angular.y = angular_velocity.Y();
  ground_truth_msg_.twist.twist.angular.z = angular_velocity.Z();
  ground_truth_pub_->publish(ground_truth_msg_);

  if (publish_ground_truth_transform_) {
    // tf publisher

    ground_truth_tf_pub_.header.stamp.sec = cur_time.sec;
    ground_truth_tf_pub_.header.stamp.nanosec = cur_time.nsec;
    ground_truth_tf_pub_.header.frame_id = map_frame_name_;
    ground_truth_tf_pub_.child_frame_id = base_link_name_;

    ground_truth_tf_pub_.transform.translation.x = pose.Pos().X();
    ground_truth_tf_pub_.transform.translation.y = pose.Pos().Y();
    ground_truth_tf_pub_.transform.translation.z = pose.Pos().Z();

    ground_truth_tf_pub_.transform.rotation.x = pose.Rot().X();
    ground_truth_tf_pub_.transform.rotation.y = pose.Rot().Y();
    ground_truth_tf_pub_.transform.rotation.z = pose.Rot().Z();
    ground_truth_tf_pub_.transform.rotation.w = pose.Rot().W();

    // Send the transformation
    tf_broadcaster_->sendTransform(ground_truth_tf_pub_);
  }

  if (publish_joint_states_) {
    // joint state publisher
    joint_state_msg_.header.stamp.sec = cur_time.sec;
    joint_state_msg_.header.stamp.nanosec = cur_time.nsec;
    joint_state_msg_.header.frame_id = base_link_name_;
    joint_state_msg_.name = {
      rl_motor_joint_name_,
      rr_motor_joint_name_,
      fl_steering_joint_name_,
      fr_steering_joint_name_,
      fl_motor_joint_name_,
      fr_motor_joint_name_};
    joint_state_msg_.position = {
      rear_left_motor.joint_->Position(0),
      rear_right_motor.joint_->Position(0),
      front_left_steering.joint_->Position(0),
      front_right_steering.joint_->Position(0),
      front_left_motor.joint_->Position(0),
      front_right_motor.joint_->Position(0)};
    joint_state_msg_.velocity = {
      rear_left_motor.joint_->GetVelocity(0),
      rear_right_motor.joint_->GetVelocity(0),
      front_left_steering.joint_->GetVelocity(0),
      front_right_steering.joint_->GetVelocity(0),
      front_left_motor.joint_->GetVelocity(0),
      front_right_motor.joint_->GetVelocity(0)};
    joint_state_msg_.effort = {
      rear_left_motor.joint_->GetForce(0),
      rear_right_motor.joint_->GetForce(0),
      front_left_steering.joint_->GetForce(0),
      front_right_steering.joint_->GetForce(0),
      front_left_motor.joint_->GetForce(0),
      front_right_motor.joint_->GetForce(0)};

    joint_state_pub_->publish(joint_state_msg_);
  }

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
