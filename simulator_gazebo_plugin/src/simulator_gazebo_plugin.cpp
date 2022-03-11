#include "simulator_gazebo_plugin.hpp"

#include <gazebo/physics/physics.hh>
#include <iostream>
#include <gazebo_ros/node.hpp>
#include <math.h>

namespace gokart_gazebo_plugin
{

GokartGazeboPlugin::GokartGazeboPlugin()
: robot_namespace_{""},
  last_sim_time_{0},
  last_update_time_{0},
  update_period_ms_{8},
  desired_steering_angle{0.0},
  desired_velocity{0.0}
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
  RCLCPP_INFO(ros_node_->get_logger(), "\033[31mLoading Gokart Gazebo Plugin\033[37m");

  control_command_sub_ = ros_node_->create_subscription<ControlCommand>(
    "/control_cmd",
    1,
    [ = ](ControlCommand::SharedPtr msg) {
      //RCLCPP_INFO(ros_node_->get_logger(), "\033[31mReceiving new command message\033[37m");
      desired_steering_angle = msg->steering_angle;
      desired_velocity = msg->velocity;
    }
  );

  RCLCPP_INFO(ros_node_->get_logger(), "\033[31mCreating subscriber\033[37m");

  std::string fl_steering_joint_name = "drivewhl_fl_steer_joint";
  std::string fr_steering_joint_name = "drivewhl_fr_steer_joint";
  std::string rl_motor_joint_name = "drivewhl_l_joint";
  std::string rr_motor_joint_name = "drivewhl_r_joint";

  front_left_steering.SetJoint(fl_steering_joint_name, 2.7, 0.5, 0.3);
  front_left_steering.joint_ = model_->GetJoint(fl_steering_joint_name);

  front_right_steering.SetJoint(fr_steering_joint_name, 2.7, 0.5, 0.3);
  front_right_steering.joint_ = model_->GetJoint(fr_steering_joint_name);

  rear_left_motor.SetJoint(rl_motor_joint_name, 4.8, 2.8, 0.0);
  rear_left_motor.joint_ = model_->GetJoint(rl_motor_joint_name);

  rear_right_motor.SetJoint(rr_motor_joint_name, 4.8, 2.8, 0.0);
  rear_right_motor.joint_ = model_->GetJoint(rr_motor_joint_name);

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
  
  // Compute pid for speed

  double wheel_radius = 0.14; // [m] TODO compute automaticaly from joint definition

  double desired_radial_velocity = desired_velocity / wheel_radius;

  auto err_rear_left = rear_left_motor.joint_->GetVelocity(0) - desired_radial_velocity; // need to chceck if id of rotation axis is 0
  auto err_rear_right = rear_right_motor.joint_->GetVelocity(0) - desired_radial_velocity;

  auto force_rear_left = rear_left_motor.pid.Update(err_rear_left, dt);
  auto force_rear_right = rear_right_motor.pid.Update(err_rear_right, dt);

  rear_left_motor.joint_->SetForce(0, force_rear_left); // need to chceck if id of rotation axis is 0
  rear_right_motor.joint_->SetForce(0, force_rear_right);

  // Car dimensions
  double L = 1.050; // wheelbase [m] TODO compute automaticaly from joint definition
  double T = 1.1; // track width [m] TODO compute automaticaly from joint definition
  
  // Compute ackermann geometry
  double front_left_steering_desired_angle;
  double front_right_steering_desired_angle;

  if (desired_steering_angle < 0.005 && desired_steering_angle > -0.005){
    front_left_steering_desired_angle = desired_steering_angle;
    front_right_steering_desired_angle = desired_steering_angle;
  } else {
    front_left_steering_desired_angle = atan(L/((L/tan(desired_steering_angle)) - T/2.0));
    front_right_steering_desired_angle = atan(L/((L/tan(desired_steering_angle)) + T/2.0));
  }
  // Compute pid for steering
  auto err_front_left_steer = front_left_steering.joint_->Position(0) - front_left_steering_desired_angle; // need to chceck if id of rotation axis is 0
  auto err_front_right_steer = front_right_steering.joint_->Position(0) - front_right_steering_desired_angle;

  auto force_front_left_steer = front_left_steering.pid.Update(err_front_left_steer, dt);
  auto force_front_right_steer = front_right_steering.pid.Update(err_front_right_steer, dt);

  front_left_steering.joint_->SetForce(0, force_front_left_steer); // need to chceck if id of rotation axis is 0
  front_right_steering.joint_->SetForce(0, force_front_right_steer);

  RCLCPP_INFO(ros_node_->get_logger(), "\033[31m" + std::to_string(cur_time.sec) + "  " + std::to_string(rear_left_motor.joint_->GetVelocity(0)) + "  " + std::to_string(rear_right_motor.joint_->GetVelocity(0)) + "\033[37m");

  last_sim_time_ = cur_time;
}

GZ_REGISTER_MODEL_PLUGIN(GokartGazeboPlugin)

}  // namespace boldbot_gazebo_plugin
