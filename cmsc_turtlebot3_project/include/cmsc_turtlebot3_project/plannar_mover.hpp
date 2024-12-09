#ifndef PLANNAR_MOVER_HPP
#define PLANNAR_MOVER_HPP

#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include <geometry_msgs/msg/twist.hpp>

namespace gazebo
{
  class PlannarMover : public ModelPlugin
  {
  public:
    // Constructor
    PlannarMover();

    // Destructor
    virtual ~PlannarMover();

    // Load function called by Gazebo when the plugin is loaded
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) override;

  private:
    // Callback for receiving velocity commands
    void OnRosMsg_Pos(const geometry_msgs::msg::Twist::SharedPtr _msg);

    // Update function called every simulation iteration
    void OnUpdate();

    // Function to move the model based on velocity commands
    void MoveModelsPlane(float linear_x_vel, float linear_y_vel, float linear_z_vel,
                         float angular_x_vel, float angular_y_vel, float angular_z_vel);

    // Pointer to the model
    physics::ModelPtr model;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

    // ROS node handle
    rclcpp::Node::SharedPtr node_;

    // ROS subscription for velocity commands
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  };
}

#endif // PLANNAR_MOVER_HPP