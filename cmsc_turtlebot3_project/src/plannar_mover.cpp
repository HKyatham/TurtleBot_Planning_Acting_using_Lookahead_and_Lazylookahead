#include "cmsc_turtlebot3_project/plannar_mover.hpp"

namespace gazebo
{
  // Constructor
  PlannarMover::PlannarMover() {}

  // Destructor
  PlannarMover::~PlannarMover() {}

  void PlannarMover::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    this->model = _parent;

    this->node_ = rclcpp::Node::make_shared("plannar_rosnode");

    this->subscription_ = this->node_->create_subscription<geometry_msgs::msg::Twist>(
      "/block_vel", 10,
      std::bind(&PlannarMover::OnRosMsg_Pos, this, std::placeholders::_1));

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&PlannarMover::OnUpdate, this));

    RCLCPP_WARN(this->node_->get_logger(), "Loaded PlannarMover Plugin with parent...%s, only X Axis Freq Supported in this V-1.0", 
                this->model->GetName().c_str());
  }

  void PlannarMover::OnUpdate()
  {
    rclcpp::spin_some(this->node_);
  }

  void PlannarMover::MoveModelsPlane(float linear_x_vel, float linear_y_vel, float linear_z_vel,
                                      float angular_x_vel, float angular_y_vel, float angular_z_vel)
  {
      this->model->SetLinearVel(ignition::math::Vector3d(linear_x_vel, linear_y_vel, linear_z_vel));
      this->model->SetAngularVel(ignition::math::Vector3d(angular_x_vel, angular_y_vel, angular_z_vel));
  }

  void PlannarMover::OnRosMsg_Pos(const geometry_msgs::msg::Twist::SharedPtr _msg)
  {
      this->MoveModelsPlane(_msg->linear.x, _msg->linear.y, _msg->linear.z,
                            _msg->angular.x, _msg->angular.y, _msg->angular.z);
  }

  GZ_REGISTER_MODEL_PLUGIN(PlannarMover)
}