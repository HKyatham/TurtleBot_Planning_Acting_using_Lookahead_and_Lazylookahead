#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
  class WaypointPlugin : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
    {
      this->model = _model;
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&WaypointPlugin::OnUpdate, this));

      // Define waypoints and times
      this->waypoints = {
        ignition::math::Pose3d(4.60714, 0.72155, 0.2, 0, 0, 0),
        ignition::math::Pose3d(4.60714, -0.72155, 0.2, 0, 0, 0),
        ignition::math::Pose3d(4.60714, 0.72155, 0.2, 0, 0, 0)
      };
      this->times = {0, 8, 16};
    }

    public: void OnUpdate()
    {
      double currentTime = this->model->GetWorld()->SimTime().Double();
      
      size_t nextWaypoint = (currentTime / this->times.back()) * waypoints.size();
      nextWaypoint = nextWaypoint % waypoints.size();

      ignition::math::Pose3d currentPose = this->waypoints[nextWaypoint];
      
      this->model->SetWorldPose(currentPose);
    }

    private: physics::ModelPtr model;
    private: event::ConnectionPtr updateConnection;
    private: std::vector<ignition::math::Pose3d> waypoints;
    private: std::vector<double> times;
  };

  GZ_REGISTER_MODEL_PLUGIN(WaypointPlugin)
}