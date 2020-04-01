#ifndef GAZEBO_PLUGINS_ACTORCONTROL_HH_
#define GAZEBO_PLUGINS_ACTORCONTROL_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/KeyFrame.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Actor.hh>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>

#include <geometry_msgs/Pose.h>
#include <social_worlds/ActorTrajectory.h>

#include <thread>
#include <iostream>

namespace gazebo
{
  class GAZEBO_VISIBLE ActorControl : public ModelPlugin
  {
    /// Pointer to the actor.
    private: physics::ActorPtr actor{nullptr};

    // ros node
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    // services
    private: ros::ServiceServer srv_server;

    public: ActorControl()
    {
    }

    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      this->actor = boost::reinterpret_pointer_cast<physics::Actor>(_model);
      this->actor->Reset();

      std::cout << "starting actor control: " << this->actor->GetName() << std::endl;

      // start ROS
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",
            ros::init_options::NoSigintHandler);
      }
      this->rosNode.reset(new ros::NodeHandle());

      // Service server
      this->srv_server = this->rosNode->advertiseService(
          "/actor_control/" + this->actor->GetName() + "/action",
          &ActorControl::OnService_Start, this);
    }

    private: bool OnService_Start(
        social_worlds::ActorTrajectory::Request &_req,
        social_worlds::ActorTrajectory::Response &_res)
    {
      std::cout << "Actor control - "
                << "(animation: " << _req.animation.data << ",  "
                << "trajectory size: " << _req.waypoints.size() << ")" << std::endl;

      // Set custom trajectory
      gazebo::physics::TrajectoryInfoPtr trajectoryInfo(new physics::TrajectoryInfo());
      trajectoryInfo->type = _req.animation.data;
      this->actor->SetCustomTrajectory(trajectoryInfo);

      //
      double animationFactor = 5.1;
      double distanceTraveled = 0.0;
      auto actorPose = this->actor->WorldPose();

      for (size_t i = 0; i < _req.waypoints.size(); i++) {
        // Current pose - actor is oriented Y-up and Z-front
        actorPose = this->actor->WorldPose();
        actorPose.Pos().X() = _req.waypoints[i].position.x;
        actorPose.Pos().Y() = _req.waypoints[i].position.y;
        actorPose.Pos().Z() = _req.waypoints[i].position.z;
        actorPose.Rot().X() = _req.waypoints[i].orientation.x;
        actorPose.Rot().Y() = _req.waypoints[i].orientation.y;
        actorPose.Rot().Z() = _req.waypoints[i].orientation.z;
        actorPose.Rot().W() = _req.waypoints[i].orientation.w;
        distanceTraveled = (actorPose.Pos() - this->actor->WorldPose().Pos()).Length();
        this->actor->SetWorldPose(actorPose, false, false);
        this->actor->SetScriptTime(this->actor->ScriptTime() +
          (distanceTraveled * animationFactor));
        gazebo::common::Time::MSleep(500);
      }

      return true;
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ActorControl)
}
#endif
