#ifndef GAZEBO_PLUGINS_ACTORCONTROL_HH_
#define GAZEBO_PLUGINS_ACTORCONTROL_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Actor.hh>


#include <iostream>
#include <thread>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
// #include <std_msgs/Float32.h>
#include <geometry_msgs/Pose.h>

namespace gazebo
{
  class GAZEBO_VISIBLE ActorControl : public ModelPlugin
  {
    /// \brief Pointer to the actor.
    private: physics::ActorPtr actor{nullptr};

    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub;

    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;

    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;

    public: ActorControl()
    {
    }

    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      this->actor = boost::reinterpret_pointer_cast<physics::Actor>(_model);

      std::cout << "starting actor control: " << this->actor->GetName() << std::endl;

      // Initialize ros, if it has not already bee initialized.
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",
            ros::init_options::NoSigintHandler);
      }

      // Create our ROS node. This acts in a similar manner to
      // the Gazebo node
      this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

      // Create a named topic, and subscribe to it.
      ros::SubscribeOptions so =
        ros::SubscribeOptions::create<geometry_msgs::Pose>(
            "/actor_control/" + _model->GetName() + "/pose",
            1,
            boost::bind(&ActorControl::OnRosMsg, this, _1),
            ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);

      // Spin up the queue helper thread.
      this->rosQueueThread =
        std::thread(std::bind(&ActorControl::QueueThread, this));
    }

    /// \brief Handle an incoming message from ROS
    /// \param[in] _msg A float value that is used to set the velocity
    /// of the Velodyne.
    public: void OnRosMsg(const geometry_msgs::PoseConstPtr &_msg)
    {
      std::cout << this->actor->ScriptTime() << std::endl;

      // Set custom trajectory
      gazebo::physics::TrajectoryInfoPtr trajectoryInfo(new physics::TrajectoryInfo());
      trajectoryInfo->type = "walking";
      trajectoryInfo->duration = 5.0;
      this->actor->SetCustomTrajectory(trajectoryInfo);

      //
      double animationFactor = 5.1;
      double distanceTraveled = 0.0;
      auto actorPose = this->actor->WorldPose();

      // Current pose - actor is oriented Y-up and Z-front
      actorPose = this->actor->WorldPose();
      actorPose.Pos().X() = _msg->position.x;
      actorPose.Pos().Y() = _msg->position.y;
      actorPose.Pos().Z() = _msg->position.z;
      distanceTraveled = (actorPose.Pos() - this->actor->WorldPose().Pos()).Length();
      this->actor->SetWorldPose(actorPose, false, false);
      this->actor->SetScriptTime(this->actor->ScriptTime() +
        (distanceTraveled * animationFactor));

    }

    /// \brief ROS helper function that processes messages
    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ActorControl)
}
#endif
