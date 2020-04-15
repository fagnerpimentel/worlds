#ifndef GAZEBO_PLUGINS_ACTORCONTROL_HH_
#define GAZEBO_PLUGINS_ACTORCONTROL_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/KeyFrame.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Actor.hh>

// #include <actionlib/server/simple_action_server.h>
// #include <social_worlds/ActorAction.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>

#include <geometry_msgs/Pose.h>
#include <social_worlds/ActorControl.h>

#include <thread>
#include <iostream>

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


    // // action
    // private: actionlib::SimpleActionServer<social_worlds::ActorAction> as_;

    public: ActorControl()
    // :as_("/actor_control/a/action", boost::bind(&ActorControl::executeCB, this, _1), false)
    {
      // this->as_.start();
    }

    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      this->actor = boost::reinterpret_pointer_cast<physics::Actor>(_model);
      this->actor->Reset();

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
        ros::SubscribeOptions::create<social_worlds::ActorControl>(
            "/actor_control/" + _model->GetName() + "/action",
            1,
            boost::bind(&ActorControl::OnRosMsg, this, _1),
            ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);

      // Spin up the queue helper thread.
      this->rosQueueThread =
        std::thread(std::bind(&ActorControl::QueueThread, this));
    }

    // public: void executeCB(const social_worlds::ActorGoalConstPtr &goal)
    // {
    //   social_worlds::ActorFeedback feedback_;
    //   social_worlds::ActorResult result_;
    //
    //   this->as_.setSucceeded(result_);
    //
    // }

    /// \brief Handle an incoming message from ROS
    /// \param[in] _msg A float value that is used to set the velocity
    /// of the Velodyne.
    public: void OnRosMsg(const social_worlds::ActorControlConstPtr &_msg)
    {
      std::cout << _msg->animation.data << std::endl;

      // Set custom trajectory
      gazebo::physics::TrajectoryInfoPtr trajectoryInfo(new physics::TrajectoryInfo());
      // trajectoryInfo->SetAnimIndex(2);
      // trajectoryInfo->SetWaypoints();
      trajectoryInfo->type = _msg->animation.data;
      // trajectoryInfo->duration = 50.0;
      this->actor->SetCustomTrajectory(trajectoryInfo);


      //
      double animationFactor = 5.1;
      double distanceTraveled = 0.0;
      auto actorPose = this->actor->WorldPose();

      // Current pose - actor is oriented Y-up and Z-front
      actorPose = this->actor->WorldPose();
      actorPose.Pos().X() = _msg->target.position.x;
      actorPose.Pos().Y() = _msg->target.position.y;
      actorPose.Pos().Z() = _msg->target.position.z;
      actorPose.Rot().X() = _msg->target.orientation.x;
      actorPose.Rot().Y() = _msg->target.orientation.y;
      actorPose.Rot().Z() = _msg->target.orientation.z;
      actorPose.Rot().W() = _msg->target.orientation.w;
      distanceTraveled = (actorPose.Pos() - this->actor->WorldPose().Pos()).Length();
      this->actor->SetWorldPose(actorPose, false, false);
      this->actor->SetScriptTime(this->actor->ScriptTime() +
        (distanceTraveled * animationFactor));


      // gazebo::common::PoseAnimationPtr anim(
      //           new gazebo::common::PoseAnimation("begin", 12.0, true));
      //
      // gazebo::common::PoseKeyFrame *key;
      //
      // key = anim->CreateKeyFrame(0);
      // key->Translation( ignition::math::Vector3<double>(10,10,0) );
      // key->Rotation( ignition::math::Quaternion<double>(0,0,0) );
      //
      // key = anim->CreateKeyFrame(1);
      // key->Translation( ignition::math::Vector3<double>(10,10,0) );
      // key->Rotation( ignition::math::Quaternion<double>(0,0,0) );
      //
      // this->actor->SetAnimation(anim);
      //
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
