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
#include <geometry_msgs/Twist.h>
#include <social_worlds/ActorTrajectory.h>

#include <thread>
#include <iostream>

namespace gazebo
{
  class GAZEBO_VISIBLE ActorControl : public ModelPlugin
  {
    ////////////////////////////////////////////////////////////////////
    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub;
    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;
    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;
    /// \brief ROS helper function that processes messages
    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        update_actor();
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }
    ///  \brief
    private: void InitROS()
    {
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

      // Spin up the queue helper thread.
      this->rosQueueThread =
        std::thread(std::bind(&ActorControl::QueueThread, this));
    }
    ////////////////////////////////////////////////////////////////////


    /// Pointer to the actor.
    private: physics::ActorPtr actor{nullptr};

    private: geometry_msgs::Twist cmd_vel;

    // ros node
    // private: std::unique_ptr<ros::NodeHandle> rosNode;

    // services
    // private: ros::ServiceServer srv_server;

    public: ActorControl()
    {
    }

    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      this->actor = boost::reinterpret_pointer_cast<physics::Actor>(_model);
      this->actor->Reset();

      // start ROS
      InitROS();

      std::string name;
      if (_sdf->HasAttribute ("name"))
      {
        name = _sdf->GetAttribute("name")->GetAsString();
      }

      // Create a named topic, and subscribe to it.
      ros::SubscribeOptions so =
        ros::SubscribeOptions::create<geometry_msgs::Twist>(
            "/" + name + "/cmd_vel", 1,
            boost::bind(&ActorControl::OnRosMsg, this, _1),
            ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);

      // this->cmd_vel.linear.x = 0.3;

      std::cout << "starting actor control: " << name << std::endl;

      // // Service server
      // this->srv_server = this->rosNode->advertiseService(
      //     "/actor_control/" + this->actor->GetName() + "/action",
      //     &ActorControl::OnService_Start, this);
    }

    private: void update_actor()
    {
      // Set custom trajectory
      gazebo::physics::TrajectoryInfoPtr trajectoryInfo(new physics::TrajectoryInfo());
      trajectoryInfo->type = "walking";
      this->actor->SetCustomTrajectory(trajectoryInfo);

      //
      double animationFactor = 5.1;
      double distanceTraveled = 0.0;
      auto actorPose = this->actor->WorldPose();

      double f = sqrt(pow(cmd_vel.linear.x*0.5,2) + pow(cmd_vel.linear.y*0.5,2));
      double theta = (M_PI - actorPose.Rot().Euler().Z()) - atan2(cmd_vel.linear.y*0.5,cmd_vel.linear.x*0.5);
      double delta_px = f*sin(theta);
      double delta_py = f*cos(theta);

      double p_x = actorPose.Pos().X() + delta_px;
      double p_y = actorPose.Pos().Y() + delta_py;
      double p_z = actorPose.Pos().Z() + 0;
      double r_x = actorPose.Rot().Euler().X() + this->cmd_vel.angular.x*0.5;
      double r_y = actorPose.Rot().Euler().Y() + this->cmd_vel.angular.y*0.5;
      double r_z = actorPose.Rot().Euler().Z() + this->cmd_vel.angular.z*0.5;
      actorPose.Set(p_x, p_y, p_z, r_x, r_y, r_z);

      distanceTraveled = (actorPose.Pos() - this->actor->WorldPose().Pos()).Length();
      this->actor->SetWorldPose(actorPose, false, false);
      this->actor->SetScriptTime(this->actor->ScriptTime() +
        (distanceTraveled * animationFactor));

      gazebo::common::Time::MSleep(500);

    }

    // private: bool OnService_Start(
    //     social_worlds::ActorTrajectory::Request &_req,
    //     social_worlds::ActorTrajectory::Response &_res)
    // {
    //   std::cout << "Actor control - "
    //             << "(animation: " << _req.animation.data << ",  "
    //             << "trajectory size: " << _req.waypoints.size() << ")" << std::endl;
    //
    //   // Set custom trajectory
    //   gazebo::physics::TrajectoryInfoPtr trajectoryInfo(new physics::TrajectoryInfo());
    //   trajectoryInfo->type = _req.animation.data;
    //   this->actor->SetCustomTrajectory(trajectoryInfo);
    //
    //   //
    //   double animationFactor = 5.1;
    //   double distanceTraveled = 0.0;
    //   auto actorPose = this->actor->WorldPose();
    //
    //   for (size_t i = 0; i < _req.waypoints.size(); i++) {
    //     // Current pose - actor is oriented Y-up and Z-front
    //     actorPose = this->actor->WorldPose();
    //     actorPose.Pos().X() = _req.waypoints[i].position.x;
    //     actorPose.Pos().Y() = _req.waypoints[i].position.y;
    //     actorPose.Pos().Z() = _req.waypoints[i].position.z;
    //     actorPose.Rot().X() = _req.waypoints[i].orientation.x;
    //     actorPose.Rot().Y() = _req.waypoints[i].orientation.y;
    //     actorPose.Rot().Z() = _req.waypoints[i].orientation.z;
    //     actorPose.Rot().W() = _req.waypoints[i].orientation.w;
    //     distanceTraveled = (actorPose.Pos() - this->actor->WorldPose().Pos()).Length();
    //     this->actor->SetWorldPose(actorPose, false, false);
    //     this->actor->SetScriptTime(this->actor->ScriptTime() +
    //       (distanceTraveled * animationFactor));
    //     gazebo::common::Time::MSleep(500);
    //   }
    //
    //   return true;
    // }

    public: void OnRosMsg(const geometry_msgs::TwistConstPtr &_msg)
    {
      this->cmd_vel = *_msg;
      // std::cout << "cmd_vel updated to "
      //           << "(linear: "
      //           << _msg->linear.x << ", "
      //           << _msg->linear.y << ", "
      //           << _msg->linear.z << "), "
      //           << "(angular: "
      //           << _msg->angular.x << ", "
      //           << _msg->angular.y << ", "
      //           << _msg->angular.z << ")."
      //           << std::endl;
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ActorControl)
}
#endif
