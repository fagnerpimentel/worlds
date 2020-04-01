#ifndef GAZEBO_PLUGINS_SLIDINGDOORCONTROL_HH_
#define GAZEBO_PLUGINS_SLIDINGDOORCONTROL_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/physics.hh>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>

#include <thread>
#include <iostream>

namespace gazebo
{
  class GAZEBO_VISIBLE SlidingDoorControl : public ModelPlugin
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
        std::thread(std::bind(&SlidingDoorControl::QueueThread, this));
    }
    ////////////////////////////////////////////////////////////////////

    //
    physics::ModelPtr door;

    public: SlidingDoorControl()
    {
    }

    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      this->InitROS();
      this->door = _model;

      // Create a named topic, and subscribe to it.
      ros::SubscribeOptions so =
        ros::SubscribeOptions::create<std_msgs::String>(
            "/sliding_door_control/" + this->door->GetName() + "/command",
            1,
            boost::bind(&SlidingDoorControl::OnRosMsg, this, _1),
            ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);

      std::cout << "starting sliding_door control: " << this->door->GetName() << std::endl;

    }

    public: void OnRosMsg(const std_msgs::StringConstPtr &_msg)
    {

      std::string jointDoorName = this->door->GetName() + "::door_joint";
      physics::JointPtr jointDoor = this->door->GetJoint(jointDoorName);

      if(_msg->data == "open")
      {
        // jointDoor->SetForce(0, 1);
        jointDoor->SetVelocity(0, 0.7);
        std::cout << "sliding_door control: Open " << this->door->GetName() << std::endl;
      }
      else if(_msg->data == "close")
      {
        // jointDoor->SetForce(0, -1);
        jointDoor->SetVelocity(0, -0.7);
        std::cout << "sliding_door control: Close " << this->door->GetName() << std::endl;
      }
      else
      {
        std::cout << "sliding_door control: Error " << this->door->GetName() << std::endl;
      }
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(SlidingDoorControl)
}
#endif
