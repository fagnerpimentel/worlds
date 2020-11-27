#ifndef GAZEBO_PLUGINS_OBJECTSPUBLISHER_HH_
#define GAZEBO_PLUGINS_OBJECTSPUBLISHER_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/KeyFrame.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Actor.hh>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <social_msgs/Objects.h>
#include <social_msgs/Object.h>

#include <thread>
#include <iostream>
#include <boost/algorithm/string.hpp>

namespace gazebo
{

  struct object_info {
    std::string name;
    physics::ModelPtr model;
    std::string type;
    // std::string mesh;
  } ;

  class GAZEBO_VISIBLE ObjectsPublisher : public WorldPlugin
  {
    ////////////////////////////////////////////////////////////////////
    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    // /// \brief A ROS subscriber
    // private: ros::Subscriber rosSub;
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
        this->publish_objects();
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
        std::thread(std::bind(&ObjectsPublisher::QueueThread, this));
    }
    ////////////////////////////////////////////////////////////////////


    /// Pointers.
    private: physics::WorldPtr world;
    private: std::vector<object_info> objects_info;

    // Publishers
    private: ros::Publisher pub_objects;

    public: ObjectsPublisher()
    {
    }

    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
      this->world = _world;
      this->InitROS();

      std::string name;

      if (_sdf->HasAttribute ("name"))
      {
        name = _sdf->GetAttribute("name")->GetAsString();
      }

      while(_sdf->HasElement("model"))
      {
        sdf::ElementPtr el = _sdf->GetElement("model");
        el->RemoveFromParent();

        object_info oi;
        oi.name = el->Get<std::string>();
        oi.model = _world->ModelByName(oi.name);
        oi.type = el->GetAttribute("type")->GetAsString();
        // oi.mesh = el->GetAttribute("mesh")->GetAsString();
        this->objects_info.push_back(oi);

      }

      // people publisher
      this->pub_objects =
          this->rosNode->advertise<social_msgs::Objects>(
          "/"+name, 1000);

      std::cout << "starting objects publisher." << std::endl;
    }

    private: void publish_objects()
    {
      social_msgs::Objects msg;
      msg.header.seq = 0;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "map";

      for (size_t i = 0; i < this->objects_info.size(); i++)
      {

        double x = this->objects_info.at(i).model->WorldPose().Pos().X();
        double y = this->objects_info.at(i).model->WorldPose().Pos().Y();
        double z = this->objects_info.at(i).model->WorldPose().Pos().Z();
        double qx = this->objects_info.at(i).model->WorldPose().Rot().X();
        double qy = this->objects_info.at(i).model->WorldPose().Rot().Y();
        double qz = this->objects_info.at(i).model->WorldPose().Rot().Z();
        double qw = this->objects_info.at(i).model->WorldPose().Rot().W();

        social_msgs::Object object;
        object.name = this->objects_info.at(i).name;
        object.type = this->objects_info.at(i).type;
        // object.mesh = this->objects_info.at(i).mesh;
        object.pose.position.x = x;
        object.pose.position.y = y;
        object.pose.position.z = z;
        object.pose.orientation.x = qx;
        object.pose.orientation.y = qy;
        object.pose.orientation.z = qz;
        object.pose.orientation.w = qw;

        msg.objects.push_back(object);
      }

      this->pub_objects.publish(msg);
      gazebo::common::Time::MSleep(500);

    }

  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(ObjectsPublisher)
}
#endif
