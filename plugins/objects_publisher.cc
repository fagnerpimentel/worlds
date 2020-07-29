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
#include <object_recognition_msgs/RecognizedObjectArray.h>
#include <object_recognition_msgs/RecognizedObject.h>

#include <thread>
#include <iostream>
#include <boost/algorithm/string.hpp>

namespace gazebo
{
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
    private: std::vector<std::pair<std::string,physics::ModelPtr>> objects_pair;

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
        std::string model_type = el->GetAttribute("type")->GetAsString();
        std::string model_name = el->Get<std::string>();
        std::pair<std::string,physics::ModelPtr> p;
        p.first = model_type;
        p.second = _world->ModelByName(model_name);
        this->objects_pair.push_back(p);
      }

      // people publisher
      this->pub_objects =
          this->rosNode->advertise<object_recognition_msgs::RecognizedObjectArray>(
          "/"+name, 1000);

      std::cout << "starting objects publisher." << std::endl;
    }

    private: void publish_objects()
    {
      object_recognition_msgs::RecognizedObjectArray msg;
      msg.header.seq = 0;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "map";

      for (size_t i = 0; i < this->objects_pair.size(); i++)
      {

        double x = this->objects_pair.at(i).second->WorldPose().Pos().X();
        double y = this->objects_pair.at(i).second->WorldPose().Pos().Y();
        double z = this->objects_pair.at(i).second->WorldPose().Pos().Z();
        double qx = this->objects_pair.at(i).second->WorldPose().Rot().X();
        double qy = this->objects_pair.at(i).second->WorldPose().Rot().Y();
        double qz = this->objects_pair.at(i).second->WorldPose().Rot().Z();
        double qw = this->objects_pair.at(i).second->WorldPose().Rot().W();

        object_recognition_msgs::RecognizedObject object;
        object.header.seq = i;
        object.header.stamp = ros::Time::now();
        object.header.frame_id = "map";
        object.type.key = this->objects_pair.at(i).first;
        object.pose.header.seq = i;
        object.pose.header.stamp = ros::Time::now();
        object.pose.header.frame_id = "map";
        object.pose.pose.pose.position.x = x;
        object.pose.pose.pose.position.y = y;
        object.pose.pose.pose.position.z = z;
        object.pose.pose.pose.orientation.x = qx;
        object.pose.pose.pose.orientation.x = qy;
        object.pose.pose.pose.orientation.x = qz;
        object.pose.pose.pose.orientation.x = qw;
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
