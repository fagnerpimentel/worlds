#ifndef GAZEBO_PLUGINS_PEOPLEPUBLISHER_HH_
#define GAZEBO_PLUGINS_PEOPLEPUBLISHER_HH_

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
#include <people_msgs/People.h>
#include <people_msgs/Person.h>

#include <thread>
#include <iostream>
#include <boost/algorithm/string.hpp>

namespace gazebo
{
  class GAZEBO_VISIBLE PeoplePublisher : public WorldPlugin
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
        this->publish_people();
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
        std::thread(std::bind(&PeoplePublisher::QueueThread, this));
    }
    ////////////////////////////////////////////////////////////////////


    /// Pointers.
    private: physics::WorldPtr world;
    private: std::vector<std::pair<std::string,physics::ActorPtr>> people_pair;

    // last time and positions
    private: std::vector<ignition::math::Vector3d> last_pos;
    private: float last_time = 0;

    // Publishers
    private: ros::Publisher pub_people;

    public: PeoplePublisher()
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
        std::string model_name = el->Get<std::string>();
        physics::ModelPtr model = _world->ModelByName(model_name);
        physics::ActorPtr actor = boost::reinterpret_pointer_cast<physics::Actor>(model);

        std::pair<std::string,physics::ActorPtr> p;
        p.first = model_name;
        p.second = actor;
        this->people_pair.push_back(p);

        ignition::math::Vector3d v;
        v.Set(
          actor->WorldPose().Pos().X(),
          actor->WorldPose().Pos().Y(),
          actor->WorldPose().Pos().Z());
        this->last_pos.push_back(v);
      }

      // people publisher
      this->pub_people =
          this->rosNode->advertise<people_msgs::People>(
          "/"+name, 1000);

      this->last_time = this->world->SimTime().Float();

      std::cout << "starting people publisher." << std::endl;
    }

    private: void publish_people()
    {
      people_msgs::People msg;
      msg.header.seq = 0;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "map";

      for (size_t i = 0; i < this->people_pair.size(); i++)
      {
        double x = this->people_pair.at(i).second->WorldPose().Pos().X();
        double y = this->people_pair.at(i).second->WorldPose().Pos().Y();
        double z = this->people_pair.at(i).second->WorldPose().Pos().Z();
        // double qx = this->people_pair.at(i).second->WorldPose().Rot().X();
        // double qy = this->people_pair.at(i).second->WorldPose().Rot().Y();
        // double qz = this->people_pair.at(i).second->WorldPose().Rot().Z();
        // double qw = this->people_pair.at(i).second->WorldPose().Rot().W();
        double vx = (x - this->last_pos.at(i).X())/
          (this->world->SimTime().Float()-this->last_time);
        double vy = (y - this->last_pos.at(i).Y())/
          (this->world->SimTime().Float()-this->last_time);
        double vz = (z - this->last_pos.at(i).Z())/
          (this->world->SimTime().Float()-this->last_time);
        // double vx = this->people_pair.at(i).second->WorldLinearVel().X();
        // double vy = this->people_pair.at(i).second->WorldLinearVel().Y();
        // double vz = this->people_pair.at(i).second->WorldLinearVel().Z();

        people_msgs::Person person;
        person.name = this->people_pair.at(i).first;
        person.position.x = x;
        person.position.y = y;
        person.position.z = z;
        person.velocity.x = vx;
        person.velocity.y = vy;
        person.velocity.z = vz;
        msg.people.push_back(person);

        this->last_pos.at(i).Set(x,y,z);
      }
      this->last_time = this->world->SimTime().Float();

      this->pub_people.publish(msg);
      gazebo::common::Time::MSleep(500);

    }

  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(PeoplePublisher)
}
#endif
