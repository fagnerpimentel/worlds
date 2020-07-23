#ifndef GAZEBO_PLUGINS_LINKACTORTOMODEL_HH_
#define GAZEBO_PLUGINS_LINKACTORTOMODEL_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/KeyFrame.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Actor.hh>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>

#include <std_msgs/Header.h>
#include <people_msgs/Person.h>
#include <people_msgs/People.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <social_worlds/ActorTrajectory.h>

#include <thread>
#include <iostream>
#include <boost/algorithm/string.hpp>

namespace gazebo
{
  class GAZEBO_VISIBLE LinkActorToModel : public WorldPlugin
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
        this->update_actor_collision_model();
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
        std::thread(std::bind(&LinkActorToModel::QueueThread, this));
    }
    ////////////////////////////////////////////////////////////////////


    /// Pointers.
    private: physics::WorldPtr world;
    // private: std::vector<physics::ActorPtr> actor;
    // private: std::vector<physics::ModelPtr> model;
    private: physics::ActorPtr actor;
    private: physics::ModelPtr model;

    // // last time and positions
    // private: std::vector<ignition::math::Vector3d> last_pos;
    // private: float last_time = 0;

    // // Publishers
    // private: ros::Publisher people;

    public: LinkActorToModel()
    {
    }

    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
      this->world = _world;
      this->InitROS();

      std::string actor_name = "";
      std::string model_name = "";

      if (_sdf->HasElement("actor"))
      {
        actor_name = _sdf->GetElement("actor")->Get<std::string>();
        physics::ModelPtr actor_model = _world->ModelByName(actor_name);
        physics::ActorPtr actor = boost::reinterpret_pointer_cast<physics::Actor>(actor_model);
        this->actor = actor;
      }

      if (_sdf->HasElement("model"))
      {
        model_name = _sdf->GetElement("model")->Get<std::string>();
        this->model = _world->ModelByName(model_name);
      }

      std::cout << "link actor '" << actor_name << "' to model ‘" << model_name << "'." << std::endl;
    }

    private: void update_actor_collision_model()
    {
      // people_msgs::People msg;
      // msg.header.seq = 0;
      // msg.header.stamp = ros::Time::now();
      // msg.header.frame_id = "map";
      // for (size_t i = 0; i < this->actor.size(); i++) {

        double x = this->actor->WorldPose().Pos().X();
        double y = this->actor->WorldPose().Pos().Y();
        double z = this->actor->WorldPose().Pos().Z();

        // people_msgs::Person p;
        // p.name = this->actor.at(i)->GetName();
        // p.position.x = x;
        // p.position.y = y;
        // p.position.z = z;
        // p.velocity.x = (x - this->last_pos.at(i).X())/
        //   (this->world->SimTime().Float()-this->last_time);
        // p.velocity.y = (y - this->last_pos.at(i).Y())/
        //   (this->world->SimTime().Float()-this->last_time);
        // p.velocity.z = (z - this->last_pos.at(i).Z())/
        //   (this->world->SimTime().Float()-this->last_time);
        // // p.reliability
        // p.tagnames
        // p.tags
        // msg.people.push_back(p);

        ignition::math::Pose3d pose = ignition::math::Pose3d();
        pose.Pos().X(x);
        pose.Pos().Y(y);
        pose.Pos().Z(z);
        this->model->SetWorldPose(pose);

        // this->last_pos.at(i).Set(x,y,z);
      // }
      // this->last_time = this->world->SimTime().Float();

      // this->people.publish(msg);
      gazebo::common::Time::MSleep(500);

    }

  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(LinkActorToModel)
}
#endif
