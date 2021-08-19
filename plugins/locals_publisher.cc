#ifndef GAZEBO_PLUGINS_LOCALSPUBLISHER_HH_
#define GAZEBO_PLUGINS_LOCALSPUBLISHER_HH_

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
#include <social_msgs/Locals.h>
#include <social_msgs/Local.h>

#include <thread>
#include <iostream>
#include <boost/algorithm/string.hpp>


void euler_to_quaternion(float roll, float pitch, float yaw, float *qx, float *qy, float *qz, float *qw)
{
  *qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2);
  *qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2);
  *qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2);
  *qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2);
}

void quaternion_to_euler(float x, float y, float z, float w, float *yaw, float *pitch, float *roll)
{
  float t0 = +2.0 * (w * x + y * z);
  float t1 = +1.0 - 2.0 * (x * x + y * y);
  *roll = atan2(t0, t1);
  float t2 = +2.0 * (w * y - z * x);
  t2 = (t2 > +1.0)? +1.0 : t2;
  t2 = (t2 < -1.0)? -1.0 : t2;
  *pitch = asin(t2);
  float t3 = +2.0 * (w * z + x * y);
  float t4 = +1.0 - 2.0 * (y * y + z * z);
  *yaw = atan2(t3, t4);
}

double GenerateRandom(double min, double max)
{
    static bool first = true;
    if (first)
    {
        srand(time(NULL));
        first = false;
    }
    if (min > max)
    {
        std::swap(min, max);
    }
    return min + (double)rand() * (max - min) / (double)RAND_MAX;
}

namespace gazebo
{

  struct local_info {
    std::string name;
    std::string type;
    geometry_msgs::Pose pose;
    // std::string mesh;
  } ;

  class GAZEBO_VISIBLE LocalsPublisher : public WorldPlugin
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
        this->publish_locals();
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
        std::thread(std::bind(&LocalsPublisher::QueueThread, this));
    }
    ////////////////////////////////////////////////////////////////////


    /// Pointers.
    private: physics::WorldPtr world;
    private: std::vector<local_info> locals_info;

    // Publishers
    private: ros::Publisher pub_locals;

    public: LocalsPublisher()
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

      while(_sdf->HasElement("local"))
      {
        sdf::ElementPtr el = _sdf->GetElement("local");
        el->RemoveFromParent();

        local_info li;
        li.name = el->GetAttribute("name")->GetAsString();

        std::string values = el->Get<std::string>();
        std::vector<std::string> params;
        boost::split(params, values, boost::is_any_of("\t "));

        li.pose.position.x = std::stof(params.at(0));
        li.pose.position.y = std::stof(params.at(1));
        li.pose.position.z = std::stof(params.at(2));
        li.pose.orientation.x = std::stof(params.at(3));
        li.pose.orientation.y = std::stof(params.at(4));
        li.pose.orientation.z = std::stof(params.at(5));
        // li.pose.orientation.w = ;
        this->locals_info.push_back(li);

      }

      // locals publisher
      this->pub_locals =
          this->rosNode->advertise<social_msgs::Locals>(
          "/"+name, 1000);

      std::cout << "starting locals publisher." << std::endl;
    }

    private: void publish_locals()
    {
      social_msgs::Locals msg;
      msg.header.seq = 0;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "map";

      for (size_t i = 0; i < this->locals_info.size(); i++)
      {
        social_msgs::Local local;
        if(locals_info.at(i).name.rfind("_random_", 0) == 0)
        {


          double x = GenerateRandom(this->locals_info.at(i).pose.position.x - this->locals_info.at(i).pose.orientation.x/2,
                             this->locals_info.at(i).pose.position.x + this->locals_info.at(i).pose.orientation.x/2);
          double y = GenerateRandom(this->locals_info.at(i).pose.position.y - this->locals_info.at(i).pose.orientation.y/2,
                             this->locals_info.at(i).pose.position.y + this->locals_info.at(i).pose.orientation.y/2);
          double z = GenerateRandom(this->locals_info.at(i).pose.position.z - this->locals_info.at(i).pose.orientation.z/2,
                             this->locals_info.at(i).pose.position.z + this->locals_info.at(i).pose.orientation.z/2);

          double ang = GenerateRandom(-3.1415,3.1415);
          float qx, qy, qz, qw;
          euler_to_quaternion(0,0,ang,&qx, &qy, &qz, &qw);

          local.name = this->locals_info.at(i).name;
          local.name.erase(0,8);
          local.pose.position.x = x;
          local.pose.position.y = y;
          local.pose.position.z = z;
          local.pose.orientation.x = qx;
          local.pose.orientation.y = qy;
          local.pose.orientation.z = qz;
          local.pose.orientation.w = qw;
        }
        else
        {
          float qx, qy, qz, qw;
          euler_to_quaternion(
            this->locals_info.at(i).pose.orientation.x,
            this->locals_info.at(i).pose.orientation.y,
            this->locals_info.at(i).pose.orientation.z,
            &qx, &qy, &qz, &qw);

          local.name = this->locals_info.at(i).name;
          local.pose.position = this->locals_info.at(i).pose.position;
          local.pose.orientation.x = qx;
          local.pose.orientation.y = qy;
          local.pose.orientation.z = qz;
          local.pose.orientation.w = qw;

        }
        msg.locals.push_back(local);
      }

      this->pub_locals.publish(msg);
      gazebo::common::Time::MSleep(500);

    }

  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(LocalsPublisher)
}
#endif
