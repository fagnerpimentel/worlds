#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Actor.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Quaternion.hh>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <std_msgs/String.h>
#include <social_worlds/Start.h>
#include <social_worlds/ActorTrajectory.h>

#include <unistd.h>
#include <iostream>

#define deg2rad(X) X*3.1415/180
#define rad2deg(X) X*180/3.1415

namespace gazebo
{
  class GAZEBO_VISIBLE TaskControl_Receptionist : public WorldPlugin
  {

    // ros node
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    // A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;
    // A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;

    // service servers
    private: ros::ServiceServer srv_server;

    // service clients
    private: ros::ServiceClient srv_client_human_1;
    private: ros::ServiceClient srv_client_human_2;
    private: ros::ServiceClient srv_client_human_3;

    // Publishers
    private: ros::Publisher pub_door_1;
    private: ros::Publisher pub_door_2;
    private: ros::Publisher pub_door_3;
    private: ros::Publisher pub_door_4;
    private: ros::Publisher pub_door_5;

    // actors
    private: physics::ActorPtr human_1;
    private: physics::ActorPtr human_2;
    private: physics::ActorPtr human_3;

    // world
    private: physics::WorldPtr world;

    // state
    private: int state;

    public: TaskControl_Receptionist()
    {
    }

    public: void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
      std::cout << "Starting task simulation: Receptionist!" << std::endl;
      this->world = _world;
      this->state = 0;

      // start ROS
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",
            ros::init_options::NoSigintHandler);
      }
      this->rosNode.reset(new ros::NodeHandle());

      // Service server: task
      this->srv_server = this->rosNode->advertiseService(
          "/task_control/receptionist/start",
          &TaskControl_Receptionist::OnService_Start, this);

      // Service clients: actors
      this->srv_client_human_1 = this->rosNode->serviceClient
          <social_worlds::ActorTrajectory>
          ("/actor_control/human_1/action");
      this->srv_client_human_2 = this->rosNode->serviceClient
          <social_worlds::ActorTrajectory>
          ("/actor_control/human_2/action");
      this->srv_client_human_3 = this->rosNode->serviceClient
          <social_worlds::ActorTrajectory>
          ("/actor_control/human_3/action");

      // Doors publishers
      this->pub_door_1 =
          this->rosNode->advertise<std_msgs::String>(
          "/sliding_door_control/door_1/command", 1000);
      this->pub_door_2 =
          this->rosNode->advertise<std_msgs::String>(
          "/sliding_door_control/door_2/command", 1000);
      this->pub_door_3 =
          this->rosNode->advertise<std_msgs::String>(
          "/sliding_door_control/door_3/command", 1000);
      this->pub_door_4 =
          this->rosNode->advertise<std_msgs::String>(
          "/sliding_door_control/door_4/command", 1000);
      this->pub_door_5 =
          this->rosNode->advertise<std_msgs::String>(
          "/sliding_door_control/door_5/command", 1000);

      // Actors models
      this->human_1 = boost::reinterpret_pointer_cast
          <physics::Actor>(this->world->ModelByName("human_1"));
      this->human_2 = boost::reinterpret_pointer_cast
          <physics::Actor>(this->world->ModelByName("human_2"));
      this->human_3 = boost::reinterpret_pointer_cast
          <physics::Actor>(this->world->ModelByName("human_3"));

      // Spin up the queue helper thread.
      this->rosQueueThread =
        std::thread(std::bind(&TaskControl_Receptionist::QueueThread, this));

    }

    // ROS helper function that processes messages
    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

    private: bool OnService_Start(
        social_worlds::Start::Request &_req,
        social_worlds::Start::Response &_res)
    {
      // strat task
      std::cout << "Receptionist task: Start." << std::endl;
      this->world->SetPaused(false);

      // open inside doors
      std_msgs::String msg;
      msg.data = "open";
      this->pub_door_2.publish(msg);
      this->pub_door_3.publish(msg);
      this->pub_door_4.publish(msg);

      std::cout << "Receptionist task: Phase 1." << std::endl;
      this->state = 1;
      this->phase_1();
      std::cout << "Receptionist task: Phase 2." << std::endl;
      this->state = 2;
      this->phase_2();
      std::cout << "Receptionist task: Phase 3." << std::endl;
      this->state = 3;
      this->phase_3();

      return true;
    }


    private: void phase_1()
    {
      // variables
      geometry_msgs::Pose p;
      std_msgs::String msg;
      social_worlds::ActorTrajectory trajectory;
      double angle = 180;
      ignition::math::Vector3<double> position(2.7,-8,1.05);
      ignition::math::Quaternion<double> orientation(deg2rad(90),deg2rad(0),deg2rad(angle));
      p.position.x = position.X();
      p.position.y = position.Y();
      p.position.z = position.Z();
      p.orientation.x = orientation.X();
      p.orientation.y = orientation.Y();
      p.orientation.z = orientation.Z();
      p.orientation.w = orientation.W();

      // open entrance door
      msg.data = "open";
      this->pub_door_1.publish(msg);
      // gazebo::common::Time::MSleep(20000);
      usleep(3000000); // microseconds

      // enter home
      trajectory.request.animation.data = "walking";
      for (size_t i = 0; i < 10; i++) {
        position.Y(position.Y()+0.5);
        p.position.y = position.Y();
        trajectory.request.waypoints.push_back(p);
      }
      this->srv_client_human_2.call(trajectory);


      // close entrance door
      // msg.data = "close";
      // this->pub_door_1.publish(msg);

    }

    private: void phase_2()
    {

      // variables
      geometry_msgs::Pose p;
      social_worlds::ActorTrajectory trajectory;
      double angle = 180;
      ignition::math::Vector3<double> position(2.7,-3,1.05);
      ignition::math::Quaternion<double> orientation(deg2rad(90),deg2rad(0),deg2rad(angle));
      p.position.x = position.X();
      p.position.y = position.Y();
      p.position.z = position.Z();
      p.orientation.x = orientation.X();
      p.orientation.y = orientation.Y();
      p.orientation.z = orientation.Z();
      p.orientation.w = orientation.W();

      // follow
      trajectory.request.animation.data = "walking";
      for (size_t i = 0; i < 6; i++) {
        position.Y(position.Y()+0.5);
        p.position.y = position.Y();
        trajectory.request.waypoints.push_back(p);
      }
      this->srv_client_human_2.call(trajectory);

    }

    private: void phase_3()
    {
      // variables
      geometry_msgs::Pose p;
      social_worlds::ActorTrajectory trajectory;
      double angle = 180;
      ignition::math::Vector3<double> position(2.7,0,1.05);
      ignition::math::Quaternion<double> orientation(deg2rad(90),deg2rad(0),deg2rad(angle));
      p.position.x = position.X();
      p.position.y = position.Y();
      p.position.z = position.Z();
      p.orientation.x = orientation.X();
      p.orientation.y = orientation.Y();
      p.orientation.z = orientation.Z();
      p.orientation.w = orientation.W();

      // sit_down
      trajectory.request.animation.data = "walking";
      for (size_t i = 0; i < 2; i++) {
        position.Y(position.Y()+0.5);
        p.position.y = position.Y();
        trajectory.request.waypoints.push_back(p);
      }
      for (size_t i = 0; i < 2; i++) {
        position.X(position.X()+0.5);
        p.position.x = position.X();
        trajectory.request.waypoints.push_back(p);
      }
      for (size_t i = 0; i < 3; i++) {
        position.Y(position.Y()+0.5);
        p.position.y = position.Y();
        trajectory.request.waypoints.push_back(p);
      }
      angle = -90;
      orientation.Euler(deg2rad(90),deg2rad(0),deg2rad(angle));
      p.orientation.x = orientation.X();
      p.orientation.y = orientation.Y();
      p.orientation.z = orientation.Z();
      p.orientation.w = orientation.W();
      trajectory.request.waypoints.push_back(p);
      this->srv_client_human_2.call(trajectory);

    }

  };

  GZ_REGISTER_WORLD_PLUGIN(TaskControl_Receptionist)
}
