#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/World.hh>

#include <iostream>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>

namespace gazebo
{
  class GAZEBO_VISIBLE TaskControl_Recepcionist : public WorldPlugin
  {

    // ros node
    private: std::unique_ptr<ros::NodeHandle> rosNode{nullptr};

    // services
    private: ros::ServiceServer srv_start;

    // Publishers
    public: ros::Publisher pub_door_1;

    // world
    private: physics::WorldPtr world;

    public: TaskControl_Recepcionist()
    {
    }

    public: void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
      std::cout << "Starting task simulation: Recepcionist!" << std::endl;

      this->world = _world;

      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",
            ros::init_options::NoSigintHandler);
      }
      this->rosNode.reset(new ros::NodeHandle());

      this->srv_start = this->rosNode->advertiseService(
          "/task_control/recepcionist/start",
          &TaskControl_Recepcionist::OnService_Start, this);

      this->pub_door_1 =
          this->rosNode->advertise<std_msgs::String>(
          "/sliding_door_control/door_1/command", 1000);

    }

    public: bool OnService_Start(
        std_srvs::Empty::Request &_req,
        std_srvs::Empty::Response &_res)
    {
      std::cout << "Start recepcionist task" << std::endl;
      this->world->SetPaused(false);

      std_msgs::String msg;
      msg.data = "open";
      this->pub_door_1.publish(msg);

      return true;
    }
  };

  GZ_REGISTER_WORLD_PLUGIN(TaskControl_Recepcionist)
}
