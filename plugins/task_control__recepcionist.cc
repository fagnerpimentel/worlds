#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <ignition/math/Vector2.hh>


#include <iostream>

namespace gazebo
{

  class TaskControl_Recepcionist : public WorldPlugin
  {

    public: TaskControl_Recepcionist() : WorldPlugin()
    {
      std::cout << "Start TaskControl_Recepcionist!" << std::endl;

      // Create our node for communication
      gazebo::transport::NodePtr node(new gazebo::transport::Node());
      node->Init();

      // Publish to a Gazebo topic
      gazebo::transport::PublisherPtr pub =
        node->Advertise<gazebo::msgs::Vector2d>("~/pose_example1");

      // Wait for a subscriber to connect
      pub->WaitForConnection();

      // // Publisher loop...replace with your own code.
      // while (true)
      // {
      //   gazebo::common::Time::MSleep(100);
      //   ignition::math::Vector2<double> vect(5, 7);
      //   gazebo::msgs::Vector2d msg;
      //   gazebo::msgs::Set(&msg, vect);
      //   pub->Publish(msg);
      // }
      //
      // // Make sure to shut everything down.
      // gazebo::transport::fini();
    }

    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
    {

    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(TaskControl_Recepcionist)
}
