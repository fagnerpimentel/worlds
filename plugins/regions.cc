#ifndef GAZEBO_PLUGINS_ACTORCONTROL_HH_
#define GAZEBO_PLUGINS_ACTORCONTROL_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/World.hh>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <geometry_msgs/Point.h>
#include <social_worlds/Regions.h>

#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>

#include <thread>
#include <string>

namespace gazebo
{
  class GAZEBO_VISIBLE Regions : public WorldPlugin
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
        std::thread(std::bind(&Regions::QueueThread, this));
    }
    ////////////////////////////////////////////////////////////////////

    private: physics::WorldPtr world;

    private: float region[4];

    // service server
    private: ros::ServiceServer srv_server;

    public: Regions()
    {
    }

    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
      this->world = _world;
      this->InitROS();

      std::string region_name;
      int layer;
      std::string color;

      if (_sdf->HasElement("name"))
      {
        region_name = _sdf->GetElement("name")->Get<std::string>();
      }
      if (_sdf->HasElement("layer"))
      {
        layer = _sdf->GetElement("layer")->Get<int>();
      }
      if (_sdf->HasElement("color"))
      {
        color = _sdf->GetElement("color")->Get<std::string>();
      }

      if (_sdf->HasElement("region"))
      {
        std::string values = _sdf->GetElement("region")->Get<std::string>();
        std::vector<std::string> params;
        boost::split(params, values, boost::is_any_of("\t "));
        this->region[0] = std::stof(params.at(0)); // x0
        this->region[1] = std::stof(params.at(1)); // y0
        this->region[2] = std::stof(params.at(2)); // x1
        this->region[3] = std::stof(params.at(3)); // y1
      }

      AddRegion(region_name, layer, color,
        this->region[0], this->region[1], this->region[2], this->region[3]);

      // Service server: regions
      this->srv_server = this->rosNode->advertiseService(
          "/regions/"+region_name, &Regions::OnService_Regions, this);

      std::cout << "starting region '" << region_name <<"'."  << std::endl;
    }

    private: void AddRegion(std::string name, int layer, std::string color,
                            float x0, float y0, float x1, float y1){

      float x = (x0+x1)/2;
      float y = (y0+y1)/2;
      float w = abs(x0-x1);
      float h = abs(y0-y1);
      sdf::SDF sphereSDF;
      std::string s = boost::str(boost::format(
      "<sdf version ='1.5'>\
        <model name='%1%'>\
         <static>true</static>\
         <pose>%4% %5% 0.001 0 0 0</pose>\
         <link name='link'>\
           <visual name='visual'>\
             <meta>\
               <layer>%2%</layer>\
             </meta>\
             <material>\
               <script>\
                 <name>Gazebo/%3%</name>\
                 <uri>file://media/materials/scripts/gazebo.material</uri>\
               </script>\
             </material>\
             <geometry>\
               <plane>\
                 <normal>0 0 1</normal>\
                 <size>%6% %7%</size>\
               </plane>\
             </geometry>\
           </visual>\
         </link>\
        </model>\
      </sdf>") % name % layer % color % x % y % w % h);
      sphereSDF.SetFromString(s);
      this->world->InsertModelSDF(sphereSDF);

    }

    private: bool OnService_Regions(
        social_worlds::Regions::Request &_req,
        social_worlds::Regions::Response &_res)
    {
      for (float x = this->region[0]; x <= this->region[2]; x+=0.05) {
      for (float y = this->region[1]; y <= this->region[3]; y+=0.05) {
        geometry_msgs::Point p;
        p.x = x;
        p.y = y;
        _res.points.push_back(p);
      }}

      return true;
    }

  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(Regions)
}
#endif
