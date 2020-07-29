#ifndef GAZEBO_PLUGINS_ACTORCONTROL_HH_
#define GAZEBO_PLUGINS_ACTORCONTROL_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/World.hh>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <geometry_msgs/Point.h>
#include <social_worlds/Region.h>
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

    private: std::vector<social_worlds::Region> regions;

    // service server
    private: ros::ServiceServer srv_server;

    public: Regions()
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

      int i = 0;
      while (_sdf->HasElement("region"))
      {
        i += 1;
        social_worlds::Region region;

        sdf::ElementPtr region_sdf = _sdf->GetElement("region");
        region_sdf->RemoveFromParent();

        std::string region_name;
        int layer;
        std::string color;

        if (region_sdf->HasAttribute ("name"))
        {
          region_name = region_sdf->GetAttribute("name")->GetAsString();
        }
        if (region_sdf->HasElement("layer"))
        {
          layer = region_sdf->GetElement("layer")->Get<int>();
        }
        if (region_sdf->HasElement("color"))
        {
          color = region_sdf->GetElement("color")->Get<std::string>();
        }
        int ii = 0;
        while(region_sdf->HasElement("area")){
          ii += 1;
          sdf::ElementPtr el = region_sdf->GetElement("area");
          el->RemoveFromParent();

          std::string values = el->Get<std::string>();
          std::vector<std::string> params;
          boost::split(params, values, boost::is_any_of("\t "));
          // std::vector<float> area;
          // area.push_back(std::stof(params.at(0))); // x0
          // area.push_back(std::stof(params.at(1))); // y0
          // area.push_back(std::stof(params.at(2))); // x1
          // area.push_back(std::stof(params.at(3))); // y1
          // float x0 = std::stof(params.at(0));
          // float y0 = std::stof(params.at(1));
          // float x1 = std::stof(params.at(2));
          // float y1 = std::stof(params.at(3));
          float x = std::stof(params.at(0));
          float y = std::stof(params.at(1));
          float w = std::stof(params.at(2));
          float l = std::stof(params.at(3));

          region.name = region_name;
          for (float px = x-(w/2); px <= x+(w/2); px+=0.05) {
          for (float py = y-(l/2); py <= y+(l/2); py+=0.05) {
            geometry_msgs::Point p;
            p.x = px;
            p.y = py;
            region.points.push_back(p);
          }}


          AddRegion(region_name+"_"+std::to_string(ii),
           layer, color, x, y, w, l);

        }
        this->regions.push_back(region);

      }

      // Service server: regions
      this->srv_server = this->rosNode->advertiseService(
          "/regions/"+name, &Regions::OnService_Regions, this);

      std::cout << "starting region '" << name <<"'."  << std::endl;
    }

    private: void AddRegion(std::string name, int layer, std::string color,
                            float x, float y, float w, float l){

      // float x = (x0+x1)/2;
      // float y = (y0+y1)/2;
      // float w = abs(x0-x1);
      // float l = abs(y0-y1);
      sdf::SDF regionSDF;
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
      </sdf>") % name % layer % color % x % y % w % l);
      regionSDF.SetFromString(s);
      this->world->InsertModelSDF(regionSDF);

    }

    private: bool OnService_Regions(
        social_worlds::Regions::Request &_req,
        social_worlds::Regions::Response &_res)
    {
      // social_worlds::Region region;
      // std::vector<std::vector<std::vector<float>>>::iterator iter_1;
      // for(iter_1 = this->regions.begin(); iter_1 != this->regions.end(); iter_1++)
      // {
      //   std::vector<geometry_msgs::Point> points;
      //   std::vector<std::vector<float>>::iterator iter_2;
      //   for(iter_2 = (*iter_1).begin(); iter_2 != (*iter_1).end(); iter_2++)
      //   {
      //     for (float x = (*iter_2).at(0); x <= (*iter_2).at(2); x+=0.05) {
      //     for (float y = (*iter_2).at(1); y <= (*iter_2).at(3); y+=0.05) {
      //       geometry_msgs::Point p;
      //       p.x = x;
      //       p.y = y;
      //       points.push_back(p);
      //     }}
      //     region.name = "a";
      //     region.points = points;
      //   }
      //   _res.regions.push_back(region);
      // }
      _res.regions = this->regions;
      return true;
    }

  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(Regions)
}
#endif
