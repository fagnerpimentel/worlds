#ifndef GAZEBO_PLUGINS_LOADHUMAN_HH_
#define GAZEBO_PLUGINS_LOADHUMAN_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/World.hh>

#include <ros/ros.h>
#include <ros/callback_queue.h>

// #include <geometry_msgs/Point.h>
// #include <social_worlds/Regions.h>

#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>

#include <thread>
#include <string>

namespace gazebo
{
  class GAZEBO_VISIBLE LoadHuman : public WorldPlugin
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
        std::thread(std::bind(&LoadHuman::QueueThread, this));
    }
    ////////////////////////////////////////////////////////////////////

    private: physics::WorldPtr world;

    // private: std::vector<std::vector<float>> regions;

    // service server
    private: ros::ServiceServer srv_server;

    public: LoadHuman()
    {
    }

    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
      this->world = _world;
      this->InitROS();

      std::string human_name;
      // int layer;
      // std::string color;

      if (_sdf->HasAttribute ("name"))
      {
        human_name = _sdf->GetAttribute("name")->GetAsString();
      }
      if (_sdf->HasElement("pose"))
      {

        sdf::ElementPtr el = _sdf->GetElement("pose");
        std::string pose = el->Get<std::string>();
        // std::vector<std::string> params;
        // boost::split(params, values, boost::is_any_of("\t "));
        // std::vector<float> v_pose;
        // v_pose.push_back(std::stof(params.at(0))); // x
        // v_pose.push_back(std::stof(params.at(1))); // y
        // v_pose.push_back(std::stof(params.at(2))); // z
        // v_pose.push_back(std::stof(params.at(3))); // R
        // v_pose.push_back(std::stof(params.at(4))); // P
        // v_pose.push_back(std::stof(params.at(4))); // Y

        AddHuman(human_name, pose);

      }

      std::cout << "Load human :'" << human_name <<"'."  << std::endl;
    }

    private: void AddHuman(std::string name, std::string pose){

      sdf::SDF modelSDF;
      std::string s1 = boost::str(boost::format(
      "<sdf version ='1.5'>\
      <include>\
        <name>model_%1%</name>\
        <pose>%2%</pose>\
        <uri>model://human_cylinder</uri>\
      </include>\
      </sdf>") % name % pose);
      modelSDF.SetFromString(s1);
      this->world->InsertModelSDF(modelSDF);

      sdf::SDF actorSDF;
      std::string s2 = boost::str(boost::format(
      "<sdf version ='1.5'>\
      <actor name='actor_%1%'>\
        <pose>%2%</pose>\
        <skin>\
          <filename>stand.dae</filename>\
        </skin>\
        <script>\
          <trajectory id='0' type='stand'>\
            <waypoint>\
              <time>0</time>\
              <pose>%2%</pose>\
            </waypoint>\
            <waypoint>\
              <time>3</time>\
              <pose>%2%</pose>\
            </waypoint>\
          </trajectory>\
        </script>\
      </actor>\
      </sdf>") % name % pose);
      actorSDF.SetFromString(s2);
      this->world->InsertModelSDF(actorSDF);

    }

    // private: bool OnService_Regions(
    //     social_worlds::Regions::Request &_req,
    //     social_worlds::Regions::Response &_res)
    // {
    //   for(std::vector<std::vector<float>>::iterator iter = this->regions.begin();
    //     iter != this->regions.end(); iter++)
    //   {
    //     for (float x = (*iter).at(0); x <= (*iter).at(2); x+=0.05) {
    //     for (float y = (*iter).at(1); y <= (*iter).at(3); y+=0.05) {
    //       geometry_msgs::Point p;
    //       p.x = x;
    //       p.y = y;
    //       _res.points.push_back(p);
    //     }}
    //   }
    //   return true;
    // }

  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(LoadHuman)
}
#endif
