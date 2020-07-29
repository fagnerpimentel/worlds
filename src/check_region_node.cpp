#include <ros/ros.h>
#include <algorithm>
// #include <gazebo/gazebo_client.hh>
// #include <gazebo/transport/transport.hh>
#include <gazebo_msgs/ModelStates.h>
#include <social_worlds/Regions.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

#include <typeinfo>

class CheckRegion
{
  private:
  // rate
  // ros::Rate loop_rate;

  // parameters
  std::string region_name;
  std::string model_name;

  // variables
  geometry_msgs::Pose model_pose;

  // publisher
  ros::Publisher pub_check;

  // Subscriber
  // gazebo::transport::SubscriberPtr sub_models;
  ros::Subscriber sub_models;

  // service client
  ros::ServiceClient srv_region;


public:
  CheckRegion(){

    // gazebo transport node initialization
    // gazebo::transport::NodePtr node(new gazebo::transport::Node());
    // node->Init();

    // ros NodeHandle and loop rate
    ros::NodeHandle n("~");
    // this->loop_rate = ros::Rate(10);


    // parameters
    n.getParam("region_name", this->region_name);
    n.getParam("model_name", this->model_name);
    ROS_INFO_STREAM("region_name: " << this->region_name);
    ROS_INFO_STREAM("model_name: " << this->model_name);

    this->pub_check = n.advertise<social_worlds::Region>
      ("", 1000);

    // this->sub_models = node->Subscribe("/gazebo/model_states",
    //   &CheckRegion::modelsCallback, this);
    this->sub_models = n.subscribe("/gazebo/model_states", 1000,
      &CheckRegion::modelsCallback, this);

    std::string s = "/regions/" + this->region_name;
    this->srv_region = n.serviceClient<social_worlds::Regions>(s);
    this->srv_region.waitForExistence();

  }

  ~CheckRegion()
  {
    // gazebo::client::shutdown();
  }

  // void modelsCallback(ConstModelStatesPtr &_msg)
  void modelsCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
  {
    std::vector<std::string> models = msg->name;
    std::vector<std::string>::iterator it =
      std::find(models.begin(), models.end(), this->model_name);
      if (it != models.end()){
        int index = std::distance(models.begin(), it);
        this->model_pose = msg->pose[index];
      }
  }

  void start()
  {
    social_worlds::Regions srv;
    bool ok = false;
    do{
      ROS_INFO_STREAM("Trying to get region points.");
      ok = this->srv_region.call(srv);
      ros::Rate(10).sleep();
    }while(!ok);

    while (ros::ok())
    {

      social_worlds::Region region;

      std::vector<social_worlds::Region>::iterator iter_1;
      for (iter_1 = srv.response.regions.begin(); iter_1 < srv.response.regions.end(); iter_1++)
      {

        std::vector<geometry_msgs::Point>::iterator iter_2;
        for (iter_2 = (*iter_1).points.begin(); iter_2 < (*iter_1).points.end(); iter_2++)
        {

          double dist = sqrt(
            pow(this->model_pose.position.x - (*iter_2).x,2)+
            pow(this->model_pose.position.y - (*iter_2).y,2));

          if(dist <= 0.05)
          {
            geometry_msgs::Point p;
            p.x = (*iter_2).x;
            p.y = (*iter_2).y;
            region.name = (*iter_1).name;
            region.points.push_back(p);
            ROS_INFO_STREAM("point: (" << p.x << "," << p.y << "). Dist: " << dist << ".");
          }
        }
      }

      if(region.points.size() > 0) this->pub_check.publish(region);


      ros::Rate(10).sleep();
      ros::spinOnce();
    }

  }

};


int main(int argc, char **argv){

  // init gazebo and ros
  ros::init(argc, argv, "check_region_node");
  // gazebo::client::setup(argc, argv);

  // start node
  CheckRegion* cr = new CheckRegion();
  cr->start();

  return 0;
}
