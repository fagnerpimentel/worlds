#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>

class RealTimeFactorPublisher
{
  private:
  // ros NodeHandle and loop rate
  ros::NodeHandle n;

  // parameters
  std::string world_name;

  // subscribers
  gazebo::transport::SubscriberPtr sub_world_stats;

  // publisher
  ros::Publisher factor = n.advertise<std_msgs::Float32>("real_time_factor", 1000);

public:
  RealTimeFactorPublisher(){

    // gazebo transport node initialization
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // parameters
    // n.getParam("real_time_factor_publisher_node/world_name", world_name);

    // subscribers
    sub_world_stats = node->Subscribe("/gazebo/default/world_stats", &RealTimeFactorPublisher::statsCallback, this);

  }

  ~RealTimeFactorPublisher()
  {
    gazebo::client::shutdown();
  }

  void start()
  {
    // ROS_INFO_STREAM("world_name: " << world_name);
    ros::spin();
  }


  void statsCallback(ConstWorldStatisticsPtr &_msg)
  {
    float sim_time = _msg->sim_time().sec() ;
    float real_time = _msg->real_time().sec();
    ROS_DEBUG_STREAM("sim_time: " << sim_time);
    ROS_DEBUG_STREAM("real_time: " << real_time);

    std_msgs::Float32 factor_msg;

    factor_msg.data = (real_time==0) ? 0 : sim_time/real_time;
    ROS_DEBUG_STREAM("factor: " << factor_msg.data);

    this->factor.publish(factor_msg);
  }

};


int main(int argc, char **argv){

  // init gazebo and ros
  ros::init(argc, argv, "real_time_factor_publisher_node");
  gazebo::client::setup(argc, argv);

  // start node
  RealTimeFactorPublisher* rtfp = new RealTimeFactorPublisher();
  rtfp->start();

  return 0;
}
