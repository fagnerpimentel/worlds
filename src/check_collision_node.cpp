#include <ros/ros.h>
#include <std_msgs/String.h>
#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>

class CheckCollision
{
  private:

  // parameters
  std::string world_name;
  std::string robot_name;

  // subscribers
  gazebo::transport::SubscriberPtr sub_contacts;

  // publisher
  ros::Publisher collision;

public:
  CheckCollision(){
    // ros NodeHandle and loop rate
    ros::NodeHandle n("~");

    // gazebo transport node initialization
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // parameters
    n.getParam("world_name", world_name);
    n.getParam("robot_name", robot_name);

    // subscribers
    sub_contacts = node->Subscribe("/gazebo/default/physics/contacts", &CheckCollision::contactsCallback, this);

    // publisher
    collision = n.advertise<std_msgs::String>("/collision", 1000);

  }

  ~CheckCollision()
  {
    gazebo::client::shutdown();
  }

  void start()
  {
    ROS_INFO_STREAM("world_name: " << world_name);
    ROS_INFO_STREAM("robot_name: " << robot_name);
    ros::spin();
  }


  void contactsCallback(ConstContactsPtr &_msg)
  {
    std::string collision_obj_1 = "";
    std::string collision_obj_2 = "";
    std_msgs::String collision_msg;
    collision_msg.data = "";

    for (size_t i = 0; i < _msg->contact_size() ; i++)
    {
      collision_obj_1 = _msg->contact(i).collision1();
      collision_obj_2 = _msg->contact(i).collision2();

      if (collision_obj_1.find("ground_plane") != std::string::npos or
          collision_obj_2.find("ground_plane") != std::string::npos)
      {
        continue;
      }

      if (collision_obj_1.find(this->robot_name) != std::string::npos)
      {
        collision_msg.data = collision_obj_2;
        break;
      }
      if (collision_obj_2.find(this->robot_name) != std::string::npos)
      {
        collision_msg.data = collision_obj_1;
        break;
      }

    }

    if(collision_msg.data != ""){
      this->collision.publish(collision_msg);
    }
  }

};


int main(int argc, char **argv){

  // init gazebo and ros
  ros::init(argc, argv, "check_collision_node");
  gazebo::client::setup(argc, argv);

  // start node
  CheckCollision* c = new CheckCollision();
  c->start();

  return 0;
}
