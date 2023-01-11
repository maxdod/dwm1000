#include "ros/ros.h"
#include "localizer_dwm1001/askPosition.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_position");
  

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<localizer_dwm1001::askPosition>("getPosition");
  localizer_dwm1001::askPosition srv;
  if (client.call(srv))
  {
    std::cout << srv.response.posX << " "  << srv.response.posY << std::endl;
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}