#include "main.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "drone_flight_node");
  ros::NodeHandle nh_private("~");

  drone_flight_class drone_flight_class_(nh_private);

  ros::AsyncSpinner spinner(3); // Use 3 threads
  spinner.start();
  ros::waitForShutdown();

  return 0;
}