#include "visp_auto_tracker/autotracker.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  visp_auto_tracker::AutoTracker().spin();
  return 0;
}
