#include <stdexcept>

#include "visp_tracker/tracker-client.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv /*, "tracker_mbt_client"*/);
  auto node = std::make_shared<visp_tracker::TrackerClient>();
  // rclcpp::spin(node);
  node->spin();
  rclcpp::shutdown();

  return 0;
}
