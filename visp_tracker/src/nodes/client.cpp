#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include "visp_tracker/tracker-client.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv/*, "tracker_mbt_client"*/);
  auto node_master = std::make_shared<rclcpp::Node>("tracker_mbt_client", "my_ns");
  volatile bool exiting;
  auto node = std::make_shared<visp_tracker::TrackerClient>(
     node_master,
     nullptr,
     exiting,
     5u
     );
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
