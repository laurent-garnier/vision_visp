#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include "visp_tracker/tracker-viewer.h"


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv/*, "tracker_mbt_viewer"*/);
  auto node_master = std::make_shared<rclcpp::Node>("tracker_mbt_viewer", "my_ns");
  volatile bool exiting;
  auto node = std::make_shared<visp_tracker::TrackerViewer>(
       node_master,
       nullptr,
       exiting,
       5u
  );
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
