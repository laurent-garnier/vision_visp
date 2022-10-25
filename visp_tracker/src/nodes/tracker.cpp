#include <stdexcept>

//#include <nodelet/loader.h>
#include <rclcpp/rclcpp.hpp>
#include "visp_tracker/tracker.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv/*, "tracker_mbt"*/);
  auto node_master = std::make_shared<rclcpp::Node>("tracker_mbt", "my_ns");
  bool exiting;
  auto node = std::make_shared<visp_tracker::Tracker>(
         node_master,
         nullptr,
         exiting,
         5u
  );
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}
