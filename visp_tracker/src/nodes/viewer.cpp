#include <stdexcept>

#include <rclcpp/rclcpp.hpp>


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv, "tracker_mbt_viewer");

  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;

  nodelet.load
      (ros::this_node::getName (), "visp_tracker/TrackerViewer", remap, nargv);

  rclcpp::spin();

  return 0;
}
