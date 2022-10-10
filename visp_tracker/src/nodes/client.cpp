#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <visp_tracker/tracker-client.h>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv, "tracker_mbt_client");

  auto node = std::make_shared<TrackerClient>();

  try {
    if (node->setup() != 0) {
      RCLCPP_ERROR(node->get_logger(), "Afma6 setup failed... \n");
      return EXIT_FAILURE;
    }

    rclcpp::WallRate loop_rate(100ms);

    while (rclcpp::ok()) {
      node->publish();

      rclcpp::spin(node);
      loop_rate.sleep();
    }
  } catch (const vpException &e) {
    RCLCPP_ERROR(node->get_logger(), "Catch ViSP exception: %s", e.getMessage());
  } catch (const rclcpp::exceptions::RCLError &e) {
    RCLCPP_ERROR(node->get_logger(), "Unexpectedly failed with %s", e.what());
  }

  return 0;
}
