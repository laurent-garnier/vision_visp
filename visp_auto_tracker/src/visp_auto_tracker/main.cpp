#include "visp_auto_tracker/node.h"

int main(int argc,char** argv){
  rclcpp::init(argc, argv, "visp_auto_tracker");
  visp_auto_tracker::Node().spin();
  return 0;
}
