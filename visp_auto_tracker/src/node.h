#ifndef __VISP_AUTO_TRACKER_NODE_H__
#define __VISP_AUTO_TRACKER_NODE_H__
#include "rclcpp/rclcpp.h"

#include <visp3/core/vpConfig.h>
#include "libauto_tracker/tracking.h"
#include <string>

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include <sstream>

namespace visp_auto_tracker{
  class Node{
  private:
    std::mutex lock_;
    rclcpp::node::Node n_;
    unsigned long queue_size_;
    std::string tracker_config_path_;
    std::string model_description_;
    std::string model_path_;
    std::string model_name_;
    std::string code_message_;
    std::string tracker_ref_frame_;
    
    bool debug_display_;

    vpImage<vpRGBa> I_; // Image used for debug display
    std_msgs::msg::Header image_header_;
    bool got_image_;
    vpCameraParameters cam_;

    tracking::Tracker* t_;
    CmdLine cmd_;

    void waitForImage();

    void frameCallback(const sensor_msgs::msg::Image::ConstPtr& image, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_info);

  public:
    Node();
    void spin();
  };
};
#endif
