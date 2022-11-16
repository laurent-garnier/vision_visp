#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <stdexcept>
#include <visp3/core/vpImage.h>

#include <visp_tracker/srv/init.hpp>

#include "visp_tracker/callbacks.h"
#include "visp_tracker/conversion.h"
#include "visp_tracker/names.h"

#include <visp3/mbt/vpMbGenericTracker.h>

void imageCallback_master(vpImage<unsigned char> &image, const sensor_msgs::msg::Image::ConstSharedPtr &msg,
                          const sensor_msgs::msg::CameraInfo::ConstSharedPtr & /*info*/)
{
  try {
    rosImageToVisp(image, msg);
  } catch (std::exception &e) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "dropping frame: " << e.what());
  }
}

void imageCallback(vpImage<unsigned char> &image, std_msgs::msg::Header &header,
                   sensor_msgs::msg::CameraInfo::ConstSharedPtr &info,
                   const sensor_msgs::msg::Image::ConstSharedPtr &msg,
                   const sensor_msgs::msg::CameraInfo::ConstSharedPtr &infoConst)
{
  imageCallback_master(image, msg, info);
  header = msg->header;
  info = infoConst;
}

/*  TODO PORT ROS2
void reconfigureCallback(vpMbGenericTracker &tracker,
                         vpImage<unsigned char>& I,
                         vpMe& moving_edge,
                         vpKltOpencv& kltTracker,
                         std::recursive_mutex& mutex,
                         visp_tracker::ModelBasedSettingsConfig& config,
                         uint32_t level)
{
  mutex.lock ();
  try
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Reconfigure Model Based Hybrid Tracker request received.");

    convertModelBasedSettingsConfigToVpMbTracker<visp_tracker::ModelBasedSettingsConfig>(config, tracker);

    convertModelBasedSettingsConfigToVpMe<visp_tracker::ModelBasedSettingsConfig>(config, moving_edge, tracker);
    //         moving_edge.print();

    convertModelBasedSettingsConfigToVpKltOpencv<visp_tracker::ModelBasedSettingsConfig>(config, kltTracker, tracker);

    vpHomogeneousMatrix cMo;
    tracker.getPose(cMo);

    // Check if the image is ready to use
    if (I.getHeight() != 0 && I.getWidth() != 0) {
      tracker.initFromPose(I, cMo);
    }
  }
  catch (...)
  {
    mutex.unlock ();
    throw;
  }
  mutex.unlock ();
}

void reconfigureEdgeCallback(vpMbGenericTracker &tracker,
                             vpImage<unsigned char>& I,
                             vpMe& moving_edge,
                             std::recursive_mutex& mutex,
                             visp_tracker::ModelBasedSettingsEdgeConfig& config,
                             uint32_t level)
{

  mutex.lock ();
  try
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Reconfigure Model Based Edge Tracker request received.");

    convertModelBasedSettingsConfigToVpMbTracker<visp_tracker::ModelBasedSettingsEdgeConfig>(config, tracker);
    convertModelBasedSettingsConfigToVpMe<visp_tracker::ModelBasedSettingsEdgeConfig>(config, moving_edge, tracker);
    // moving_edge.print();

    // Check if the image is ready to use
    if (I.getHeight() != 0 && I.getWidth() != 0) {
      vpHomogeneousMatrix cMo;
      tracker.getPose(cMo);
      // Could not use initFromPose for edge tracker
      // init() function has to be fixed in the trunk first
      // It might have to reset the meLines
      tracker.setPose(I, cMo);
    }
  }
  catch (...)
  {
    mutex.unlock ();
    throw;
  }
  mutex.unlock ();
}

void reconfigureKltCallback(vpMbGenericTracker &tracker,
                            vpImage<unsigned char>& I,
                            vpKltOpencv& kltTracker,
                            std::recursive_mutex& mutex,
                            visp_tracker::ModelBasedSettingsKltConfig& config,
                            uint32_t level)
{
  mutex.lock ();
  try
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Reconfigure Model Based KLT Tracker request received.");

    convertModelBasedSettingsConfigToVpMbTracker<visp_tracker::ModelBasedSettingsKltConfig>(config, tracker);
    convertModelBasedSettingsConfigToVpKltOpencv<visp_tracker::ModelBasedSettingsKltConfig>(config, kltTracker,
tracker);

    // Check if the image is ready to use
    if (I.getHeight() != 0 && I.getWidth() != 0) {
      vpHomogeneousMatrix cMo;
      tracker.getPose(cMo);
      tracker.initFromPose(I, cMo);
    }
  }
  catch (...)
  {
    mutex.unlock ();
    throw;
  }
  mutex.unlock ();
}
*/
/*
void reInitViewerCommonParameters(rclcpp::Node& nh,
                                  vpMbGenericTracker &tracker)
{
  rclcpp::Client<visp_tracker::srv::Init>::SharedPtr clientViewer =
      nh.create_client<visp_tracker::srv::Init>(visp_tracker::Reconfigure_viewer_service);

   auto srv = std::make_shared<visp_tracker::srv::Init::Request>();
  convertVpMbTrackerToInitRequest(tracker, srv);

  // Call service
  auto srv_request = std::make_shared<visp_tracker::srv::Init::Request>();

   auto result = clientViewer->async_send_request(srv_request);
   // Wait for the result.
 //  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_client");

  //   if (srv.Response.initialization_succeed)
   if (rclcpp::spin_until_future_complete(nh, result) ==
    rclcpp::FutureReturnCode::SUCCESS)  // FIX ?
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Tracker Viewer initialized with success.");
  } else {
      throw std::runtime_error("failed to initialize tracker viewer.");
  }
}
*/
/* TODO PORT ROS2
void reconfigureCallbackAndInitViewer(rclcpp::Node& nh,
                                      vpMbGenericTracker &tracker,
                                      vpImage<unsigned char>& I,
                                      vpMe& moving_edge,
                                      vpKltOpencv& kltTracker,
                                      std::recursive_mutex& mutex,
                                      visp_tracker::ModelBasedSettingsConfig& config,
                                      uint32_t level)
{
  reconfigureCallback(tracker,I,moving_edge,kltTracker,mutex,config,level);
  reInitViewerCommonParameters(nh,tracker);
}

void reconfigureEdgeCallbackAndInitViewer(rclcpp::Node& nh,
                                          vpMbGenericTracker &tracker,
                                          vpImage<unsigned char>& I,
                                          vpMe& moving_edge,
                                          std::recursive_mutex& mutex,
                                          visp_tracker::ModelBasedSettingsEdgeConfig& config,
                                          uint32_t level)
{
  reconfigureEdgeCallback(tracker,I,moving_edge,mutex,config,level);
  reInitViewerCommonParameters(nh,tracker);
}

void reconfigureKltCallbackAndInitViewer(rclcpp::Node& nh,
                                         vpMbGenericTracker &tracker,
                                         vpImage<unsigned char>& I,
                                         vpKltOpencv& kltTracker,
                                         std::recursive_mutex& mutex,
                                         visp_tracker::ModelBasedSettingsKltConfig& config,
                                         uint32_t level)
{
  reconfigureKltCallback(tracker,I,kltTracker,mutex,config,level);
  reInitViewerCommonParameters(nh,tracker);
}
*/
