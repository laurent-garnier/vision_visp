#ifndef VISP_TRACKER_CALLBACKS_HH
# define VISP_TRACKER_CALLBACKS_HH
#include <image_transport/image_transport.h>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include <string>
#include "rclcpp/rclcpp.hpp"
#include <visp3/core/vpImage.h>
#include <visp3/mbt/vpMbGenericTracker.h>
#include <visp3/me/vpMe.h>
#include <visp3/klt/vpKltOpencv.h>

/* TODO FIX:
#include <visp_tracker/msg/model_based_settings_config.hpp>

#include <visp_tracker/ModelBasedSettingsKltConfig.h>
#include <visp_tracker/ModelBasedSettingsEdgeConfig.h>
*/

void
imageCallback(vpImage<unsigned char>& image,
              const sensor_msgs::msg::Image::ConstPtr& msg,
              const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info);

void
imageCallback(vpImage<unsigned char>& image,
              std_msgs::msg::Header& header,
              sensor_msgs::msg::CameraInfo::ConstSharedPtr& info,
              const sensor_msgs::msg::Image::ConstPtr& msg,
              const sensor_msgs::msg::CameraInfo::ConstSharedPtr& infoConst);

image_transport::CameraSubscriber::Callback
bindImageCallback(vpImage<unsigned char>& image);

image_transport::CameraSubscriber::Callback
bindImageCallback(vpImage<unsigned char>& image,
                  std_msgs::msg::Header& header,
                  sensor_msgs::msg::CameraInfo::ConstSharedPtr& info);

/*
void reconfigureCallback(vpMbGenericTracker &tracker,
                         vpImage<unsigned char>& I,
                         vpMe& moving_edge,
                         vpKltOpencv& kltTracker,
                         std::recursive_mutex& mutex,
                         visp_tracker::ModelBasedSettingsConfig& config,
                         uint32_t level);

void reconfigureEdgeCallback(vpMbGenericTracker &tracker,
                             vpImage<unsigned char>& I,
                             vpMe& moving_edge,
                             std::recursive_mutex& mutex,
                             visp_tracker::ModelBasedSettingsEdgeConfig& config,
                             uint32_t level);

void reconfigureKltCallback(vpMbGenericTracker &tracker,
                            vpImage<unsigned char>& I,
                            vpKltOpencv& kltTracker,
                            std::recursive_mutex& mutex,
                            visp_tracker::ModelBasedSettingsKltConfig& config,
                            uint32_t level);
*/
void reInitViewerCommonParameters(rclcpp::Node& nh,
                                  vpMbGenericTracker &tracker);

/* TODO FIX 
void reconfigureCallbackAndInitViewer(rclcpp::Node& nh,
                                      vpMbGenericTracker &tracker,
                                      vpImage<unsigned char>& I,
                                      vpMe& moving_edge,
                                      vpKltOpencv& kltTracker,
                                      std::recursive_mutex& mutex,
                                      visp_tracker::ModelBasedSettingsConfig& config,
                                      uint32_t level);

void reconfigureEdgeCallbackAndInitViewer(rclcpp::Node& nh,
                                          vpMbGenericTracker &tracker,
                                          vpImage<unsigned char>& I,
                                          vpMe& moving_edge,
                                          std::recursive_mutex& mutex,
                                          visp_tracker::ModelBasedSettingsEdgeConfig& config,
                                          uint32_t level);

void reconfigureKltCallbackAndInitViewer(rclcpp::Node& nh,
                                         vpMbGenericTracker &tracker,
                                         vpImage<unsigned char>& I,
                                         vpKltOpencv& kltTracker,
                                         std::recursive_mutex& mutex,
                                         visp_tracker::ModelBasedSettingsKltConfig& config,
                                         uint32_t level);
*/

#endif //! VISP_TRACKER_CALLBACKS_HH
