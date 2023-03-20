#ifndef VISP_TRACKER_CALLBACKS_HH
#define VISP_TRACKER_CALLBACKS_HH
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include "rclcpp/rclcpp.hpp"
#include <string>
#include <visp3/core/vpImage.h>
#include <visp3/klt/vpKltOpencv.h>
#include <visp3/mbt/vpMbGenericTracker.h>
#include <visp3/me/vpMe.h>

/* TODO PORT ROS2
#include <visp_tracker/msg/model_based_settings_config.hpp>

#include <visp_tracker/ModelBasedSettingsEdgeConfig.h>
#include <visp_tracker/ModelBasedSettingsKltConfig.h>
*/

void
imageCallback_master( vpImage< unsigned char > &image, const sensor_msgs::msg::Image::ConstSharedPtr &msg,
                      const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info );

void
imageCallback( vpImage< unsigned char > &image, std_msgs::msg::Header &header,
               sensor_msgs::msg::CameraInfo::ConstSharedPtr &info, const sensor_msgs::msg::Image::ConstSharedPtr &msg,
               const sensor_msgs::msg::CameraInfo::ConstSharedPtr &infoConst );

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
/* TODO PORT ROS2
void reInitViewerCommonParameters(rclcpp::Node& nh,
                                  vpMbGenericTracker &tracker);

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
