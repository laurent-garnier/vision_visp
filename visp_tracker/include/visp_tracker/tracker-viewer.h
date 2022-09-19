#ifndef VISP_TRACKER_TRACKER_VIEWER_HH
# define VISP_TRACKER_TRACKER_VIEWER_HH

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <visp_tracker/srv/init.hpp>
#include <visp_tracker/msg/moving_edge_sites.hpp>
#include <visp_tracker/msg/klt_points.hpp>

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpImage.h>
#include <visp3/mbt/vpMbGenericTracker.h>

#include <rclcpp/rclcpp.hpp>

namespace visp_tracker
{
  /// \brief Monitors the tracking result provided by the tracking node.
  class TrackerViewer : public rclcpp::Node
  {
  public:
    /// \brief ViSP image type
    typedef vpImage<unsigned char> image_t;

    /// \brief Synchronization policy
    ///
    /// This is used to make sure that the image, the object position
    /// and the moving edge sites are synchronized. This may not be the
    /// case as these informations are published on different topics.
    /// The approximate time allows light differences in timestamps
    /// which are not critical as this is only a viewer.
    typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo,
    geometry_msgs::msg::PoseWithCovarianceStamped,
    visp_tracker::msg::MovingEdgeSites,
    visp_tracker::msg::KltPoints
    > syncPolicy_t;

    /// \brief Constructor.
    TrackerViewer(rclcpp::Node& nh,
                  rclcpp::Node& privateNh,
                  volatile bool& exiting,
                  unsigned queueSize = 5u);

    /// \brief Display camera image, tracked object position and moving
    /// edge sites.
    void spin();
  protected:
    /// \brief Initialize the tracker.
    void initializeTracker();

    /// \brief Initialize the common parameters (visibility angles, etc)
    void loadCommonParameters();

    /// \brief Make sure the topics we subscribe already exist.
    void checkInputs();

    /// \brief Hang until the first image is received.
    void waitForImage();

    bool initCallback(visp_tracker::srv::Init::Request& req,
                      visp_tracker::srv::Init::Response& res);

    bool reconfigureCallback(visp_tracker::srv::Init::Request& req,
                             visp_tracker::srv::Init::Response& res);

    /// \brief Callback used to received synchronized data.
    void
    callback
    (const sensor_msgs::msg::Image::ConstPtr& imageConst,
     const sensor_msgs::msg::CameraInfo::ConstSharedPtr& infoConst,
     const geometry_msgs::msg::PoseWithCovarianceStamped::ConstPtr& trackingResult,
     const visp_tracker::msg::MovingEdgeSites::ConstPtr& sitesConst,
     const visp_tracker::msg::KltPoints::ConstPtr& kltConst);

    void timerCallback();

    /// \brief Display moving edge sites.
    void displayMovingEdgeSites();
    /// \brief Display KLT points that are tracked.
    void displayKltPoints();

  private:
    bool exiting ()
    {
      return exiting_ || !rclcpp::ok();
    }

    volatile bool& exiting_;

    /// \brief Queue size for all subscribers.
    unsigned queueSize_;

    std::shared_ptr<rclcpp::Node> nodeHandle_;
    std::shared_ptr<rclcpp::Node> nodeHandlePrivate_;

    /// \brief Image transport used to receive images.
    image_transport::ImageTransport imageTransport_;

    double frameSize_;

    /// \name Topics and services strings.
    /// \{

    /// \brief Full topic name for rectified image.
    std::string rectifiedImageTopic_;
    /// \brief Full topic name for camera information.
    std::string cameraInfoTopic_;

    /// \}
  //define services
  ros::ServiceServer initService_;
  ros::ServiceServer set_camera_info_service_;
  >> rclcpp::Service<sensor_msgs::srv::SetCameraInfo>::SharedPtr set_camera_info_service_;

  set_camera_info_service_ = n_.advertiseService(visp_camera_calibration::set_camera_info_service,set_camera_info_callback);

  // define services
  >> rclcpp::Client<sensor_msgs::srv::SetCameraInfo>::SharedPtr set_camera_info_service_;
  >> set_camera_info_service_ = this->create_service<sensor_msgs::srv::SetCameraInfo>(
      visp_camera_calibration::set_camera_info_service,
      std::bind(&Camera::setCameraInfoCallback, this, std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3));


    /// \brief Service called when user ends tracker_client node
    rclcpp::Service<visp_tracker::srv::Init_service_viewer>::SharedPtr InitService_;

    /// \brief Service called when user is reconfiguring tracker node
    rclcpp::Service<visp_tracker::srv::reconfigure_service_viewer>::SharedPtr reconfigureService_;

    /// \brief Name of the tracker used in this viewer node
    std::string trackerName_;

    /// \brief Model path.
    std::filesystem::path modelPath_;

    /// \brief ViSP edge tracker.
    vpMbGenericTracker tracker_;
    /// \brief ViSP camera parameters.
    vpCameraParameters cameraParameters_;
    /// \brief ViSP image.
    image_t image_;

    /// \brief Shared pointer to latest received camera information.
    sensor_msgs::msg::CameraInfo::ConstSharedPtr info_;
    /// \brief Last tracked object position, set to none if tracking failed.
    std::optional<vpHomogeneousMatrix> cMo_;
    /// \brief Shared pointer to latest received moving edge sites.
    visp_tracker::msg::MovingEdgeSites::ConstPtr sites_;
    /// \brief Shared pointer to latest received KLT point positions.
    visp_tracker::msg::KltPoints::ConstPtr klt_;

    /// \name Subscribers and synchronizer.
    /// \{
    /// \brief Subscriber to image topic.
    image_transport::SubscriberFilter imageSubscriber_;
    /// \brief Subscriber to camera information topic.
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> cameraInfoSubscriber_;
    /// \brief Subscriber to tracking result topic.
    message_filters::Subscriber<geometry_msgs::msg::PoseWithCovarianceStamped>
    trackingResultSubscriber_;
    /// \brief Subscriber to moving edge sites topics.
    message_filters::Subscriber<visp_tracker::msg::MovingEdgeSites>
    movingEdgeSitesSubscriber_;
    /// \brief Subscriber to KLT point topics.
    message_filters::Subscriber<visp_tracker::msg::KltPoints>
    kltPointsSubscriber_;

    /// \brief Synchronizer with approximate time policy.
    message_filters::Synchronizer<syncPolicy_t> synchronizer_;
    ///}

    /// \name Synchronization check
    /// \{
    rclcpp::WallTimer timer_;
    unsigned countAll_;
    unsigned countImages_;
    unsigned countCameraInfo_;
    unsigned countTrackingResult_;
    unsigned countMovingEdgeSites_;
    unsigned countKltPoints_;
    ///}
  };
} // end of namespace visp_tracker

#endif //! VISP_TRACKER_TRACKER_VIEWER_HH
