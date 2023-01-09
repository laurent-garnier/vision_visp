#include <cstdlib>
#include <fstream>
#include <sstream>

#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visp3/gui/vpDisplayX.h>

#include "visp_tracker/callbacks.h"
#include "visp_tracker/conversion.h"
#include "visp_tracker/file.h"
#include "visp_tracker/names.h"
#include <message_filters/time_synchronizer.h>

#include "visp_tracker/tracker-viewer.h"

namespace visp_tracker
{
namespace
{
static void increment(unsigned int *value) { ++(*value); }
} // end of anonymous namespace.

// Callback to fix ROS bug when /model_description (not properly cleared) doesn't contain the right model of the object
// to track. Bug only occurs when viewer is started too early and with a different model than the previous call.
bool TrackerViewer::initCallback(const std::shared_ptr<rmw_request_id_t> /*request_header*/,
                                 const std::shared_ptr<visp_tracker::srv::Init::Request> req,
                                 std::shared_ptr<visp_tracker::srv::Init::Response> res)
{
  std::ofstream modelStream;
  std::string path;

  if (!makeModelFile(this->get_node_parameters_interface(), req->model_description_param, modelStream, path))
    throw std::runtime_error("failed to load the model from the callback");
  // RCLCPP_WARN_STREAM(this->get_logger(),"Make model Viewer: " << path.c_str());
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Model loaded from the service.");
  modelPath_ = path;
  tracker_.resetTracker();
  initializeTracker();

  // Common parameters
  convertInitRequestToVpMbTracker(req, tracker_);
  visp_tracker::model_description_param = req->model_description_param;

  res->initialization_succeed = true;
  return true;
}

bool TrackerViewer::reconfigureCallback(const std::shared_ptr<rmw_request_id_t> /*request_header*/,
                                        const std::shared_ptr<visp_tracker::srv::Init::Request> req,
                                        std::shared_ptr<visp_tracker::srv::Init::Response> res)
{
  // Common parameters
  RCLCPP_INFO_STREAM(this->get_logger(), "Reconfiguring Tracker Viewer.");
  convertInitRequestToVpMbTracker(req, tracker_);

  res->initialization_succeed = true;
  return true;
}

TrackerViewer::TrackerViewer() : Node("TrackerViewer"), 
    /*imageTransport_(nodeHandle_), */frameSize_(0.1), rectifiedImageTopic_(), cameraInfoTopic_(),
    /*checkInputs_(nodeHandle_, get_name()), */ // TODO PORT ROS2
    trackerName_(), tracker_(), cameraParameters_(), image_(),
    info_(), cMo_(std::nullopt), sites_(), imageSubscriber_(), cameraInfoSubscriber_(), trackingResultSubscriber_(),
    movingEdgeSitesSubscriber_(), kltPointsSubscriber_(), synchronizer_(syncPolicy_t(5u)), countAll_(0u),
    countImages_(0u), countCameraInfo_(0u), countTrackingResult_(0u), countMovingEdgeSites_(0u), countKltPoints_(0u)
{
  // Compute topic and services names.
  std::string cameraPrefix = this->declare_parameter<std::string>("camera_prefix", "/wide_left/camera");

  rclcpp::Rate rate(1);
  while (cameraPrefix.empty()) {
    // Check for the global parameter /camera_prefix set by visp_tracker node
    this->get_parameter("camera_prefix", cameraPrefix);
    // TODO PORT ROS2 ??
    if (cameraPrefix.empty()) {
      this->get_parameter("~camera_prefix", cameraPrefix);
      if (cameraPrefix.empty()) {

        RCLCPP_WARN(this->get_logger(), "the camera_prefix parameter does not exist.\n"
                                        "This may mean that:\n"
                                        "- the tracker is not launched,\n"
                                        "- the tracker and viewer are not running in the same namespace.");
      }
    } else if (cameraPrefix.empty()) {
      RCLCPP_INFO(this->get_logger(), "tracker is not yet initialized, waiting...\n"
                                      "You may want to launch the client to initialize the tracker.");
    }
    if (this->exiting())
      return;
    rate.sleep();
  }
  if (frameSize_ != 0.1) {
    this->declare_parameter<double>("frame_size", frameSize_);
    ;
  } else {
    this->declare_parameter<double>("frame_size", 0.1);
  }
  // ROS2 FIXME
  // rectifiedImageTopic_ = this->get_node_base_interface()->resolve_topic_name(cameraPrefix + "/image_rect");
  // cameraInfoTopic_ = this->get_node_base_interface()->resolve_topic_name(cameraPrefix + "/camera_info");
  rectifiedImageTopic_ = cameraPrefix + "/image_rect";
  cameraInfoTopic_ = cameraPrefix + "/camera_info";

  //  initCallback_t initCallback = std::bind(&TrackerViewer::initCallback, this, std::placeholders::_1,
  //        std::placeholders::_2, std::placeholders::_3);

  //  typedef boost::function<bool(visp_tracker::srv::Init::Request &, visp_tracker::srv::Init::Response & res)>
  //  reconfigureCallback_t reconfigureCallback = boost::bind(&TrackerViewer::reconfigureCallback, this,
  //       std::placeholders::_1, std::placeholders::_2);

  // define services
  /*   calibrate_service_ = this->create_service<visp_camera_calibration::srv::Calibrate>(
        visp_camera_calibration::calibrate_service, std::bind(&Calibrator::calibrateCallback, this,
     std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  */
  init_viewer_service_ = this->create_service<visp_tracker::srv::Init>(
      visp_tracker::init_viewer_service, std::bind(&TrackerViewer::initCallback, this, std::placeholders::_1,
                                                   std::placeholders::_2, std::placeholders::_3));

  //    init_viewer_service_ = nodeHandle_.advertiseService
  //        (visp_tracker::srv::Init_service_viewer, initCallback);

  // define services
  reconfigure_viewer_service_ = this->create_service<visp_tracker::srv::Init>(
      visp_tracker::reconfigure_viewer_service,
      std::bind(&TrackerViewer::reconfigureCallback, this, std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3));
  //                std::placeholders::_3));

  //    reconfigure_viewer_service_ = nodeHandle_.advertiseService
  //        (visp_tracker::reconfigure_viewer_service, reconfigureCallback);

  std::ofstream modelStream;
  std::string path;
  std::string model_description_txt = this->declare_parameter<std::string>(visp_tracker::model_description_param,"");
/*
  std::string model_description_txt = this->declare_parameter<std::string>(visp_tracker::model_description_param,"");
  unsigned int cpt = 0;
  while (model_description_txt == "") {
    if (cpt % 10 == 0) {
      RCLCPP_WARN_STREAM(this->get_logger(),
                          "[Node: " << std::string(get_name())
                                   << "]\n"
                                      "The model_description parameter does not exist.\n"
                                      "This may mean that:\n"
                                      "- the tracker is not launched or not initialized,\n"
                                      "- the tracker and viewer are not running in the same namespace.");
    }
    cpt++;
    }
    if (this->exiting())
      return;
    rate.sleep();
  }

  if (!makeModelFile(this->get_node_parameters_interface(), visp_tracker::model_description_param, modelStream, path))
    throw std::runtime_error("failed to load the model from the parameter server");
*/
  RCLCPP_INFO_STREAM(this->get_logger(), "Model loaded from the parameter server.");
  // RCLCPP_WARN_STREAM(this->get_logger(),"Make model Viewer: " << path.c_str());

/* remove; already in intCallback
  modelPath_ = path;

  initializeTracker();
*/
  if (this->exiting())
    return;

  checkInputs();
  if (this->exiting())
    return;

  // Subscribe to camera and tracker synchronously.
  // TODO PORT ROS2
  /*    point_correspondence_subscriber_callback_t point_correspondence_callback =
    boost::bind(&Calibrator::pointCorrespondenceCallback, this, _1); calibrate_service_callback_t calibrate_callback =
    boost::bind(&Calibrator::calibrateCallback, this, _1, _2);

    point_correspondence_subscriber_ = n_.subscribe(visp_camera_calibration::point_correspondence_topic, queue_size_,
                                                      point_correspondence_callback);
    point_correspondence_subscriber_ = this->create_subscription<visp_camera_calibration::msg::CalibPointArray>(
        visp_camera_calibration::point_correspondence_topic, queue_size_,
        std::bind(&Calibrator::pointCorrespondenceCallback, this, std::placeholders::_1));
  */
  // ROS2 : FROM
  // https://github.com/ruffsl/Kimera-VIO-ROS/blob/2ab53a796cf91f45df1060de3aeaaaf460aa0028/src/KimeraVioNode.cpp#L42-L43

  // void image_transport::SubscriberFilter::subscribe(
  // rclcpp::Node*,
  // const string&,
  // const string&,
  // rmw_qos_profile_t,
  // rclcpp::SubscriptionOptions)’

  imageSubscriber_.subscribe(this, rectifiedImageTopic_, "raw"); // ROS2 : FIXME raw ?

  cameraInfoSubscriber_.subscribe(this, cameraInfoTopic_);
  trackingResultSubscriber_.subscribe(this, visp_tracker::object_position_covariance_topic);
  movingEdgeSitesSubscriber_.subscribe(this, visp_tracker::moving_edge_sites_topic);
  kltPointsSubscriber_.subscribe(this, visp_tracker::klt_points_topic);

  // Test old/ new version
  if (true) {
// FIXME ROS2 10-2s
    message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, geometry_msgs::msg::PoseWithCovarianceStamped, visp_tracker::msg::MovingEdgeSites, visp_tracker::msg::KltPoints>
     synchronize_(imageSubscriber_, cameraInfoSubscriber_, trackingResultSubscriber_,  movingEdgeSitesSubscriber_, kltPointsSubscriber_, 2);
//    typedef sync_policies::ApproximateTime<learn_msg_filter::NewString, learn_msg_filter::NewString> MySyncPolicy;

//    message_filters::Synchronizer<MySyncPolicy> synchronize_(MySyncPolicy(10), imageSubscriber_, cameraInfoSubscriber_, trackingResultSubscriber_,
//                             movingEdgeSitesSubscriber_, kltPointsSubscriber_);
    synchronize_.registerCallback(std::bind(&TrackerViewer::viewerCallback, this, std::placeholders::_1, std::placeholders::_2,
                                           std::placeholders::_3, std::placeholders::_4, std::placeholders::_5));                 
//    synchronize_.registerCallback(std::bind(increment, &countAll_));

  } else {
    synchronizer_.connectInput(imageSubscriber_, cameraInfoSubscriber_, trackingResultSubscriber_,
                             movingEdgeSitesSubscriber_, kltPointsSubscriber_);

    // TODO ROS2 : Does not work as expected
    synchronizer_.registerCallback(std::bind(&TrackerViewer::viewerCallback, this, std::placeholders::_1, std::placeholders::_2,
                                           std::placeholders::_3, std::placeholders::_4, std::placeholders::_5));

    // Check for synchronization every 30s.
    synchronizer_.registerCallback(std::bind(increment, &countAll_));
  }
  imageSubscriber_.registerCallback(std::bind(increment, &countImages_));
  cameraInfoSubscriber_.registerCallback(std::bind(increment, &countCameraInfo_));
  trackingResultSubscriber_.registerCallback(std::bind(increment, &countTrackingResult_));
  movingEdgeSitesSubscriber_.registerCallback(std::bind(increment, &countMovingEdgeSites_));
  kltPointsSubscriber_.registerCallback(std::bind(increment, &countKltPoints_));

// FIXME ROS2 30-5s
  timer_ = create_wall_timer(std::chrono::seconds(5),std::bind(&TrackerViewer::timerCallback, this));

  // Wait for image.
  waitForImage();
  if (this->exiting())
    return;
  if (!image_.getWidth() || !image_.getHeight())
    throw std::runtime_error("failed to retrieve image");

  // Load camera parameters.
  initializeVpCameraFromCameraInfo(cameraParameters_, info_);
  tracker_.setCameraParameters(cameraParameters_);

  // Load the common parameters from ROS messages
  loadCommonParameters();
}

void TrackerViewer::spin()
{
  std::string fmtWindowTitle("ViSP MBT tracker viewer");

  vpDisplayX d(image_, image_.getWidth(), image_.getHeight(), fmtWindowTitle.c_str());
  vpImagePoint point(10, 10);
  vpImagePoint pointTime(22, 10);
  vpImagePoint pointCameraTopic(34, 10);
  rclcpp::Rate loop_rate(80);

  std::string fmtTime;
  std::string fmt;

  std::string fmtCameraTopic = std::string("camera topic = ") + rectifiedImageTopic_;
  rclcpp::Clock clock;
  constexpr size_t LOG_THROTTLE_PERIOD = 10;
  while (!exiting()) {
    vpDisplay::display(image_);
    displayMovingEdgeSites();
    displayKltPoints();
    if (cMo_) {
      try {
        tracker_.initFromPose(image_, *cMo_);
        tracker_.display(image_, *cMo_, cameraParameters_, vpColor::red);
        vpDisplay::displayFrame(image_, *cMo_, cameraParameters_, frameSize_, vpColor::none, 2);
      } catch (...) {
        RCLCPP_DEBUG_STREAM_THROTTLE(this->get_logger(), clock, LOG_THROTTLE_PERIOD, "Failed to display cMo");
      }

      RCLCPP_DEBUG_STREAM_THROTTLE(this->get_logger(), clock, LOG_THROTTLE_PERIOD, "cMo viewer:\n" << *cMo_);
      fmt = std::string("tracking (x=") + std::to_string((*cMo_)[0][3]) + " y=" + std::to_string((*cMo_)[1][3]) +
            " z=" + std::to_string((*cMo_)[2][3]) + ")";
      vpDisplay::displayCharString(image_, point, fmt.c_str(), vpColor::red);
      fmtTime = std::string("time = " + info_->header.stamp.sec);
      vpDisplay::displayCharString(image_, pointTime, fmtTime.c_str(), vpColor::red);
      vpDisplay::displayCharString(image_, pointCameraTopic, fmtCameraTopic.c_str(), vpColor::red);
    } else {
      vpDisplay::displayCharString(image_, point, "tracking failed", vpColor::red);
    }

    vpDisplay::flush(image_);
    rclcpp::spin_some(this->get_node_base_interface());
    loop_rate.sleep();
  }
}

void TrackerViewer::waitForImage()
{
  rclcpp::Rate loop_rate(10);
  rclcpp::Clock clock;
  constexpr size_t LOG_THROTTLE_PERIOD = 10; 
  while (!exiting() && (!image_.getWidth() || !image_.getHeight())) {
    RCLCPP_INFO_THROTTLE(rclcpp::get_logger("rclcpp"), clock, LOG_THROTTLE_PERIOD, "waiting for a rectified image...");
    rclcpp::spin_some(this->get_node_base_interface());
    loop_rate.sleep();
  }
}

void TrackerViewer::checkInputs()
{
  // TODO PORT ROS2
  // ros::V_string topics;
  // topics.push_back(rectifiedImageTopic_);
  // topics.push_back(cameraInfoTopic_);
  // topics.push_back(visp_tracker::object_position_topic);
  // topics.push_back(visp_tracker::moving_edge_sites_topic);
  // topics.push_back(visp_tracker::klt_points_topic);
  // checkInputs_.start(topics, 60.0);
}

void TrackerViewer::loadCommonParameters()
{
  if (trackerName_ != "") {
    this->declare_parameter<std::string>("tracker_name", trackerName_);
  } else {
    this->declare_parameter<std::string>("tracker_name", "");
  }

  bool loadParam = false;

  if (trackerName_.empty()) {
    rclcpp::Parameter param;
    if (!this->get_parameter("/angle_appear", param)) {
      trackerName_ = "tracker_mbt";
      if (!this->get_parameter(trackerName_ + "/angle_appear", param)) {
        RCLCPP_WARN_STREAM(
            this->get_logger(),
            "No tracker has been found with the default name value \""
                << trackerName_ << "/angle_appear\".\n"
                << "Tracker name parameter (tracker_name) should be provided for this node (tracker_viewer).\n"
                << "Polygon visibility might not work well in the viewer window.");
      } else
        loadParam = true;
    } else
      loadParam = true;
  } else
    loadParam = true;

  // Reading common parameters
  if (loadParam) {
    rclcpp::Parameter double_param;
    if (this->get_parameter(trackerName_ + "/angle_appear", double_param)) {
      double value = double_param.as_double();

      RCLCPP_WARN_STREAM(this->get_logger(), "Angle appear viewer: " << value);
      tracker_.setAngleAppear(vpMath::rad(value));

    } else {
      RCLCPP_WARN_STREAM(this->get_logger(), "No tracker has been found with the provided parameter "
                                                 << "(tracker_name=\"" << trackerName_ << "\")\n"
                                                 << "Polygon visibility might not work well in the viewer window");
    }

    if (this->get_parameter(trackerName_ + "/angle_disappear", double_param)) {
      double value = double_param.as_double();

      RCLCPP_WARN_STREAM(this->get_logger(), "Angle disappear viewer: " << value);
      tracker_.setAngleDisappear(vpMath::rad(value));
    }
  }
}

void TrackerViewer::initializeTracker()
{
  try {
    // RCLCPP_WARN_STREAM(this->get_logger(),"Trying to load the model Viewer: " << modelPath_);
    tracker_.loadModel(modelPath_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Model " << modelPath_ << " has been successfully loaded.");

  } catch (...) {
    std::stringstream fmt;
    fmt << "Failed to load the model " << modelPath_;
    throw std::runtime_error(fmt.str());
  }
}

void TrackerViewer::viewerCallback(const sensor_msgs::msg::Image::ConstSharedPtr &image,
                             const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info,
                             const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr &trackingResult,
                             const visp_tracker::msg::MovingEdgeSites::ConstSharedPtr &sites,
                             const visp_tracker::msg::KltPoints::ConstSharedPtr &klt)
{
  // Copy image.
  try {
    rosImageToVisp(image_, image);
  } catch (std::exception &e) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "dropping frame: " << e.what());
  }

  // Copy moving camera infos, edges sites and optional KLT points.
  info_ = info;
  sites_ = sites;
  klt_ = klt;

  // Copy cMo.
  cMo_ = vpHomogeneousMatrix();
  transformToVpHomogeneousMatrix(*cMo_, trackingResult->pose.pose);
}

void TrackerViewer::displayMovingEdgeSites()
{
  if (!sites_)
    return;
  for (unsigned i = 0; i < sites_->moving_edge_sites.size(); ++i) {
    double x = sites_->moving_edge_sites[i].x;
    double y = sites_->moving_edge_sites[i].y;
    int suppress = sites_->moving_edge_sites[i].suppress;
    vpColor color = vpColor::black;

    switch (suppress) {
    case vpMeSite::NO_SUPPRESSION:
      color = vpColor::green;
      break;
    case vpMeSite::CONSTRAST:
      color = vpColor::blue;
      break;
    case vpMeSite::THRESHOLD:
      color = vpColor::purple;
      break;
    case vpMeSite::M_ESTIMATOR:
      color = vpColor::red;
      break;
    default: // vpMeSite::UNKOWN
      color = vpColor::yellow;
    }

    vpDisplay::displayCross(image_, vpImagePoint(x, y), 3, color, 1);
  }
}

void TrackerViewer::displayKltPoints()
{
  if (!klt_)
    return;
  vpImagePoint pos;

  for (unsigned i = 0; i < klt_->klt_points_positions.size(); ++i) {
    double ii = klt_->klt_points_positions[i].i;
    double jj = klt_->klt_points_positions[i].j;
    int id = klt_->klt_points_positions[i].id;
    vpColor color = vpColor::red;

    vpDisplay::displayCross(image_, vpImagePoint(ii, jj), 15, color, 1);

    pos.set_i(vpMath::round(ii + 7));
    pos.set_j(vpMath::round(jj + 7));
    char ide[10];
    sprintf(ide, "%d", id);
    vpDisplay::displayCharString(image_, pos, ide, vpColor::red);
  }
}

void TrackerViewer::timerCallback()
{
  if (countTrackingResult_ != countMovingEdgeSites_ || countKltPoints_ != countMovingEdgeSites_) {
    std::string fmt =
        std::string("[visp_tracker] Low number of synchronized tuples received.\n") +
        "Images: " + std::to_string(countImages_) + "\n" + "Camera info: " + std::to_string(countCameraInfo_) + "\n" +
        "Tracking result: " + std::to_string(countTrackingResult_) + "\n" +
        "Moving edge sites: " + std::to_string(countMovingEdgeSites_) + "\n" +
        "KLT points: " + std::to_string(countKltPoints_) + "\n" + "Synchronized tuples: " + std::to_string(countAll_) +
        "\n" + "Possible issues:\n" + "\t* The network is too slow.";
    rclcpp::Clock clock;
    constexpr size_t LOG_THROTTLE_PERIOD = 10;
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), clock, LOG_THROTTLE_PERIOD, fmt);
  }
}
} // end of namespace visp_tracker.
