#include <cstdlib>
#include <fstream>
#include <sstream>

#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <visp3/gui/vpDisplayX.h>

#include "visp_tracker/callbacks.h"
#include "visp_tracker/conversion.h"
#include "visp_tracker/file.h"
#include "visp_tracker/names.h"

#include "visp_tracker/tracker-viewer.h"

namespace visp_tracker
{
namespace
{
static void increment(unsigned int *value) { ++(*value); }
} // end of anonymous namespace.

// Callback to fix ROS bug when /model_description (not properly cleared) doesn't contain the right model of the object
// to track. Bug only occurs when viewer is started too early and with a different model than the previous call.
bool TrackerViewer::initCallback(visp_tracker::srv::Init::Request &req, visp_tracker::srv::Init::Response &res)
{
  std::ofstream modelStream;
  std::string path;

  if (!makeModelFile(nodeHandle_, modelStream, path))
    throw std::runtime_error("failed to load the model from the callback");
  // RCLCPP_WARN_STREAM(this->get_logger(),"Make model Viewer: " << path.c_str());
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Model loaded from the service.");
  modelPath_ = path;
  tracker_.resetTracker();
  initializeTracker();

  // Common parameters
  convertInitRequestToVpMbTracker(req, tracker_);

  res.initialization_succeed = true;
  return true;
}

bool TrackerViewer::reconfigureCallback(visp_tracker::srv::Init::Request &req, visp_tracker::srv::Init::Response &res)
{
  // Common parameters
  RCLCPP_INFO_STREAM(this->get_logger(), "Reconfiguring Tracker Viewer.");
  convertInitRequestToVpMbTracker(req, tracker_);

  res.initialization_succeed = true;
  return true;
}

TrackerViewer::TrackerViewer(std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<rclcpp::Node> privateNh,
                             volatile bool &exiting, unsigned queueSize)
  : Node("TrackerViewer"), exiting_(exiting), queueSize_(queueSize), nodeHandle_(nh), nodeHandlePrivate_(privateNh),
    imageTransport_(nodeHandle_), frameSize_(0.1), rectifiedImageTopic_(), cameraInfoTopic_(),
    /*checkInputs_(nodeHandle_, get_name()), */ // TODO PORT ROS2
    tracker_(), cameraParameters_(), image_(), info_(), init_viewer_service_(), reconfigure_viewer_service_(),
    trackerName_(), cMo_(std::nullopt), sites_(), imageSubscriber_(), cameraInfoSubscriber_(),
    trackingResultSubscriber_(), movingEdgeSitesSubscriber_(), kltPointsSubscriber_(),
    synchronizer_(syncPolicy_t(queueSize_)), countAll_(0u), countImages_(0u), countCameraInfo_(0u),
    countTrackingResult_(0u), countMovingEdgeSites_(0u), countKltPoints_(0u)
{
  // Compute topic and services names.
  std::string cameraPrefix;

  rclcpp::Rate rate(1);
  while (cameraPrefix.empty()) {
    // Check for the global parameter /camera_prefix set by visp_tracker node
    this->declare_parameter<std::string>("~camera_prefix", this->get_parameter("~camera_prefix"));
    rclcpp::Parameter cameraPrefix_param = this->get_parameter("~camera_prefix");
    cameraPrefix = cameraPrefix_param.as_string();
// TODO PORT ROS2
    if (cameraPrefix.empty()) {
      RCLCPP_WARN(this->get_logger(), "the camera_prefix parameter does not exist.\n"
                                      "This may mean that:\n"
                                      "- the tracker is not launched,\n"
                                      "- the tracker and viewer are not running in the same namespace.");
    } else if (cameraPrefix.empty()) {
      RCLCPP_INFO(this->get_logger(), "tracker is not yet initialized, waiting...\n"
                                      "You may want to launch the client to initialize the tracker.");
    }
    if (this->exiting())
      return;
    rate.sleep();
  }
  if (frameSize_ != NULL) {
    nodeHandlePrivate_->declare_parameter<double>("frame_size", frameSize_);
    ;
  } else {
    nodeHandlePrivate_->declare_parameter<double>("frame_size", 0.1);
  }

  rectifiedImageTopic_ = this->get_node_base_interface()->resolve_topic_name(cameraPrefix + "/image_rect");
  cameraInfoTopic_ = this->get_node_base_interface()->resolve_topic_name(cameraPrefix + "/camera_info");

  typedef boost::function<bool(visp_tracker::srv::Init::Request &, visp_tracker::srv::Init::Response & res)>
      initCallback_t initCallback = boost::bind(&TrackerViewer::initCallback, this, std::placeholders::_1, std::placeholders::_2);

  typedef boost::function<bool(visp_tracker::srv::Init::Request &, visp_tracker::srv::Init::Response & res)>
      reconfigureCallback_t reconfigureCallback = boost::bind(&TrackerViewer::reconfigureCallback, this, std::placeholders::_1, std::placeholders::_2);

  // define services
  init_viewer_service_ = this->create_service<visp_tracker::srv::Init_viewer_service>(
      visp_tracker::Init_viewer_service_, std::bind(&TrackerViewer::initCallback, this, std::placeholders::_1,
                                                    std::placeholders::_2, std::placeholders::_3));

  //    init_viewer_service_ = nodeHandle_.advertiseService
  //        (visp_tracker::srv::Init_service_viewer, initCallback);

  // define services
  reconfigure_viewer_service_ = this->create_service<visp_tracker::srv::Reconfigure_viewer_service>(
      visp_tracker::calibrate_service, std::bind(&TrackerViewer::reconfigureCallback, this, std::placeholders::_1,
                                                 std::placeholders::_2, std::placeholders::_3));
  //     calibrate_service_ = n_.advertiseService(visp_camera_calibration::calibrate_service,calibrate_callback);

  //    reconfigure_viewer_service_ = nodeHandle_.advertiseService
  //        (visp_tracker::reconfigure_viewer_service, reconfigureCallback);

  std::ofstream modelStream;
  std::string path;

  unsigned int cpt = 0;
  while (!nodeHandle_->t(visp_tracker::model_description_param)) {
    if (!nodeHandle_->get_parameter(visp_tracker::model_description_param)) {
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

  if (!makeModelFile(modelStream, path))
    throw std::runtime_error("failed to load the model from the parameter server");

  RCLCPP_INFO_STREAM(this->get_logger(), "Model loaded from the parameter server.");
  // RCLCPP_WARN_STREAM(this->get_logger(),"Make model Viewer: " << path.c_str());
  modelPath_ = path;

  initializeTracker();

  if (this->exiting())
    return;

  checkInputs();
  if (this->exiting())
    return;

  // Subscribe to camera and tracker synchronously.
  // TODO PORT ROS2
  imageSubscriber_.subscribe(imageTransport_, rectifiedImageTopic_, queueSize_);
  cameraInfoSubscriber_.subscribe(nodeHandle_, cameraInfoTopic_, queueSize_);
  trackingResultSubscriber_.subscribe(nodeHandle_, visp_tracker::object_position_covariance_topic, queueSize_);
  movingEdgeSitesSubscriber_.subscribe(nodeHandle_, visp_tracker::moving_edge_sites_topic, queueSize_);
  kltPointsSubscriber_.subscribe(nodeHandle_, visp_tracker::klt_points_topic, queueSize_);

  synchronizer_.connectInput(imageSubscriber_, cameraInfoSubscriber_, trackingResultSubscriber_,
                             movingEdgeSitesSubscriber_, kltPointsSubscriber_);
  synchronizer_.registerCallback(std::bind(&TrackerViewer::callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_3, std::placeholders::_5));

  // Check for synchronization every 30s.
  synchronizer_.registerCallback(std::bind(increment, &countAll_));
  imageSubscriber_.registerCallback(std::bind(increment, &countImages_));
  cameraInfoSubscriber_.registerCallback(std::bind(increment, &countCameraInfo_));
  trackingResultSubscriber_.registerCallback(std::bind(increment, &countTrackingResult_));
  movingEdgeSitesSubscriber_.registerCallback(std::bind(increment, &countMovingEdgeSites_));
  kltPointsSubscriber_.registerCallback(std::bind(increment, &countKltPoints_));
  auto ros_clock = rclcpp::Clock::make_shared();
  timer_ = rclcpp::create_timer(this, ros_clock, rclcpp::Duration(30, 0), std::bind(&TrackerViewer::timerCallback));

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
    std::string fmtWindowTitle ("ViSP MBT tracker viewer - [ns: ")+rclcpp::getNamespace ()+"]");

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
          RCLCPP_DEBUG_STREAM_THROTTLE(this->get_logger(),clock, LOG_THROTTLE_PERIOD, "failed to display cmo");
        }

        RCLCPP_DEBUG_STREAM_THROTTLE(this->get_logger(),clock, LOG_THROTTLE_PERIOD, "cMo viewer:\n" << *cMo_);
        fmt = std::string("tracking (x=") + std::to_string((*cMo_)[0][3]) + " y=" + std::to_string((*cMo_)[1][3]) + " z=" + std::to_string((*cMo_)[2][3]) + ")";
        vpDisplay::displayCharString
            (image_, point, fmt.c_str(), vpColor::red);
        fmtTime = std::string("time = "+info_->header.stamp.sec);
        vpDisplay::displayCharString
            (image_, pointTime, fmtTime.c_str(), vpColor::red);
        vpDisplay::displayCharString
            (image_, pointCameraTopic, fmtCameraTopic.c_str(),
             vpColor::red);
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
    RCLCPP_INFO_THROTTLE(rclcpp::get_logger("rclcpp"),clock,LOG_THROTTLE_PERIOD , "waiting for a rectified image...");
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
    nodeHandlePrivate_->declare_parameter <std::string> ("tracker_name", trackerName_);
  } else {
    nodeHandlePrivate_->declare_parameter <std::string> ("tracker_name", "");
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
    tracker_.loadModel(modelPath_.native().c_str());
  } catch (...) {
    std::string fmt("failed to load the model ");
    throw std::runtime_error(fmt+std::string(modelPath_));
  }
  // RCLCPP_WARN(this->get_logger(),"Model has been successfully loaded.");
}

void TrackerViewer::callback(const sensor_msgs::msg::Image::ConstSharedPtr &image,
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
