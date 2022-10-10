#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
//#include <rclcpp/package.h>
//#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.h>
#include <visp_tracker/srv/init.hpp>
//#include <visp_tracker/msg/model_based_settings_config.hpp>

#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/me/vpMe.h>
#include <visp3/vision/vpPose.h>

#include <visp3/mbt/vpMbGenericTracker.h>

#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>

#include "visp_tracker/callbacks.h"
#include "visp_tracker/conversion.h"
#include "visp_tracker/file.h"
#include "visp_tracker/names.h"

#include "visp_tracker/tracker-client.h"

namespace visp_tracker
{
TrackerClient::TrackerClient(std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<rclcpp::Node> privateNh,
                             volatile bool &exiting, unsigned queueSize)
  : Node("TrackerClient"), exiting_(exiting), queueSize_(queueSize), nodeHandle_(nh), nodeHandlePrivate_(privateNh),
    imageTransport_(nodeHandle_), image_(), modelPath_(), modelPathAndExt_(), modelName_(), cameraPrefix_(),
    rectifiedImageTopic_(), cameraInfoTopic_(), trackerType_("mbt"), frameSize_(0.1), bModelPath_(), bInitPath_(),
    cameraSubscriber_(), mutex_(),
    /*FIXME: RECONFIGURATION
          reconfigureSrv_(NULL),
          reconfigureKltSrv_(NULL),
          reconfigureEdgeSrv_(NULL),
    */
    movingEdge_(), kltTracker_(), cameraParameters_(), tracker_(), startFromSavedPose_(),
    // checkInputs_(),// TODO PORT ROS2
    resourceRetriever_()
{

  // checks if param_name is exist the command pass the value of param in variable but if the param
  // doesn't exist then the command pass "default" to variable

  if (modelPath_ != "") {
    nodeHandlePrivate_->declare_parameter<std::string>("model_path", modelPath_);
  } else {
    nodeHandlePrivate_->declare_parameter<std::string>("model_path", visp_tracker::default_model_path);
  }
  if (modelName_ != "") {
    nodeHandlePrivate_->declare_parameter<std::string>("model_name", modelName_);
  } else {
    nodeHandlePrivate_->declare_parameter<std::string>("model_name", "");
  }
  if (startFromSavedPose_ == true) {
    nodeHandlePrivate_->declare_parameter<bool>("start_from_saved_pose", startFromSavedPose_);
  } else {
    nodeHandlePrivate_->declare_parameter<bool>("start_from_saved_pose", false);
  }
  if (confirmInit_ == false) {
    nodeHandlePrivate_->declare_parameter<bool>("confirm_init", confirmInit_);
  } else {
    nodeHandlePrivate_->declare_parameter<bool>("confirm_init", true);
  }

  if (trackerType_ != "") {
    nodeHandlePrivate_->declare_parameter<std::string>("tracker_type", trackerType_);
  } else {
    nodeHandlePrivate_->declare_parameter<std::string>("tracker_type", "mtb");
  }
  if (trackerType_ == "mbt")
    tracker_.setTrackerType(vpMbGenericTracker::EDGE_TRACKER);
  else if (trackerType_ == "klt")
    tracker_.setTrackerType(vpMbGenericTracker::KLT_TRACKER);
  else
    tracker_.setTrackerType(vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER);

  if (frameSize_ != 0.1) {
    nodeHandlePrivate_->declare_parameter<double>("frame_size", frameSize_);
  } else {
    nodeHandlePrivate_->declare_parameter<double>("frame_size", 0.1);
  }

  if (modelName_.empty())
    throw std::runtime_error("empty model\n"
                             "Relaunch the client while setting the model_name parameter, i.e.\n"
                             "$ rosrun visp_tracker client _model_name:=my-model");

  // Compute topic and services names.

  rclcpp::Rate rate(1);
  while (cameraPrefix_.empty()) {
    if (!nodeHandle_->get_parameter("camera_prefix", cameraPrefix_) && !this->get_parameter("~camera_prefix", cameraPrefix_)) {
      RCLCPP_WARN(this->get_logger(), "the camera_prefix parameter does not exist.\n"
                                      "This may mean that:\n"
                                      "- the tracker is not launched,\n"
                                      "- the tracker and viewer are not running in the same namespace.");
    } else if (cameraPrefix_.empty()) {
      RCLCPP_INFO(this->get_logger(), "tracker is not yet initialized, waiting...\n"
                                      "You may want to launch the client to initialize the tracker.");
    }
    if (this->exiting())
      return;
    rate.sleep();
  }
// ros1   rectifiedImageTopic_ =        ros::names::resolve(cameraPrefix_ + "/image_rect");
// ros1   cameraInfoTopic_ =        ros::names::resolve(cameraPrefix_ + "/camera_info");
// ros::names::resolve - Resolve a graph resource name into a fully qualified graph resource name.

//  auto node_topics_interface = rclcpp::node_interfaces::get_node_topics_interface();
//  node_topics_interface->resolve_topic_name("topic_name");
    
  rectifiedImageTopic_ = this->get_node_base_interface()->resolve_topic_name(cameraPrefix_ + "/image_rect");
  cameraInfoTopic_ = this->get_node_base_interface()->resolve_topic_name(cameraPrefix_ + "/camera_info");

  // Check for subscribed topics.
  checkInputs();

  cameraSubscriber_ = imageTransport_.subscribeCamera(
    
      rectifiedImageTopic_,
      queueSize_,
      std::bind(imageCallback, 
                std::ref(image_), 
                std::ref(header_), 
                std::ref(info_), 
                std::placeholders::_1, 
                std::placeholders::_2)
      );

  // Model loading.
  bModelPath_ = getModelFileFromModelName(modelName_, modelPath_);
  bInitPath_ = getInitFileFromModelName(modelName_, modelPath_);

  RCLCPP_INFO_STREAM(this->get_logger(), "Model file: " << bModelPath_);
  RCLCPP_INFO_STREAM(this->get_logger(), "Init file: " << bInitPath_);

  // Load the 3d model.
  loadModel();

  /* FIXME: RECONFIGURATION
     // Dynamic reconfigure.
      if(trackerType_=="mbt+klt"){ // Hybrid Tracker reconfigure
        reconfigureSrv_ = new reconfigureSrvStruct<visp_tracker::ModelBasedSettingsConfig>::reconfigureSrv_t(mutex_,
     nodeHandlePrivate_); reconfigureSrvStruct<visp_tracker::ModelBasedSettingsConfig>::reconfigureSrv_t::CallbackType f
     = boost::bind(&reconfigureCallback, std::ref(tracker_), std::ref(image_), std::ref(movingEdge_),
     std::ref(kltTracker_), std::ref(mutex_), std::placeholders::_1, std::placeholders::_2); reconfigureSrv_->setCallback(f);
      }
      else if(trackerType_=="mbt"){ // Edge Tracker reconfigure
        reconfigureEdgeSrv_ = new
     reconfigureSrvStruct<visp_tracker::ModelBasedSettingsEdgeConfig>::reconfigureSrv_t(mutex_, nodeHandlePrivate_);
        reconfigureSrvStruct<visp_tracker::ModelBasedSettingsEdgeConfig>::reconfigureSrv_t::CallbackType f =
            boost::bind(&reconfigureEdgeCallback, std::ref(tracker_),
                        std::ref(image_), std::ref(movingEdge_),
                        std::ref(mutex_), std::placeholders::_1, std::placeholders::_2);
        reconfigureEdgeSrv_->setCallback(f);
      }
      else{ // KLT Tracker reconfigure
        reconfigureKltSrv_ = new
     reconfigureSrvStruct<visp_tracker::ModelBasedSettingsKltConfig>::reconfigureSrv_t(mutex_, nodeHandlePrivate_);
        reconfigureSrvStruct<visp_tracker::ModelBasedSettingsKltConfig>::reconfigureSrv_t::CallbackType f =
            boost::bind(&reconfigureKltCallback, std::ref(tracker_),
                        std::ref(image_), std::ref(kltTracker_),
                        std::ref(mutex_), std::placeholders::_1, std::placeholders::_2);
        reconfigureKltSrv_->setCallback(f);
      }
  */
  // Wait for the image to be initialized.
  waitForImage();
  if (this->exiting())
    return;
  if (!image_.getWidth() || !image_.getHeight())
    throw std::runtime_error("failed to retrieve image");

  // Tracker initialization.
  // - Camera
  initializeVpCameraFromCameraInfo(cameraParameters_, info_);
  tracker_.setCameraParameters(cameraParameters_);
  tracker_.setDisplayFeatures(true);

  RCLCPP_INFO_STREAM(this->get_logger(), convertVpMbTrackerToRosMessage(tracker_));
  // - Moving edges.
  if (trackerType_ != "klt") {
    RCLCPP_INFO_STREAM(this->get_logger(), convertVpMeToRosMessage(tracker_, movingEdge_));
  }

  if (trackerType_ != "mbt") {
    RCLCPP_INFO_STREAM(this->get_logger(), convertVpKltOpencvToRosMessage(tracker_, kltTracker_));
  }

  // Display camera parameters and moving edges settings.
  RCLCPP_INFO_STREAM(this->get_logger(), cameraParameters_);
}

void TrackerClient::checkInputs()
{
  // TODO PORT ROS2
  // ros::V_string topics;
  // topics.push_back(rectifiedImageTopic_);
  // topics.push_back(cameraInfoTopic_);
  // checkInputs_.start(topics, 60.0);
  // get_topic_names_and_types
  //    pub_mono_  = image_transport::create_publisher(this, image_mono);
}

void TrackerClient::spin()
{
    std::string fmtWindowTitle = std::string(("ViSP MBT tracker initialization - [ns: ")+rclcpp::getNamespace ()+"]";

    vpDisplayX d(image_, image_.getWidth(), image_.getHeight(),
                 fmtWindowTitle.c_str());

    rclcpp::Rate loop_rate_tracking(200);
    bool ok = false;
    vpHomogeneousMatrix cMo;
    vpImagePoint point (10, 10);
    while (!ok && !exiting())
    {
    try {
      // Initialize.
      vpDisplay::display(image_);
      vpDisplay::flush(image_);
      if (!startFromSavedPose_)
        init();
      else {
        cMo = loadInitialPose();
        startFromSavedPose_ = false;
        tracker_.initFromPose(image_, cMo);
      }
      tracker_.getPose(cMo);

      RCLCPP_INFO_STREAM(this->get_logger(), "initial pose [tx,ty,tz,tux,tuy,tuz]:\n" << vpPoseVector(cMo).t());

      // Track once to make sure initialization is correct.
      if (confirmInit_) {
        vpImagePoint ip;
        vpMouseButton::vpMouseButtonType button = vpMouseButton::button1;
        do {
          vpDisplay::display(image_);
          mutex_.lock();
          tracker_.track(image_);
          tracker_.getPose(cMo);
          tracker_.display(image_, cMo, cameraParameters_, vpColor::red, 2);
          vpDisplay::displayFrame(image_, cMo, cameraParameters_, frameSize_, vpColor::none, 2);
          mutex_.unlock();
          vpDisplay::displayCharString(image_, point, "tracking, click to initialize tracker", vpColor::red);
          vpDisplay::flush(image_);

          rclcpp::spin_some(this->get_node_base_interface());
          loop_rate_tracking.sleep();
          if (exiting())
            return;
        } while (!vpDisplay::getClick(image_, ip, button, false));
        ok = true;
      } else
        ok = true;
    } catch (const std::runtime_error &e) {
      mutex_.unlock();
      RCLCPP_ERROR_STREAM(this->get_logger(), "failed to initialize: " << e.what() << ", retrying...");
    } catch (const std::string &str) {
      mutex_.unlock();
      RCLCPP_ERROR_STREAM(this->get_logger(), "failed to initialize: " << str << ", retrying...");
    } catch (...) {
      mutex_.unlock();
      RCLCPP_ERROR(this->get_logger(), "failed to initialize, retrying...");
    }
    }

    RCLCPP_INFO_STREAM(this->get_logger(),"Initialization done, sending initial cMo:\n" << cMo);
    try
    {
    sendcMo(cMo);
    }
    catch(std::exception& e)
    {
    RCLCPP_ERROR_STREAM(this->get_logger(), "failed to send cMo: " << e.what());
    }
    catch(...)
    {
    RCLCPP_ERROR(this->get_logger(), "unknown error happened while sending the cMo, retrying...");
    }
}

TrackerClient::~TrackerClient()
{
  /* FIXME: RECONFIGURATION
  if(reconfigureSrv_ != NULL)
    delete reconfigureSrv_;

  if(reconfigureKltSrv_ != NULL)
    delete reconfigureKltSrv_;

  if(reconfigureEdgeSrv_ != NULL)
    delete reconfigureEdgeSrv_;
    */
}

void TrackerClient::sendcMo(const vpHomogeneousMatrix &cMo)
{
  rclcpp::Client<visp_tracker::srv::Init>::SharedPtr client =
      nodeHandle_->create_client<visp_tracker::srv::Init>(visp_tracker::init_service_);

  rclcpp::Client<visp_tracker::srv::Init>::SharedPtr clientViewer =
      nodeHandle_->create_client<visp_tracker::srv::Init>(visp_tracker::Init_viewer_service_);
  auto srv = std::make_shared<visp_tracker::srv::Init::Request>();

  // Load the model and send it to the parameter server.
  std::string modelDescription = fetchResource(modelPathAndExt_);
  rclcpp::Parameter str_param(model_description_param, modelDescription);
  nodeHandle_->set_parameter(str_param);

  vpHomogeneousMatrixToTransform(srv->initial_pose, cMo);

  convertVpMbTrackerToInitRequest(tracker_, srv);

  if (trackerType_ != "klt") {
    convertVpMeToInitRequest(movingEdge_, tracker_, srv);
  }

  if (trackerType_ != "mbt") {
    convertVpKltOpencvToInitRequest(kltTracker_, tracker_, srv);
  }

  rclcpp::Rate rate(1);
  // Wait for this service to be advertised and available.
  while (!client.waitForExistence()) {
    RCLCPP_INFO(this->get_logger(), "Waiting for the initialization service to become available.");
    rate.sleep();
  }

  auto client_result = client->async_send_request(srv);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), client_result) ==
      rclcpp::FutureReturnCode::SUCCESS)
      RCLCPP_INFO(this->get_logger(), "Tracker initialized with success.");
    else
      throw std::runtime_error("failed to initialize tracker.");
//  } else
//    throw std::runtime_error("failed to call service init");

  auto client_viewer_result = clientViewer->async_send_request(srv);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), client_viewer_result) ==
      rclcpp::FutureReturnCode::SUCCESS)
      RCLCPP_INFO(this->get_logger(), "Tracker Viewer initialized with success.");
    else
      throw std::runtime_error("failed to initialize tracker viewer.");
//  } else
//    RCLCPP_INFO(this->get_logger(), "No Tracker Viewer node to initialize from service");
}

void TrackerClient::loadModel()
{
  try {
    RCLCPP_DEBUG_STREAM(this->get_logger(), " Trying to load the model " << bModelPath_.native());

    std::string modelPath;
    std::ofstream modelStream;
    if (!makeModelFile(modelStream, bModelPath_.native(), modelPath))
      throw std::runtime_error("failed to retrieve model");

    tracker_.loadModel(modelPath);
    RCLCPP_INFO(this->get_logger(), "Model has been successfully loaded.");

    if (trackerType_ == "mbt") {
      RCLCPP_DEBUG_STREAM(this->get_logger(), " Nb faces: " << tracker_.getFaces().getPolygon().size());
      RCLCPP_DEBUG_STREAM(this->get_logger(), " Nb visible faces: " << tracker_.getFaces().getNbVisiblePolygon());

      std::list<vpMbtDistanceLine *> linesList;
      tracker_.getLline(linesList);
      RCLCPP_DEBUG_STREAM(this->get_logger(), " Nb line: " << linesList.size());
      // RCLCPP_DEBUG_STREAM(this->get_logger()," nline: " << tracker_.nline);
    } else if (trackerType_ == "klt") {
      RCLCPP_DEBUG_STREAM(this->get_logger(), " Nb faces: " << tracker_.getFaces().getPolygon().size());
      RCLCPP_DEBUG_STREAM(this->get_logger(), " Nb visible faces: " << tracker_.getFaces().getNbVisiblePolygon());
      RCLCPP_DEBUG_STREAM(this->get_logger(), " Nb KLT points: " << tracker_.getKltNbPoints());
    } else {
      RCLCPP_DEBUG_STREAM(this->get_logger(), " Nb hidden faces: " << tracker_.getFaces().getPolygon().size());
      RCLCPP_DEBUG_STREAM(this->get_logger(), " Nb visible faces: " << tracker_.getFaces().getNbVisiblePolygon());
      RCLCPP_DEBUG_STREAM(this->get_logger(), " Nb KLT points: " << tracker_.getKltNbPoints());

      std::list<vpMbtDistanceLine *> linesList;
      tracker_.getLline(linesList);
      RCLCPP_DEBUG_STREAM(this->get_logger(), " Nb line: " << linesList.size());
      // RCLCPP_DEBUG_STREAM(this->get_logger()," nline: " << tracker_.nline);
    }
  } catch (...) {
    std::string fmt = std::string("Failed to load the model ") + bModelPath_.string() + "\n" +
                      "Do you use resource_retriever syntax?\n" +
                      "I.e. replace /my/model/path by file:///my/model/path";
    throw std::runtime_error(fmt);
  }
}

vpHomogeneousMatrix TrackerClient::loadInitialPose()
{
  vpHomogeneousMatrix cMo;
  cMo.eye();

  std::string initialPose = getInitialPoseFileFromModelName(modelName_, modelPath_);
  std::string resource;
  try {
    resource = fetchResource(initialPose);
    std::stringstream file;
    file << resource;

    if (!file.good()) {
      RCLCPP_WARN_STREAM(this->get_logger(), "failed to load initial pose: " << initialPose << "\n"
                                                                             << "using identity as initial pose");
      return cMo;
    }

    vpPoseVector pose;
    for (unsigned i = 0; i < 6; ++i) {
      if (file.good())
        file >> pose[i];
      else {
        RCLCPP_WARN(this->get_logger(), "failed to parse initial pose file");
        return cMo;
      }
    }
    cMo.buildFrom(pose);
    return cMo;
  } catch (...) {
    // Failed to retrieve initial pose since model path starts with http://, package://, file:///
    // We try to read from temporary file /tmp/$USER/
    std::string username;
    vpIoTools::getUserName(username);

    std::string filename;
#if defined(_WIN32)
    filename = "C:/temp/" + username;
#else
    filename = "/tmp/" + username;
#endif
    filename += "/" + modelName_ + ".0.pos";
    RCLCPP_INFO_STREAM(this->get_logger(), "Try to read init pose from: " << filename);
    if (vpIoTools::checkFilename(filename)) {
      RCLCPP_INFO_STREAM(this->get_logger(), "Retrieve initial pose from: " << filename);
      std::ifstream in(filename.c_str());
      vpPoseVector pose;
      pose.load(in);
      cMo.buildFrom(pose);
      in.close();
    }

    return cMo;
  }
}

void TrackerClient::saveInitialPose(const vpHomogeneousMatrix &cMo)
{
  std::filesystem::path initialPose = getInitialPoseFileFromModelName(modelName_, modelPath_);
  std::ofstream file(initialPose);
  if (!file.good()) {
    // Failed to save initial pose since model path starts with http://, package://, file:///
    // We create a temporary file in /tmp/$USER/
    std::string username;
    vpIoTools::getUserName(username);

    // Create a log filename to save velocities...
    std::string logdirname;
#if defined(_WIN32)
    logdirname = "C:/temp/" + username;
#else
    logdirname = "/tmp/" + username;
#endif
    // Test if the output path exist. If no try to create it
    if (vpIoTools::checkDirectory(logdirname) == false) {
      try {
        vpIoTools::makeDirectory(logdirname);
      } catch (...) {
        RCLCPP_WARN_STREAM(this->get_logger(),
                           "Unable to create the folder " << logdirname << " to save the initial pose");
        return;
      }
    }
    std::string filename = logdirname + "/" + modelName_ + ".0.pos";
    RCLCPP_INFO_STREAM(this->get_logger(), "Save initial pose in: " << filename);
    std::fstream finitpos;
    finitpos.open(filename.c_str(), std::ios::out);
    vpPoseVector pose;
    pose.buildFrom(cMo);

    finitpos << pose;
    finitpos.close();
  } else {
    RCLCPP_INFO_STREAM(this->get_logger(), "Save initial pose in: " << initialPose);
    vpPoseVector pose;
    pose.buildFrom(cMo);
    file << pose;
  }
}

TrackerClient::points_t TrackerClient::loadInitializationPoints()
{
  points_t points;

  std::string init = getInitFileFromModelName(modelName_, modelPath_);
  std::string resource = fetchResource(init);
  std::stringstream file;
  file << resource;

  if (!file.good()) {
    std::string fmt("failed to load initialization points: ") + init ;
    throw std::runtime_error(fmt);
  }

  char c;
  // skip lines starting with # as comment
  file.get(c);
  while (!file.fail() && (c == '#')) {
    file.ignore(256, '\n');
    file.get(c);
  }
  file.unget();

  unsigned int npoints;
  file >> npoints;
  file.ignore(256, '\n'); // skip the rest of the line
  RCLCPP_INFO_STREAM(this->get_logger(), "Number of 3D points  " << npoints << "\n");

  if (npoints > 100000) {
    throw vpException(vpException::badValue, "Exceed the max number of points.");
  }

  vpPoint point;
  double X = 0., Y = 0., Z = 0.;
  for (unsigned int i = 0; i < npoints; i++) {
    // skip lines starting with # as comment
    file.get(c);
    while (!file.fail() && (c == '#')) {
      file.ignore(256, '\n');
      file.get(c);
    }
    file.unget();

    file >> X >> Y >> Z;
    file.ignore(256, '\n'); // skip the rest of the line

    point.setWorldCoordinates(X, Y, Z); // (X,Y,Z)
    points.push_back(point);
  }

  return points;
}

bool TrackerClient::validatePose(const vpHomogeneousMatrix &cMo)
{
  rclcpp::Rate loop_rate(200);
  vpImagePoint ip;
  vpMouseButton::vpMouseButtonType button = vpMouseButton::button1;
  vpDisplay::display(image_);
  tracker_.setDisplayFeatures(false);
  tracker_.display(image_, cMo, cameraParameters_, vpColor::green);
  vpDisplay::displayFrame(image_, cMo, cameraParameters_, frameSize_, vpColor::none, 2);
  vpDisplay::displayCharString(image_, 15, 10, "Left click to validate, right click to modify initial pose",
                               vpColor::red);
  vpDisplay::flush(image_);
  tracker_.setDisplayFeatures(true);

  do {
    this->get_node_base_interface(nodeHandle_);
    loop_rate.sleep();
    if (!rclcpp::ok())
      return false;
  } while (rclcpp::ok() && !vpDisplay::getClick(image_, ip, button, false));

  if (button == vpMouseButton::button1)
    return true;

  return false;
}

void TrackerClient::init()
{
  rclcpp::Rate loop_rate(200);
  vpHomogeneousMatrix cMo;
  vpImagePoint point(10, 10);

  cMo = loadInitialPose();
  tracker_.initFromPose(image_, cMo);

  // Show last cMo.
  vpImagePoint ip;
  vpMouseButton::vpMouseButtonType button = vpMouseButton::button1;

  if (validatePose(cMo)) {
    return;
  }
  vpDisplayX *initHelpDisplay = NULL;

  std::string helpImagePath = "";
  nodeHandlePrivate_->get_parameter<std::string>("help_image_path", helpImagePath);

  if (helpImagePath.empty()) {

    resource_retriever::MemoryResource resource;

    try {
      resource = resourceRetriever_.get(getHelpImageFileFromModelName(modelName_, modelPath_));
      char *tmpname = strdup("/tmp/tmpXXXXXX");
      if (mkdtemp(tmpname) == NULL) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to create the temporary directory: " << strerror(errno));
      } else {
        std::filesystem::path path(tmpname);
        path /= ("help.ppm");
        free(tmpname);

        helpImagePath = path.native();
        RCLCPP_INFO(this->get_logger(), "Copy help image from %s to %s",
                    getHelpImageFileFromModelName(modelName_, modelPath_).c_str(), helpImagePath.c_str());

        FILE *f = fopen(helpImagePath.c_str(), "w");
        fwrite(resource.data.get(), resource.size, 1, f);
        fclose(f);
      }
    } catch (...) {
    }

    RCLCPP_WARN_STREAM(this->get_logger(), "Auto detection of help file: " << helpImagePath);
  }

  if (!helpImagePath.empty()) {
    try {
      // check if the file exists
      if (!vpIoTools::checkFilename(helpImagePath)) {
        RCLCPP_WARN(this->get_logger(), "Error tracker initialization help image file \"%s\" doesn't exist",
                    helpImagePath.c_str());
      } else {
        RCLCPP_INFO_STREAM(this->get_logger(), "Load help image: " << helpImagePath);
        int winx = 0;
        int winy = 0;
#if VISP_VERSION_INT >= VP_VERSION_INT(2, 10, 0)
        winx = image_.display->getWindowXPosition();
        winy = image_.display->getWindowYPosition();
#endif
        initHelpDisplay = new vpDisplayX(winx + image_.getWidth() + 20, winy, "Init help image");

        vpImage<vpRGBa> initHelpImage;
        vpImageIo::read(initHelpImage, helpImagePath);
        initHelpDisplay->init(initHelpImage);
        vpDisplay::display(initHelpImage);
        vpDisplay::flush(initHelpImage);
      }
    } catch (vpException &e) {
      RCLCPP_WARN(this->get_logger(), "Error diplaying tracker initialization help image file \"%s\":\n%s",
                  helpImagePath.c_str(), e.what());
    }
  }

  points_t points = loadInitializationPoints();
  imagePoints_t imagePoints;

  bool done = false;
  while (!done) {
    vpDisplay::display(image_);
    vpDisplay::flush(image_);

    imagePoints.clear();
    for (unsigned i = 0; i < points.size(); ++i) {
      do {
        this->get_node_base_interface()(nodeHandle_);
        loop_rate.sleep();
        if (!rclcpp::ok())
          return;
      } while (rclcpp::ok() && !vpDisplay::getClick(image_, ip, button, false));

      imagePoints.push_back(ip);
      vpDisplay::displayCross(image_, imagePoints.back(), 5, vpColor::green);
      vpDisplay::flush(image_);
    }

    tracker_.initFromPoints(image_, imagePoints, points);
    tracker_.getPose(cMo);
    if (validatePose(cMo))
      done = true;
  }
  tracker_.initFromPose(image_, cMo);
  saveInitialPose(cMo);
  if (initHelpDisplay != NULL)
    delete initHelpDisplay;
}

void TrackerClient::initPoint(unsigned &i, points_t &points, imagePoints_t &imagePoints, rclcpp::Rate &rate,
                              vpPose &pose)
{
  vpImagePoint ip;
  double x = 0., y = 0.;
  std::string fmt("click on point ") + std::to_string(i + 1) + "/" + std::to_string(points.size()); // list points from 1 to n.

  // Click on the point.
  vpMouseButton::vpMouseButtonType button = vpMouseButton::button1;
  do {
    vpDisplay::display(image_);
    vpDisplay::displayCharString(image_, 15, 10, fmt.c_str(), vpColor::red);

    for (unsigned j = 0; j < imagePoints.size(); ++j)
      vpDisplay::displayCross(image_, imagePoints[j], 5, vpColor::green);

    vpDisplay::flush(image_);
    this->get_node_base_interface(nodeHandle_);
    rate.sleep();
    if (exiting())
      return;
  } while (!vpDisplay::getClick(image_, ip, button, false));

  imagePoints.push_back(ip);
  vpPixelMeterConversion::convertPoint(cameraParameters_, ip, x, y);
  points[i].set_x(x);
  points[i].set_y(y);
  pose.addPoint(points[i]);
}

void TrackerClient::waitForImage()
{
  rclcpp::Rate loop_rate(10);
  rclcpp::Clock clock;
  constexpr size_t LOG_THROTTLE_PERIOD = 10;
  while (!exiting() && (!image_.getWidth() || !image_.getHeight())) {
    RCLCPP_INFO_THROTTLE(rclcpp::get_logger("rclcpp"),clock,LOG_THROTTLE_PERIOD, "waiting for a rectified image...");
    rclcpp::spin_some(this->get_node_base_interface());
    loop_rate.sleep();
  }
}

std::string TrackerClient::fetchResource(const std::string &s)
{
  resource_retriever::MemoryResource resource = resourceRetriever_.get(s);
  std::string result;
  result.resize(resource.size);
  unsigned i = 0;
  for (; i < resource.size; ++i)
    result[i] = resource.data.get()[i];
  return result;
}

bool TrackerClient::makeModelFile(std::ofstream &modelStream, const std::string &resourcePath,
                                  std::string &fullModelPath)
{
  std::string modelExt_ = ".wrl";
  bool vrmlWorked = true;
  resource_retriever::MemoryResource resource;

  try {
    resource = resourceRetriever_.get(resourcePath + modelExt_);
  } catch (...) {
    vrmlWorked = false;
  }

  if (!vrmlWorked) {
    modelExt_ = ".cao";

    try {
      resource = resourceRetriever_.get(resourcePath + modelExt_);
    } catch (...) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "No .cao nor .wrl file found in: " << resourcePath);
    }
  }

  modelPathAndExt_ = resourcePath + modelExt_;

  // RCLCPP_WARN_STREAM(this->get_logger(),"Model file Make Client: " << resourcePath << modelExt_);

  // Crash after when model not found
  std::string result;
  result.resize(resource.size);
  unsigned i = 0;
  for (; i < resource.size; ++i)
    result[i] = resource.data.get()[i];
  result[resource.size];

  char *tmpname = strdup("/tmp/tmpXXXXXX");
  if (mkdtemp(tmpname) == NULL) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to create the temporary directory: " << strerror(errno));
    return false;
  }
  std::filesystem::path path(tmpname);
  path /= ("model" + modelExt_);
  free(tmpname);

  fullModelPath = path.native();

  // RCLCPP_WARN_STREAM(this->get_logger(),"Model file Make Client Full path tmp: " << fullModelPath );

  modelStream.open(path);
  if (!modelStream.good()) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to create the temporary file: " << path);
    return false;
  }
  modelStream << result;
  modelStream.flush();
  return true;
}
} // end of namespace visp_tracker.
