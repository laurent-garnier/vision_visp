#include "visp_auto_tracker/node.h"
#include "visp_auto_tracker/names.h"

//command line parameters
#include "cmd_line/cmd_line.h"


//tracking
#include "libauto_tracker/tracking.h"
#include "libauto_tracker/threading.h"
#include "libauto_tracker/events.h"

//#include "visp_tracker/MovingEdgeSites.h"
//#include "visp_tracker/KltPoints.h"

//visp includes
#include <visp3/gui/vpDisplayX.h>
#include <visp3/mbt/vpMbGenericTracker.h>
#include <visp3/core/vpTime.h>

//detectors
#if VISP_VERSION_INT < VP_VERSION_INT(2,10,0)
#  include "detectors/datamatrix/detector.h"
#  include "detectors/qrcode/detector.h"
#else
#  include <visp3/detection/vpDetectorDataMatrixCode.h>
#  include <visp3/detection/vpDetectorQRCode.h>
#  include <visp3/detection/vpDetectorAprilTag.h>
#endif

#include <visp_bridge/camera.h>
#include <visp_bridge/image.h>
#include <visp_bridge/3dpose.h>

#include "libauto_tracker/tracking.h"

#include <resource_retriever/retriever.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"

namespace visp_auto_tracker{
  Node::Node() :
    n_("~"),
    queue_size_(1),
    tracker_config_path_(),
    model_description_(),
    model_path_(),
    model_name_(),
    code_message_(),
    tracker_ref_frame_(),
    debug_display_(false),
    I_(),
    image_header_(),
    got_image_(false),
    cam_(),
    t_(NULL) {
    //get the tracker configuration file
    //this file contains all of the tracker's parameters, they are not passed to ros directly.
    if (tracker_config_path_!= "") {
      n_.declare_parameter<std::string>("tracker_config_path", tracker_config_path_, "");
    } else {
      n_.declare_parameter<std::string>("tracker_config_path", "");
    }
    if (debug_display_!= NULL) {
      n_.declare_parameter<bool>("debug_display", debug_display_);
    } else {
      n_.declare_parameter<<bool>("debug_display", false);
    }
    std::string model_full_path;
    if (model_path_!= "") {
      n_.declare_parameter<std::string>("model_path", model_path_);
    } else {
      n_.declare_parameter<std::string>("model_path", "");
    }
    if (model_name_!= "") {
      n_.declare_parameter<std::string>("model_name", model_name_);
    } else {
      n_.declare_parameter<std::string>("model_name", "");
    }
    if (code_message_!= "") {
      n_.declare_parameter<std::string>("code_message", code_message_);
    } else {
      n_.declare_parameter<std::string>("code_message", "");
    }
    if (tracker_ref_frame_!= "") {
      n_.declare_parameter<std::string>("tracker_ref_frame", tracker_ref_frame_);
    } else {
      n_.declare_parameter<std::string>("tracker_ref_frame", "/map");
    }
    model_path_= model_path_[model_path_.length()-1]=='/'?model_path_:model_path_+std::string("/");
    model_full_path = model_path_+model_name_;
    tracker_config_path_ = model_full_path+".cfg";
    RCLCPP_INFO(this->get_logger(),"model full path=%s",model_full_path.c_str());

    //Parse command line arguments from config file (as ros param)
    cmd_.init(tracker_config_path_);
    cmd_.set_data_directory(model_path_); //force data path
    cmd_.set_pattern_name(model_name_); //force model name
    cmd_.set_show_fps(false);
    if (! code_message_.empty()) {
      RCLCPP_WARN_STREAM(this->get_logger(),"Track only code with message: \"" << code_message_ << "\"");
      cmd_.set_code_message(code_message_);
    }

    resource_retriever::Retriever r;
    resource_retriever::MemoryResource res;
    try {
      res = r.get(std::string("file://")+cmd_.get_mbt_cad_file());
    }
    catch(...) {
      RCLCPP_ERROR_STREAM(this->get_logger(),"Unable to read wrl or cao model file as resource: " << std::string("file://")+cmd_.get_mbt_cad_file());
    }

    model_description_.resize(res.size);
    for (unsigned int i=0; i < res.size; ++i)
      model_description_[i] = res.data.get()[i];

    RCLCPP_INFO(this->get_logger(),"Model content=%s",model_description_.c_str());

    rclcpp::Parameter str_param("/model_description", model_description_);
    n_->set_parameter(str_param);
  }

  void Node::waitForImage(){
    while ( rclcpp::ok()){
      if(got_image_) return;
      ros::spin_some(n_);
    }
  }

  //records last recieved image
  void Node::frameCallback(const sensor_msgs::msg::Image::ConstPtr& image, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_info){
    std::mutex::scoped_lock(lock_);
    image_header_ = image->header;
    I_ = visp_bridge::toVispImageRGBa(*image); //make sure the image isn't worked on by locking a mutex
    cam_ = visp_bridge::toVispCameraParameters(*cam_info);

    got_image_ = true;
  }

  void Node::spin(){

    if(cmd_.should_exit()) return; //exit if needed

    vpMbTracker* tracker; //mb-tracker will be chosen according to config

    //create display
    vpDisplayX* d = NULL;
    if(debug_display_)
      d = new vpDisplayX();

    //init detector based on user preference
#if VISP_VERSION_INT < VP_VERSION_INT(2,10,0)
    detectors::DetectorBase* detector = NULL;
#if defined(VISP_HAVE_ZBAR) && defined(VISP_HAVE_DMTX)
    if (cmd_.get_detector_type() == CmdLine::ZBAR)
      detector = new detectors::qrcode::Detector;
    else if(cmd_.get_detector_type() == CmdLine::DMTX)
      detector = new detectors::datamatrix::Detector;
#elif defined(VISP_HAVE_ZBAR)
    detector = new detectors::qrcode::Detector;
#elif defined(VISP_HAVE_DMTX)
    detector = new detectors::datamatrix::Detector;
#endif
#else // ViSP >= 2.10.0. In that case we use the detectors from ViSP
    vpDetectorBase *detector = NULL;
#if defined(VISP_HAVE_ZBAR) && !defined(VISP_HAVE_DMTX) && !defined(VISP_HAVE_APRILTAG)
    detector = new vpDetectorQRCode;
#elif defined(VISP_HAVE_DMTX) && !defined(VISP_HAVE_ZBAR) && !defined(VISP_HAVE_APRILTAG)
    detector = new vpDetectorDataMatrixCode;
#elif defined(VISP_HAVE_APRILTAG) && !defined(VISP_HAVE_ZBAR) && !defined(VISP_HAVE_DMTX)
    detector = new vpDetectorAprilTag;
#elif defined(VISP_HAVE_ZBAR) && defined(VISP_HAVE_DMTX)
    if (cmd_.get_detector_type() == CmdLine::ZBAR)
      detector = new vpDetectorQRCode;
    else if(cmd_.get_detector_type() == CmdLine::DMTX)
      detector = new vpDetectorDataMatrixCode;
    else if(cmd_.get_detector_type() == CmdLine::APRIL) {
      vpDetectorAprilTag::vpAprilTagFamily tag_family = vpDetectorAprilTag::vpAprilTagFamily::TAG_36h11;
      std::string tag_family_str = cmd_.get_detector_subtype();
      if(tag_family_str.find("16h5") != std::string::npos)
        tag_family = vpDetectorAprilTag::vpAprilTagFamily::TAG_16h5;
      else if(tag_family_str.find("25h7") != std::string::npos)
        tag_family = vpDetectorAprilTag::vpAprilTagFamily::TAG_25h7;
      else if(tag_family_str.find("25h9") != std::string::npos)
        tag_family = vpDetectorAprilTag::vpAprilTagFamily::TAG_25h9;
      else if(tag_family_str.find("36ARTOOLKIT") != std::string::npos)
        tag_family = vpDetectorAprilTag::vpAprilTagFamily::TAG_36ARTOOLKIT;
      else if(tag_family_str.find("36h10") != std::string::npos)
        tag_family = vpDetectorAprilTag::vpAprilTagFamily::TAG_36h10;
      else if(tag_family_str.find("36h11") != std::string::npos)
        tag_family = vpDetectorAprilTag::vpAprilTagFamily::TAG_36h11;
      detector = new vpDetectorAprilTag(tag_family);
    }
#endif
#endif

    // Use the best tracker
    int trackerType = vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER;
    tracker = new vpMbGenericTracker(1, trackerType);
    tracker->setCameraParameters(cam_);
    tracker->setDisplayFeatures(true);

    //compile detectors and paramters into the automatic tracker.
    t_ = new tracking::Tracker(cmd_, detector, tracker, debug_display_);
    t_->start(); //start the state machine

    //subscribe to ros topics and prepare a publisher that will publish the pose
    message_filters::Subscriber<sensor_msgs::msg::image> raw_image_subscriber(n_, image_topic, queue_size_);
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_subscriber(n_, camera_info_topic, queue_size_);
    message_filters::TimeSynchronizer<sensor_msgs::msg::image, sensor_msgs::msg::CameraInfo> image_info_sync(raw_image_subscriber, camera_info_subscriber, queue_size_);
    image_info_sync.registerCallback(boost::bind(&Node::frameCallback,this, std::placeholders::_1, std::placeholders::_2));
    // FIX TODO
    rclcpp::Publisher object_pose_publisher<geometry_msgs::PoseStamped>::SharedPtr = n_.advertise<geometry_msgs::PoseStamped>(object_position_topic, queue_size_);
    rclcpp::Publisher object_pose_covariance_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr = n_.advertise<geometry_msgs::msg::PoseWithCovarianceStamped>(object_position_covariance_topic, queue_size_);
    rclcpp::Publisher moving_edge_sites_publisher<visp_tracker::msg::MovingEdgeSites>::SharedPtr = n_.advertise<visp_tracker::msg::MovingEdgeSites>(moving_edge_sites_topic, queue_size_);
    rclcpp::Publisher klt_points_publisher<visp_tracker::msg::KltPoints>::SharedPtr = n_.advertise<visp_tracker::msg::KltPoints>(klt_points_topic, queue_size_);
    rclcpp::Publisher status_publisher<std_msgs::Int8>::SharedPtr = n_.advertise<std_msgs::Int8>(status_topic, queue_size_);
    rclcpp::Publisher<std_msgs::String>::::SharedPtr code_message_publisher = n_.advertise<std_msgs::String>(code_message_topic, queue_size_);

    //wait for an image to be ready
    waitForImage();
    {
      //when an image is ready tell the tracker to start searching for patterns
      std::mutex::scoped_lock(lock_);
      if(debug_display_) {
        d->init(I_); //also init display
        vpDisplay::setTitle(I_, "visp_auto_tracker debug display");
      }

      t_->process_event(tracking::select_input(I_));
    }

    unsigned int iter=0;
    geometry_msgs::PoseStamped ps;
    geometry_msgs::msg::PoseWithCovarianceStamped ps_cov;

    rclcpp::Rate rate(30); //init 25fps publishing frequency
    while(rclcpp::ok()){
      double t = vpTime::measureTimeMs();
      std::mutex::scoped_lock(lock_);
      //process the new frame with the tracker
      t_->process_event(tracking::input_ready(I_,cam_,iter));
      //When the tracker is tracking, it's in the tracking::TrackModel state
      //Access this state and broadcast the pose
      tracking::TrackModel& track_model = t_->get_state<tracking::TrackModel&>();

      ps.pose = visp_bridge::toGeometryMsgsPose(track_model.cMo); //convert

      // Publish resulting pose.
      if (object_pose_publisher.getNumSubscribers	() > 0)
      {
        ps.header = image_header_;
        ps.header.frame_id = tracker_ref_frame_;
        object_pose_publisher.publish(ps);
      }

      // Publish resulting pose covariance matrix.
      if (object_pose_covariance_publisher.getNumSubscribers	() > 0)
      {
        ps_cov.pose.pose = ps.pose;
        ps_cov.header = image_header_;
        ps_cov.header.frame_id = tracker_ref_frame_;

        for (unsigned i = 0; i < track_model.covariance.getRows(); ++i)
        {
          for (unsigned j = 0; j < track_model.covariance.getCols(); ++j)
          {
            unsigned idx = i * track_model.covariance.getCols() + j;
            if (idx >= 36)
              continue;
            ps_cov.pose.covariance[idx] = track_model.covariance[i][j];
          }
        }

        object_pose_covariance_publisher.publish(ps_cov);
      }

      // Publish state machine status.
      if (status_publisher.getNumSubscribers	() > 0)
      {
        std_msgs::Int8 status;
        status.data = (unsigned char)(*(t_->current_state()));
        status_publisher.publish(status);
      }

      // Publish moving edge sites.
      if (moving_edge_sites_publisher.getNumSubscribers	() > 0)
      {
        visp_tracker::MovingEdgeSitesPtr sites (new visp_tracker::msg::MovingEdgeSites);
        // Test if we are in the state tracking::TrackModel. In that case the pose is good;
        // we can send the moving edges. Otherwise we send an empty list of features
        if (*(t_->current_state()) == 3) {
          t_->updateMovingEdgeSites(sites);
        }

        sites->header = image_header_;
        moving_edge_sites_publisher.publish(sites);
      }

      // Publish KLT points.
      if (klt_points_publisher.getNumSubscribers	() > 0)
      {
        visp_tracker::KltPointsPtr klt (new visp_tracker::msg::KltPoints);
        // Test if we are in the state tracking::TrackModel. In that case the pose is good;
        // we can send the klt points. Otherwise we send an empty list of features
        if (*(t_->current_state()) == 3) {
          t_->updateKltPoints(klt);
        }
        klt->header = image_header_;
        klt_points_publisher.publish(klt);
      }

      if (code_message_publisher.getNumSubscribers() > 0)
      {
        std_msgs::String message;
        if (*(t_->current_state()) == 3) { // Tracking successful
#if VISP_VERSION_INT < VP_VERSION_INT(2,10,0)
          message.data = detector->get_message();
#else
          message.data = detector->getMessage( cmd_.get_code_message_index() );
#endif
        }
        else {
          message.data = std::string();
        }
        code_message_publisher.publish(message);
        RCLCPP_INFO_STREAM(this->get_logger(),("Code with message \"" <<  message.data << "\" under tracking");
      }

      rclcpp::spin_some(this);
      rate.sleep();
      if (cmd_.show_fps())
        std::cout << "Tracking done in " << vpTime::measureTimeMs() - t << " ms" << std::endl;
    }
    t_->process_event(tracking::finished());
    if(debug_display_) {
      delete d;
    }
    delete tracker;
  }
}
