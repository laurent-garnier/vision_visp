#include "cmd_line/cmd_line.h"
#include <iostream>
#include <fstream>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/mbt/vpMbGenericTracker.h>
#include <getopt.h>

void CmdLine::common(){

  const char* const short_opts = "n:o:c:w:h:d:f";
  const option long_opts[] = {
    {"dmtxonly", no_argument, nullptr, 'd'},
    {"video-camera", no_argument, nullptr, 'C'},
    {"video-source", required_argument, nullptr, 's'},
    {"data-directory", required_argument, nullptr, 'D'},
    {"video-input-path", required_argument, nullptr, 'J'},
    {"video-output-path", required_argument, nullptr, 'L'},
    {"single-image", required_argument, nullptr, 'I'},
    {"pattern-name", required_argument, nullptr, 'P'},
    {"detector-type", required_argument, nullptr, 'r'},
    {"detector-subtype", required_argument, nullptr, 'u'},
    {"tracker-type", required_argument, nullptr, 't'},
    {"verbose", required_argument, nullptr, 'v'},
    {"dmx-detector-timeout", required_argument, nullptr, 'T'},
    {"config-file", required_argument, nullptr, 'c'},
    {"show-fps", required_argument, nullptr, 'f'},
    {"show-plot", required_argument, nullptr, 'p'},
    {"code-message", required_argument, nullptr, 'm'},
    {"flashcode-coordinates", required_argument, nullptr, 'F'},
    {"inner-coordinates", required_argument, nullptr, 'i'},
    {"outer-coordinates", required_argument, nullptr, 'o'},
    {"variance-file", required_argument, nullptr, 'V'},
    {"variance-limit", required_argument, nullptr, 'l'},
    {"mbt-convergence-steps", required_argument, nullptr, 'S'},
    {"hinkley-range", required_argument, nullptr, 'H'},
    {"mbt-dynamic-range", required_argument, nullptr, 'R'},
    {"ad-hoc-recovery", required_argument, nullptr, 'W'},
    {"ad-hoc-recovery-display", required_argument, nullptr, 'D'},
    {"ad-hoc-recovery-ratio", required_argument, nullptr, 'y'},
    {"ad-hoc-recovery-size", required_argument, nullptr, 'w'},
    {"ad-hoc-recovery-threshold", required_argument, nullptr, 'Y'},
    {"log-checkpoints", required_argument, nullptr, 'g'},
    {"log-pose", required_argument, nullptr, 'q'},
    {nullptr, no_argument, nullptr, 0}
  };

  video_channel_ = "/dev/video1";
  data_dir_ = "./data/";
  input_file_pattern_ = "/images/%08d.jpg";
  pattern_name_ = "pattern";
  detector_type = "zbar";
  detector_subtype_ = "";
  tracker_type = "klt_mbt";
  verbose_ = false;
  dmx_timeout_ = 1000;
  config_file = "./data/config.cfg";
  show_fps_ = false;
  show_plot_ =  false;
  code_message_ = "";
  mbt_convergence_steps_ = 1;
  adhoc_recovery_ = true;
  adhoc_recovery_display_ = false;
  adhoc_recovery_ratio_ = 0.5;
  adhoc_recovery_size_ = 0.5;
  adhoc_recovery_treshold_ = 100;
  log_pose_ = false;
  log_checkpoints_ = false;
  help_ = false;
  
  general_options =
      std::string("General options\n")+
      "dmtxonly,d: only detect the datamatrix \n" +
      "video-camera,C: video from camera  \n" +
      "video-source,s: video source. For example /dev/video1 \n" +
      "data-directory,D: directory from which to load images \n" +
      "video-input-path,J: input video file path relative to the data directory\n" +
      "video-output-path,L: output video file path relative to the data directory\n" +
      "single-image,I: load this single image (relative to data dir)\n" +
      "pattern-name,P: name of xml,init and wrl files\n" +
      "detector-type,r: Type of your detector that will be used for initialisation/recovery. zbar for QRcodes and more, dmtx for flashcodes, april for April tags.\n" +
      "detector-subtype,u: Subtype of your detector that will be used for initialisation/recovery. For april detector : 36h11, 16h5, ...\n" +
      "tracker-type,t: Type of tracker. mbt_klt for hybrid: mbt+klt, mbt for model based, klt for klt-based\n" +
      "verbose,v: Enable or disable additional printings\n" +
      "dmx-detector-timeout,T: timeout for datamatrix detection in ms\n" +
      "config-file,c: config file for the program\n" +
      "show-fps,f: show framerate\n" +
      "show-plot,p: show variances graph\n" +
      "code-message,m: Target code message\n" +
      "help:h produce help message\n";

  
  configuration_options =
    std::string("Configuration") +
      "flashcode-coordinates,F: 3D coordinates of the flashcode in clockwise order\n" +
      "inner-coordinates,i: 3D coordinates of the inner region in clockwise order\n" +
      "outer-coordinates,o: 3D coordinates of the outer region in clockwise order\n" +
      "variance-file,V: file to store variance values\n" +
      "variance-limit,l: above this limit the tracker will be considered lost and the pattern will be detected with the flascode\n" +
      "mbt-convergence-steps,S: when a new model is detected, how many tracking iterations should the tracker perform so the model matches the projection.\n" +
      "hinkley-range,H: pair of alpha, delta values describing the two hinkley tresholds\n" +
      "mbt-dynamic-range,R: Adapt mbt range to symbol size. The width of the outer black corner is multiplied by this value to get the mbt range. Try 0.2\n" +
      "ad-hoc-recovery,W: Enable or disable ad-hoc recovery\n" +
      "ad-hoc-recovery-display,e: Enable or disable ad-hoc recovery display\n" +
      "ad-hoc-recovery-ratio,y: use ad-hoc recovery based on the model. The tracker will look for black pixels at ratio*[pattern size] from the center\n" +
      "ad-hoc-recovery-size,w: fraction of the black outer band size. The control points (those that should be black and in that way check tracking is still there).\n" +
      "ad-hoc-recovery-threshold,Y: Threshold over which the point is considered out of the black area of the object\n" +
      "log-checkpoints,g: log checkpoints in the log file\n" +
      "log-pose,q: log pose in the log file\n";

 while (true)
    {
      const auto opt = getopt_long(argc_, argv_, short_opts, long_opts, nullptr);

  if (-1 == opt)
	  break;
    
  switch (opt)
        {
	case 'd':
	  dmtxonly_ = true;
	  break;
	case 'C':
	  video_camera_ = true;
	  break;
	case 's':
	  video_channel_ = optarg;
	  break;
	case 'D':
	  data_dir_ = optarg;
	  break;
	case 'J':
	  input_file_pattern_ = optarg;
	  break;
	case 'L':
	  log_file_pattern_ = optarg;
	  break;
	case 'I':
	  single_image_name_ = optarg;
	  break;
	case 'P':
	  pattern_name_ = optarg;
	  break;
	case 'r':
	  detector_type = optarg;
	  break;
	case 'u':
	  detector_subtype_ = optarg;
	  break;
	case 't':
	  tracker_type = optarg;
	  break;
	case 'v':
	  verbose_ = optarg;
	  break;
	case 'T':
	  dmx_timeout_ = std::stoi(optarg);
	  break;
	case 'c':
	  config_file = optarg;
	  break;
	case 'f':
	  show_fps_ = optarg;
	  break;
	case 'p':
	  show_plot_ = optarg;
	  break;
	case 'm':
	  code_message_ = optarg;
	  break;
	case 'h':
	  help_ = true;
	  break;
	case 'F':
  // TODO : Port ROS2
  //	  flashcode_coordinates = optarg; 
	  break;
	case 'i':
// TODO : Port ROS2
//	  inner_coordinates = optarg;
	  break;
	case 'o':
// TODO : Port ROS2
//	  outer_coordinates = optarg;
	  break;
	case 'V':
	  var_file_ = optarg;
	  break;
	case 'l':
	  var_limit_ = std::stod(optarg);
	  break;
	case 'S':
// TODO : Port ROS2
//	  mbt_convergence_steps_ = std::stoi(optarg);
	  break;
	case 'H':
// TODO : Port ROS2
//	  hinkley_range_ = optarg;
	  break;
	case 'R':
	  mbt_dynamic_range_ = std::stod(optarg);
	  break;
	case 'W':
	  adhoc_recovery_ = optarg;
	  break;
	case 'e':
	  adhoc_recovery_display_ = optarg;
	  break;
	case 'y':
	  adhoc_recovery_ratio_ = std::stod(optarg);
	  break;
	case 'w':
	  adhoc_recovery_size_ = std::stod(optarg);
	  break;
	case 'Y':
	  adhoc_recovery_treshold_ = std::stoul(optarg);
	  break;
	case 'g':
	  log_checkpoints_ = true;
	  break;
	case 'q':
	  log_pose_ = true;
	  break;
      
  default:
	  std::cout << general_options << std::endl;
	  std::cout << configuration_options << std::endl;
	  break;

  }
  }
}

void CmdLine::loadConfig(std::string& config_file){
/*  std::ifstream in( config_file.c_str() );
  po::store(po::parse_config_file(in,prog_args,false), vm_);
  in.close();
*/
  std::cout << "Have to be implemented without boost in ROS2" << std::endl;

  return;

  for(unsigned int i =0;i<flashcode_coordinates.size()/3;i++){
    vpPoint p;
    p.setWorldCoordinates(flashcode_coordinates[i*3],flashcode_coordinates[i*3+1],flashcode_coordinates[i*3+2]);
    flashcode_points_3D_.push_back(p);
  }

  for(unsigned int i =0;i<inner_coordinates.size()/3;i++){
    vpPoint p;
    p.setWorldCoordinates(inner_coordinates[i*3],inner_coordinates[i*3+1],inner_coordinates[i*3+2]);
    inner_points_3D_.push_back(p);
  }

  for(unsigned int i =0;i<outer_coordinates.size()/3;i++){
    vpPoint p;
    p.setWorldCoordinates(outer_coordinates[i*3],outer_coordinates[i*3+1],outer_coordinates[i*3+2]);
    outer_points_3D_.push_back(p);
  }

  if(get_verbose()){
    std::cout << "Loaded " << flashcode_points_3D_.size() << " flashcode extremity points, " << inner_points_3D_.size() << " inner contour points and " << outer_points_3D_.size() << " outer contour points." << std::endl;
    std::cout << "Tracker set to:";
    switch(get_tracker_type()){
    case MBT:
      std::cout << "model based tracker";
      break;
    case KLT_MBT:
      std::cout << "hybrid (mbt+klt)";
      break;
    case KLT:
      std::cout << "tracker with klt points";
      break;
    }
    std::cout << std::endl;

    std::cout << "Detector set to:";
    switch(get_detector_type()){
    case ZBAR:
      std::cout << "QR code";
      break;
    case DMTX:
      std::cout << "Datamatrix (flashcode)";
      break;
    case APRIL:
      std::cout << "April tags";
      break;
    }
    std::cout << std::endl;

  }

  if(using_var_file())
    std::cout << "Using variance file:" << get_var_file() << std::endl;
  if (help_) {
	  std::cout << general_options << std::endl;
	  std::cout << configuration_options << std::endl;
    should_exit_ = true;

  }
}
CmdLine:: CmdLine(std::string& config_file) : should_exit_(false), code_message_index_(0) {
  this->config_file = config_file;
  common();
  loadConfig(config_file);
}
CmdLine:: CmdLine() : should_exit_(false), code_message_index_(0) {
}
void CmdLine:: init(std::string& config_file)
{
  this->config_file = config_file;
  common();
  loadConfig(config_file);
}

CmdLine:: CmdLine(int argc,char**argv) : should_exit_(false), code_message_index_(0) {
  common();
  argc_ = argc;
  argv_ = argv;

  if(get_verbose())
    std::cout << "Loading config from:" << config_file << std::endl;

  loadConfig(config_file);

}

vpCameraParameters CmdLine::get_cam_calib_params() const{
  vpCameraParameters cam;
  vpMbEdgeTracker tmptrack;
  tmptrack.loadConfigFile(get_xml_file() ); // Load the configuration of the tracker
  tmptrack.getCameraParameters(cam);
  return cam;
}

std::string CmdLine::get_log_file_pattern() const{
  return log_file_pattern_;
}

std::string CmdLine::get_input_file_pattern() const{
  return input_file_pattern_;
}

bool CmdLine:: show_plot() const{
  return show_plot_;
}

bool CmdLine:: using_hinkley() const{
  return hinkley_range_.size()==2;
}

double CmdLine:: get_hinkley_alpha() const{
  if(!using_hinkley())
    throw std::exception();
  return hinkley_range_[0];
}

double CmdLine:: get_hinkley_delta() const{
  if(!using_hinkley())
    throw std::exception();
  return hinkley_range_[1];
}

int CmdLine:: get_mbt_convergence_steps() const{
  return mbt_convergence_steps_;
}

double CmdLine:: get_mbt_dynamic_range() const{
  return mbt_dynamic_range_;
}

bool CmdLine:: using_mbt_dynamic_range(){
  return mbt_dynamic_range_ > 0;
}

double CmdLine:: get_var_limit() const{
  return var_limit_;
}

bool CmdLine:: using_var_limit() const{
  return var_limit_ >0;
}

std::string CmdLine:: get_var_file() const{
  return var_file_;
}

bool CmdLine:: using_var_file() const{
  return var_file_!= "";
}

bool CmdLine:: logging_video() const{
  return log_file_pattern_!="";
}

bool CmdLine:: dmtx_only() const{
  return dmtxonly_>0;
}

bool CmdLine:: should_exit() const{
  return should_exit_;
}

std::string CmdLine:: get_video_channel() const{
  return video_channel_;
}

bool CmdLine:: show_fps() const{
  return show_fps_;
}

bool CmdLine:: get_verbose() const{
  return verbose_;
}

int CmdLine:: get_dmx_timeout() const{
  return dmx_timeout_;
}

double CmdLine:: get_inner_ratio() const{
  return inner_ratio_;
}

double CmdLine:: get_outer_ratio() const{
  return outer_ratio_;
}

bool CmdLine:: using_data_dir() const{
  return data_dir_  != "";
}

bool CmdLine:: using_video_camera() const{
  return video_camera_>0;
}

std::string CmdLine:: get_data_dir() const{
  return data_dir_;
}

std::string CmdLine:: get_pattern_name() const{
  return pattern_name_;
}

std::string CmdLine:: get_mbt_cad_file() const{
  if(vpIoTools::checkFilename(get_data_dir() + get_pattern_name() + std::string(".wrl")))
    return get_data_dir() + get_pattern_name() + std::string(".wrl");
  else if (vpIoTools::checkFilename(get_data_dir() + get_pattern_name() + std::string(".cao")))
    return get_data_dir() + get_pattern_name() + std::string(".cao");
  else
    return get_data_dir() + get_pattern_name() + std::string(".wrl");
}

std::string CmdLine:: get_xml_file() const{
  return get_data_dir() + get_pattern_name() + std::string(".xml");
}

std::string CmdLine:: get_init_file() const{
  return get_data_dir() + get_pattern_name() + std::string(".init");
}

bool CmdLine:: using_single_image() const{
  return single_image_name_  != "";
}

std::string CmdLine:: get_single_image_path() const{
  return get_data_dir() + single_image_name_;
}

std::vector<vpPoint>& CmdLine:: get_flashcode_points_3D() {
  return flashcode_points_3D_;
}

std::vector<vpPoint>& CmdLine:: get_inner_points_3D() {
  return inner_points_3D_;
}

std::vector<vpPoint>& CmdLine:: get_outer_points_3D() {
  return outer_points_3D_;
}

CmdLine::DETECTOR_TYPE CmdLine:: get_detector_type() const{
  if(detector_type == "zbar")
    return CmdLine::ZBAR;
  else if(detector_type == "april")
    return CmdLine::APRIL;
  else
    return CmdLine::DMTX;
}

std::string CmdLine:: get_detector_subtype() const{
  return detector_subtype_;
}

CmdLine::TRACKER_TYPE CmdLine:: get_tracker_type() const{
  if(tracker_type == "mbt")
    return CmdLine::MBT;
  else if(tracker_type == "klt")
    return CmdLine::KLT;
  else
    return CmdLine::KLT_MBT;
}


double CmdLine:: get_adhoc_recovery_size() const{
  return adhoc_recovery_size_;
}

double CmdLine:: get_adhoc_recovery_ratio() const{
  return adhoc_recovery_ratio_;
}

unsigned int CmdLine:: get_adhoc_recovery_treshold() const{
  return adhoc_recovery_treshold_;
}

bool CmdLine:: get_adhoc_recovery_display() const {
  return adhoc_recovery_display_;
}

std::string CmdLine:: get_code_message() const {
  return code_message_;
}
size_t CmdLine:: get_code_message_index() const {
  return code_message_index_;
}

bool CmdLine:: using_adhoc_recovery() const{
  return adhoc_recovery_;
}

bool CmdLine:: log_checkpoints() const{
  return log_checkpoints_>0;
}

bool CmdLine:: log_pose() const{
  return log_pose_;
}

void CmdLine:: set_data_directory(std::string &dir){
  data_dir_ = dir;
}

void CmdLine:: set_pattern_name(std::string &name){
  pattern_name_ = name;
}
void CmdLine:: set_show_fps(bool show_fps){
  show_fps_ = show_fps;
}

void CmdLine:: set_code_message(const std::string &msg)
{
  code_message_ = msg;
}
void CmdLine:: set_code_message_index(const size_t &index)
{
  code_message_index_ = index;
}
