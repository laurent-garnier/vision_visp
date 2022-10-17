#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "visp_tracker/tracker-viewer.h"

namespace visp_tracker
{
  class TrackerViewerNodelet : public nodelet::Nodelet
  {
  public:
    TrackerViewerNodelet ()
      : nodelet::Nodelet (),
        exiting_ (false),
        trackerViewer_ (),
        thread_ ()
    {}

    ~TrackerViewerNodelet ()
    {
      exiting_ = true;
      if (thread_)
      // TODO PORT ROS2
      // https://www.boost.org/doc/libs/1_53_0/doc/html/thread/thread_management.html#thread.thread_management.thread.timed_join
        if (!thread_->join (std::posix_time::seconds (2)))
          NODELET_WARN ("failed to join thread but continuing anyway");
      thread_.reset ();
      trackerViewer_.reset ();
    }

    void spin ()
    {
      trackerViewer_ = std::shared_ptr<visp_tracker::TrackerViewer>
          (new visp_tracker::TrackerViewer
           (getMTNodeHandle (),
            getMTPrivateNodeHandle (),
            exiting_, 5u));
      while (rclcpp::ok() && !exiting_)
        trackerViewer_->spin ();
    }

    virtual void onInit ()
    {
      NODELET_DEBUG ("Initializing nodelet...");
      exiting_ = false;
      thread_ = std::make_shared<std::thread>
          (std::bind (&TrackerViewerNodelet::spin, this));
    }
  private:
    volatile bool exiting_;
    std::shared_ptr<visp_tracker::TrackerViewer> trackerViewer_;
    std::shared_ptr<std::thread> thread_;
  };

} // end of namespace visp_tracker.

PLUGINLIB_EXPORT_CLASS(visp_tracker::TrackerViewerNodelet, nodelet::Nodelet);
