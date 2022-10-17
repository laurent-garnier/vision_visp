#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "visp_tracker/tracker.h"

namespace visp_tracker
{
  class TrackerNodelet : public nodelet::Nodelet
  {
  public:
    TrackerNodelet ()
      : nodelet::Nodelet (),
        exiting_ (false),
        tracker_ (),
        thread_ ()
    {}

    ~TrackerNodelet ()
    {
      exiting_ = true;
      if (thread_)
        if (!thread_->timed_join (std::posix_time::seconds (2)))
          NODELET_WARN ("failed to join thread but continuing anyway");
      thread_.reset ();
      tracker_.reset ();
    }

    void spin ()
    {
      if (exiting_)
        return;
      tracker_ = std::shared_ptr<visp_tracker::Tracker>
          (new visp_tracker::Tracker (getMTNodeHandle (),
                                      getMTPrivateNodeHandle (),
                                      exiting_, 5u));
      while (rclcpp::ok() && !exiting_)
        tracker_->spin ();
    }

    virtual void onInit ()
    {
      NODELET_DEBUG ("Initializing nodelet...");
      exiting_ = false;
      thread_ = std::make_shared<std::thread>
          (std::bind (&TrackerNodelet::spin, this));
    }

  private:
    volatile bool exiting_;
    std::shared_ptr<visp_tracker::Tracker> tracker_;
    std::shared_ptr<std::thread> thread_;
  };

} // end of namespace visp_tracker.

PLUGINLIB_EXPORT_CLASS(visp_tracker::TrackerNodelet, nodelet::Nodelet);
