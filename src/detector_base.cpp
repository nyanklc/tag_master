#include <tag_master/detector_base.h>

namespace tag_detection {
DetectorBase::DetectorBase(std::string name, bool initial_enable) {
  name_ = name;
  enabled_ = initial_enable;
}

std::string DetectorBase::getName() {
  return name_;
}

void DetectorBase::enable() { enabled_ = true; }

void DetectorBase::disable() { enabled_ = false; }

bool DetectorBase::process(cv::Mat &frame) {
  if (!enabled_) {
    ROS_WARN("The detector %s is not enabled! Cannot process frame.", name_.c_str());
    return false;
  }
}

}  // namespace tag_detection