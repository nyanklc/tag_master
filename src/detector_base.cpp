#include <tag_master/detector_base.h>

namespace tag_detection
{
  DetectorBase::DetectorBase(std::string name, bool initial_enable)
  {
    name_ = name;
    enabled_ = initial_enable;
  }

  bool DetectorBase::isEnabled()
  {
    return enabled_;
  }

  std::string DetectorBase::getName() { return name_; }

  int DetectorBase::getType()
  {
    return type_;
  }

  void DetectorBase::enable() { enabled_ = true; }

  void DetectorBase::disable() { enabled_ = false; }

  // just checks if the detector is enabled, passing the frame as an argument is not necessary but it is there for simpler
  // overriding
  bool DetectorBase::process(cv::Mat &frame)
  {
    // ROS_INFO("detectorbase process");
    if (!enabled_)
    {
      ROS_WARN("The detector %s is not enabled! Cannot process frame.", name_.c_str());
      return false;
    }
    return true;
  }

  void DetectorBase::setImageFrameId(std::string fid)
  {
    img_frame_id_ = fid;
  }

  void DetectorBase::setIdSizes(std::vector<std::pair<uint32_t, double>> list)
  {
    for (int i = 0; i < list.size(); i++)
      id_size_map_.emplace(list[i].first, list[i].second);
  }

  double DetectorBase::lookupTagSize(uint32_t id)
  {
    return id_size_map_[id];
  }

  bool DetectorBase::isIdSizesSet()
  {
    if (id_size_map_.size() > 0)
      return true;
    return false;
  }

} // namespace tag_detection
