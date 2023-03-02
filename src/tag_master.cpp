#include <tag_master/tag_master.h>

namespace tag_master
{
  TagMaster::TagMaster() {}

  std::shared_ptr<tag_detection::DetectorBase> TagMaster::findDetector(std::string name)
  {
    for (size_t i = 0; i < detectors_.size(); i++)
    {
      if (detectors_[i]->getName() == name)
        return detectors_[i];
    }
    ROS_WARN("Detector %s does not exist!", name.c_str());
    return nullptr;
  }
} // namespace tag_master