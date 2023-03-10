#include <tag_master/tag_master.h>

namespace tag_master
{
  TagMaster::TagMaster() {}

  void TagMaster::addTagDescription(int _id, std::string _type, std::string _pub_frame, std::vector<double> _objvector)
  {
    TagDescription td;
    td.id = _id;
    td.type = _type;
    td.pub_frame = _pub_frame;
    td.objvector = _objvector;
    tag_descriptions_.push_back(td);
  }

  void TagMaster::enableDetector(std::string name)
  {
    auto det = findDetector(name);
    det->enable();
  }

  void TagMaster::disableDetector(std::string name)
  {
    auto det = findDetector(name);
    det->disable();
  }

  void TagMaster::runSingle(std::string name, cv::Mat &frame)
  {
    auto det = findDetector(name);
    if (det == nullptr)
    {
      ROS_WARN("Can't run a detector that doesn't exist.");
      return;
    }
    det->process(frame);
    return;
  }

  tag_detection::DetectionOutput TagMaster::getOutput(std::string name)
  {
    auto det = findDetector(name);
    return det->output();
  }

  std::shared_ptr<tag_detection::DetectorBase> TagMaster::getDetector(std::string name)
  {
    return findDetector(name);
  }

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