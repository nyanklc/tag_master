#include <tag_master/tag_master.h>

namespace tag_master
{
  TagMaster::TagMaster() {}

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
    // ROS_INFO("found detector: %s, its type: %s", det->getName().c_str(), typeid(det).name());
    if (det == nullptr)
    {
      ROS_WARN("Can't run a detector that doesn't exist.");
      return;
    }
    // ROS_INFO("calling process");
    std::cout << "#######################################\n";
    std::cout << "process output: " << det->process(frame) << std::endl;
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

  void TagMaster::updateCameraParams(double fx, double fy, double cx, double cy)
  {
    for (auto &det : detectors_)
      det->updateCameraParams(fx, fy, cx, cy);
  }

} // namespace tag_master