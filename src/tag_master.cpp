#include <tag_master/tag_master.h>

namespace tag_master {
TagMaster::TagMaster() {}

template <class T>
void TagMaster::addDetector(std::shared_ptr<T> det) {
  detectors_.push_back(det);
}

std::shared_ptr<tag_detection::DetectorBase> TagMaster::findDetector(std::string name) {
  for (size_t i = 0; i < detectors_.size(); i++) {
    if (detectors_[i]->getName() == name) return detectors_[i];
  }
  ROS_WARN("Detector %s does not exist!", name.c_str());
  return nullptr;
}

template <class T>
tag_detection::DetectionOutput TagMaster::runSingle(std::string name, cv::Mat &frame) {
  auto det = findDetector(name);
  if (det == nullptr) {
    ROS_WARN("Can't run a detector that doesn't exist.");
    tag_detection::DetectionOutput out;
    out.yes = false;
    return out;
  }
  std::shared_ptr<T> det_derived = std::dynamic_pointer_cast<std::shared_ptr<T>>(det);
  det_derived->process(frame);
  tag_detection::DetectionOutput out = det_derived->getOutput();
  out.yes = true;
  return out;
}

template <class T>
std::shared_ptr<T> TagMaster::getDetector(std::string name) {
  return std::dynamic_pointer_cast<std::shared_ptr<T>>(findDetector(name));
}
}  // namespace tag_master