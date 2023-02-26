#ifndef __TAG_MASTER_H_
#define __TAG_MASTER_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tag_master/apriltag_detector.h>
#include <tag_master/detector_base.h>

#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <vector>

/* test */
#include <typeinfo>

namespace tag_master {
class TagMaster {
 public:
  TagMaster();

  /* To add a new detector, pass in a shared pointer of that detector object */
  template <class T>
  void addDetector(std::shared_ptr<T> det) {
    detectors_.push_back(std::static_pointer_cast<tag_detection::DetectorBase>(det));
  }

  template <class T>
  void enableDetector(std::string name) {
    std::shared_ptr<T> det = std::dynamic_pointer_cast<T>(findDetector(name));
    det->enable();
  }

  template <class T>
  void disableDetector(std::string name) {
    std::shared_ptr<T> det = std::dynamic_pointer_cast<T>(findDetector(name));
    det->disable();
  }

  /* TODO: Implement */
  // template <class T>
  // void runContinuous(cv::Mat &frame);
  // void runContinuousAll();
  // template <class T>
  // void stopContinuous(std::string name);
  // void stopContinuousAll();

  template <class T>
  tag_detection::DetectionOutput runSingle(std::string name, cv::Mat &frame) {
    auto det = findDetector(name);
    ROS_INFO("found detector base, name: %s", det->getName().c_str());
    if (det == nullptr) {
      ROS_WARN("Can't run a detector that doesn't exist.");
      tag_detection::DetectionOutput out;
      out.yes = false;
      return out;
    }
    std::shared_ptr<T> det_derived = std::dynamic_pointer_cast<T>(det);
    ROS_INFO("found detector derived, name: %s, type: %d", det_derived->getName().c_str(), det_derived->getType());
    ROS_INFO("processing frame");
    det_derived->process(frame);
    ROS_INFO("processed frame, outputting");
    std::cout << "det: " << typeid(det).name() << std::endl;
    std::cout << "det_derived: " << typeid(det_derived).name() << std::endl;
    tag_detection::DetectionOutput out = det_derived->output();
    out.yes = true;
    return out;
  }

  std::vector<tag_detection::DetectionOutput> runSingleAll(cv::Mat &frame);

  template <class T>
  std::shared_ptr<T> getDetector(std::string name) {
    return std::dynamic_pointer_cast<T>(findDetector(name));
  }

 private:
  std::shared_ptr<tag_detection::DetectorBase> findDetector(std::string name);
  std::vector<std::shared_ptr<tag_detection::DetectorBase>> detectors_;
};
}  // namespace tag_master

#endif
