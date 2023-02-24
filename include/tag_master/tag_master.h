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

namespace tag_master {
class TagMaster {
 public:
  TagMaster();
  template <class T>
  void addDetector(std::shared_ptr<T> det);
  /* TODO: Implement */
  // void enableDetector(std::string name);
  // void disableDetector(std::string name);
  // template <class T>
  // void runContinuous(cv::Mat &frame);
  // void runContinuousAll();
  // template <class T>
  // void stopContinuous(std::string name);
  // void stopContinuousAll();
  template <class T>
  tag_detection::DetectionOutput runSingle(std::string name, cv::Mat &frame);
  std::vector<tag_detection::DetectionOutput> runSingleAll(cv::Mat &frame);
  template <class T>
  std::shared_ptr<T> getDetector(std::string name);

 private:
  std::shared_ptr<tag_detection::DetectorBase> findDetector(std::string name);
  std::vector<std::shared_ptr<tag_detection::DetectorBase>> detectors_;
};
}  // namespace tag_master

#endif
