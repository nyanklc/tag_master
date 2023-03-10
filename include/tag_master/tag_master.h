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

namespace tag_master
{
  class TagMaster
  {
  public:
    TagMaster();

    template <class T>
    void addDetector(std::shared_ptr<T> det)
    {
      detectors_.push_back(det);
    }

    void enableDetector(std::string name);
    void disableDetector(std::string name);

    /* TODO: Implement */
    // template <class T>
    // void runContinuous(cv::Mat &frame);
    // void runContinuousAll();
    // template <class T>
    // void stopContinuous(std::string name);
    // void stopContinuousAll();

    void runSingle(std::string name, cv::Mat &frame);
    tag_detection::DetectionOutput getOutput(std::string name);
    std::shared_ptr<tag_detection::DetectorBase> getDetector(std::string name);
    void updateCameraParams(double fx, double fy, double cx, double cy);

  private:
    std::shared_ptr<tag_detection::DetectorBase> findDetector(std::string name);
    std::vector<std::shared_ptr<tag_detection::DetectorBase>> detectors_;
  };
} // namespace tag_master

#endif
