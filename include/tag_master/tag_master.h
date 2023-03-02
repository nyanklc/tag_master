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

    void addDetector(std::shared_ptr<tag_detection::DetectorBase> det)
    {
      detectors_.push_back(det);
    }

    template <class T>
    void enableDetector(std::string name)
    {
      std::shared_ptr<T> det = std::dynamic_pointer_cast<T>(findDetector(name));
      det->enable();
    }

    template <class T>
    void disableDetector(std::string name)
    {
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
    void runSingle(std::string name, cv::Mat &frame)
    {
      // TODO: maybe try without dynamic casting?
      std::shared_ptr<T> det = std::dynamic_pointer_cast<T>(findDetector(name));
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

    template <class T>
    tag_detection::DetectionOutput getOutput(std::string name)
    {
      std::shared_ptr<T> det = std::dynamic_pointer_cast<T>(findDetector(name));
      return det->output();
    }

    std::shared_ptr<tag_detection::DetectorBase> getDetector(std::string name)
    {
      return findDetector(name);
    }

  private:
    std::shared_ptr<tag_detection::DetectorBase> findDetector(std::string name);
    std::vector<std::shared_ptr<tag_detection::DetectorBase>> detectors_;
  };
} // namespace tag_master

#endif
