#ifndef __TAG_MASTER_H_
#define __TAG_MASTER_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
// NOTE: tag master doesn't know the types of detectors
#include <tag_master/detector_base.h>

#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace tag_master
{
  struct TagDescription
  {
    int id;
    std::string type;
    std::string pub_frame;
    std::vector<double> objvector;
  };

  class TagMaster
  {
  public:
    TagMaster();

    void addTagDescription(int _id, std::string _type, std::string _pub_frame, std::vector<double> _objvector);

    template <class T>
    void addDetector(std::shared_ptr<T> det)
    {
      detectors_.push_back(det);
    }

    void enableDetector(std::string name);
    void disableDetector(std::string name);
    void runSingle(std::string name, cv::Mat &frame);
    tag_detection::DetectionOutput getOutput(std::string name);
    std::shared_ptr<tag_detection::DetectorBase> getDetector(std::string name);

  private:
    std::shared_ptr<tag_detection::DetectorBase> findDetector(std::string name);
    std::vector<std::shared_ptr<tag_detection::DetectorBase>> detectors_;
    std::vector<TagDescription> tag_descriptions_;
  };
} // namespace tag_master

#endif
