#ifndef __TAG_MASTER_DETECTOR_BASE_H_
#define __TAG_MASTER_DETECTOR_BASE_H_

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <string>

namespace tag_detection
{
  /*
   * Define new detector and tag types here.
   * ####################################
   */
  enum DetectorType
  {
    detector_type_apriltag
  };

  enum TagType
  {
    tag_type_apriltag
  };

  enum TagGeometry
  {
    square,
    circle
  };
  /*
   * ####################################
   */

  struct TagSize
  {
    double width;
    double height;
    double radius;
  };

  struct TagShape
  {
    TagGeometry geometry;
    TagSize size;
  };

  struct Tag
  {
    TagType type;
    int id;
    geometry_msgs::TransformStamped tf;
    TagShape shape;
  };

  struct Detection
  {
    Tag tag;
  };

  struct DetectionProperties
  {
    // TODO: robot's mission/task when detected etc.
    geometry_msgs::PoseStamped robot_pose_when_detected;
  };

  struct DetectionOutput
  {
    std::vector<Detection> detection;
    std::vector<DetectionProperties> properties;
    bool success;
  };

  class DetectorBase
  {
  public:
    DetectorBase(std::string name, bool initial_enable);
    virtual std::string getName();
    virtual int getType();
    virtual void enable();
    virtual void disable();
    virtual bool process(cv::Mat &frame);
    virtual DetectionOutput output() = 0;
    virtual geometry_msgs::TransformStamped getTf() = 0;
    virtual void updateCameraParams(double fx, double fy, double cx, double cy) = 0;

  protected:
    std::string name_;
    DetectorType type_;
    bool enabled_;
  };
} // namespace tag_detection

#endif