#ifndef __TAG_MASTER_DETECTOR_BASE_H_
#define __TAG_MASTER_DETECTOR_BASE_H_

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tag_master/utils.h>
#include "./tag_description.h"
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <opencv2/opencv.hpp>
#include <string>

namespace tag_detection
{
  // TODO: i probably forgot about setting all of these output properties etc. when returning detections from detectors

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
    geometry_msgs::PoseStamped pose; // pose of the tag after transformation
    TagShape shape;
    geometry_msgs::PoseStamped detection_pose; // pose of the tag in camera frame
  };

  struct Detection
  {
    Tag tag;
    geometry_msgs::PoseStamped obj_pose;
  };

  struct DetectionOutput
  {
    std::vector<Detection> detection;
    bool success;
  };

  class DetectorBase
  {
  public:
    DetectorBase(std::string name, bool initial_enable);
    virtual bool isEnabled();
    virtual std::string getName();
    virtual int getType();
    virtual void enable();
    virtual void disable();
    virtual bool process(cv::Mat &frame);
    virtual DetectionOutput output(std::vector<TagDescription> &tag_descriptions, tf2_ros::Buffer *tf2_buffer) = 0;
    virtual void updateCameraParams(double fx, double fy, double cx, double cy) = 0;
    void setImageFrameId(std::string fid);
    void setIdSizes(std::vector<std::pair<uint32_t, double>> list);
    double lookupTagSize(uint32_t id);
    bool isIdSizesSet();

  protected:
    std::string name_;
    DetectorType type_;
    bool enabled_;
    std::string img_frame_id_;
    std::map<uint32_t, double> id_size_map_;
  };
} // namespace tag_detection

#endif