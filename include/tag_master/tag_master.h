#ifndef __TAG_MASTER_H_
#define __TAG_MASTER_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tag_master/detector_base.h>
#include <tag_master/apriltag_detector.h>
#include <tag_master/TagPose.h>
#include <tag_master/utils.h>

#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace tag_master
{
  class TagMaster
  {
  public:
    TagMaster();

    void addTagDescription(int _id, std::string _type, std::string _pub_frame, std::string _obj_name, geometry_msgs::Transform _objtransform);
    void clearTagDescriptions();

    template <class T>
    void addDetector(std::shared_ptr<T> det)
    {
      det->setIdSizes(&id_size_map_);
      detectors_.push_back(det);
    }

    void enableDetector(std::string name, bool enable);
    void enableDetector(std::string name);
    void disableDetector(std::string name);
    void runSingle(std::string name, cv::Mat &frame);
    void runAll(cv::Mat &frame);
    void publishTags(ros::Publisher &tag_pub, ros::Publisher &obj_pub, ros::Publisher &tag_vis_pub, ros::Publisher &obj_vis_pub, ros::Publisher &original_vis_pub);
    tag_detection::DetectionOutput getOutput(std::string name);
    std::vector<tag_detection::DetectionOutput> getOutputs();
    std::shared_ptr<tag_detection::DetectorBase> getDetector(std::string name);
    void updateCameraParams(double fx, double fy, double cx, double cy);
    void debugOutput();
    void setBuffer(tf2_ros::Buffer *buf);
    void setImageFrameId(std::string fid);
    void setIdSizes(std::vector<std::pair<uint32_t, double>> id_size_pairs);

  private:
    std::shared_ptr<tag_detection::DetectorBase> findDetector(std::string name);
    std::vector<std::shared_ptr<tag_detection::DetectorBase>> detectors_;
    std::vector<TagDescription> tag_descriptions_;
    tf2_ros::Buffer *tf2_buffer_;
    std::string img_frame_id_;
    std::map<uint32_t, double> id_size_map_;
  };
} // namespace tag_master

#endif
