#ifndef __TAG_MASTER_APRILTAG_DETECTOR_H_
#define __TAG_MASTER_APRILTAG_DETECTOR_H_

#include <ros/ros.h>
#include <tag_master/detector_base.h>
#include <tag_master/utils.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/convert.h>
#include <Eigen/Dense>

extern "C"
{
#include <apriltag/apriltag.h>
#include <apriltag/apriltag_math.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag/common/getopt.h>
#include <apriltag/common/matd.h>
#include <apriltag/tagStandard41h12.h>
}

using namespace tag_utils;

namespace tag_detection
{
  class AprilTagDetector : public DetectorBase
  {
  public:
    AprilTagDetector(std::string name, double quad_decimate, double quad_sigma, int nthreads, bool refine_edges, double tag_size,
                     bool initial_enable = true, bool enable_orthogonal_iteration = false,
                     bool pose_estimation_enabled = true, double pose_estimation_error_max = 1.0E-04);
    virtual bool process(cv::Mat &frame) override;
    bool detect(cv::Mat &frame);
    bool estimatePose(cv::Mat &frame, double error_max = 1.0E-04);
    std::vector<apriltag_pose_t> getPoses();
    zarray *getDetections();
    void drawDetections(cv::Mat &frame);
    void drawCubes(cv::Mat &frame, std::string frame_id, visualization_msgs::MarkerArray &marker_array);
    virtual DetectionOutput output(std::vector<tag_utils::TagDescription> &tag_descriptions, tf2_ros::Buffer *tf2_buffer) override;
    geometry_msgs::PoseStamped getTagPoseDetection(Eigen::Matrix3d R, Eigen::Vector3d t);
    virtual geometry_msgs::PoseStamped getTagPose(geometry_msgs::PoseStamped &tag_pose_camera, std::string expected_frame_id, tf2_ros::Buffer *tf2_buffer);
    virtual geometry_msgs::PoseStamped getObjectPose(std::vector<tag_utils::TagDescription> &tag_descriptions, Tag &tag, tf2_ros::Buffer *tf2_buffer);
    virtual void enable() override;
    virtual void disable() override;
    void updateCameraParams(double fx, double fy, double cx, double cy) override;
    ~AprilTagDetector();

  protected:
    apriltag_family_t *family_;
    apriltag_detector_t *detector_;
    zarray_t *detections_;
    apriltag_detection_info_t detection_info_;
    std::vector<apriltag_pose_t> poses_;
    bool enable_orthogonal_iteration_;
    std::vector<std::pair<apriltag_pose_t, apriltag_pose_t>> poses_orthogonal_iteration_;
    bool pose_estimation_enabled_;
    double error_max_;
  };
} // namespace tag_detection

#endif
