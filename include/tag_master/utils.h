#ifndef __TAG_MASTER_UTILS_H_
#define __TAG_MASTER_UTILS_H_

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
extern "C"
{
#include <apriltag/apriltag.h>
#include <apriltag/apriltag_math.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag/common/getopt.h>
#include <apriltag/common/matd.h>
#include <apriltag/tagStandard41h12.h>
}

#include <ros/publisher.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace tag_utils
{
  void rotatePoints(std::vector<cv::Point3f> &cube, cv::Mat R);
  void translatePoints(std::vector<cv::Point3f> &cube, cv::Mat t);
  // void printProjection(cv::Vec3f obj, cv::Vec2f img, std::string msg = "");
  // void printProjection(cv::Point3f obj, cv::Point2f img, std::string msg = "");
  void printMat(cv::Mat m, std::string msg = "");
  // void printVec3f(cv::Vec3f m, std::string msg = "");
  // void printPoint3f(cv::Point3f m, std::string msg = "");
  // void printPoint2f(cv::Point2f m, std::string msg = "");
  cv::Mat getDistortionMatrix();
  cv::Mat getCameraMatrix(double fx, double fy, double cx, double cy);
  std::vector<cv::Vec3f> convertToVec3fVec(std::vector<cv::Point3f> pVec);
  cv::Vec3f convertToVec3f(cv::Point3f &p);
  cv::Mat convertToMat(matd_t *m);
  cv::Mat convertToMat(std::vector<cv::Point3f> &v);
  cv::Mat convertToMat(std::vector<cv::Point2f> &v);
  cv::Vec3f convertToVec3f(matd_t *m);
  std::vector<cv::Point3f> defineCubeWithPoints(double size = 0.014);
  std::vector<cv::Vec3f> defineCubeWithVectors(double side_length);
  void drawCube(std::vector<cv::Point3f> &cube, cv::Mat &frame, cv::Mat &cameraMatrix, cv::Mat &distortionCoefficients,
                cv::Mat &rotationMatrix, cv::Mat &translationMatrix, cv::Scalar &color, std::vector<visualization_msgs::Marker> &marker_arr, std::string frame_id);
  visualization_msgs::Marker getCubeMarker(const std::vector<cv::Point3f> &points, const std::string &frame_id, cv::Scalar &color);
  visualization_msgs::Marker getCubeArrow(std::vector<cv::Point3f> &points, const cv::Mat &rotationMatrix, const std::string frame_id, const cv::Scalar &color);
  cv::Point3f getCubeCenter(std::vector<cv::Point3f> &points);
  std::array<float, 3> getRPY(const cv::Mat &R);
} // namespace tag_utils
#endif
