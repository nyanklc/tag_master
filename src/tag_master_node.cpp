#include <cv_bridge/cv_bridge.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Image.h>
#include <tag_master/tag_master.h>
// #include "../include/tag_master/tag_master.h"
// #include "../include/tag_master/apriltag_detector.h"

sensor_msgs::Image img;
bool img_received = false;

cv::Mat convertToMat(sensor_msgs::Image &i) {
  cv_bridge::CvImagePtr cv_img_msg = cv_bridge::toCvCopy(i, "bgr8");
  cv::Mat frame = cv_img_msg.get()->image;
  cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
  return frame;
}

void cameraCallback(const sensor_msgs::Image::ConstPtr &msg) {
  ROS_INFO("received image");
  img = sensor_msgs::Image(*msg);
  img_received = true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "tag_master_node");
  ros::NodeHandle nh;

  std::string camera_topic_name = nh.param<std::string>("camera_topic_name", "");
  ros::Subscriber camera_sub = nh.subscribe("/camera/color/image_raw", 10, cameraCallback);

  tag_master::TagMaster tm;

  /* test */
  std::string atag_det_name = "atag_det";
  auto atag_ptr = std::make_shared<tag_detection::AprilTagDetector>(
      atag_det_name, 1.0, 0.8, 8, false, 619.32689027, 617.14607294, 364.50967726, 264.79765919, 0.014, true, true);
  tm.addDetector<tag_detection::AprilTagDetector>(atag_ptr);

  ros::Rate r(10);
  while (1) {
    while (!img_received) {
      ros::spinOnce();
      r.sleep();
    }
    img_received = false;
    ROS_INFO("processing");
    auto frame = convertToMat(img);
    cv::Mat frame_color;
    cv::cvtColor(frame, frame_color, cv::COLOR_GRAY2BGR);
    tag_detection::DetectionOutput out = tm.runSingle<tag_detection::AprilTagDetector>("atag_det", frame);
    ROS_INFO("after run single");
    if (!out.yes) {
      ROS_INFO("Detector isn't defined!");
      continue;
    }

    ROS_INFO("out.detection size: %zd", out.detection.size());

    for (auto &detection : out.detection) {
      ROS_INFO("herhehrehrehhre");
      auto xd = tm.getDetector<tag_detection::AprilTagDetector>("atag_det");
      ROS_INFO("asdjajsdjasjdjasd");
      // xd->drawCubes(frame_color);
      ROS_INFO("yyyyyyyyyyyyyyyyyyyyyyyyyy");
    }
    
    cv::imshow("frame", frame_color);
    cv::waitKey(0);
  }

  return 0;
}